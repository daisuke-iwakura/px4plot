% THIS IS ALPHA VERSION. THIS MIGHT CHANGE IN FUTURE VERSION.
%
% -- LOG = decode_px4log(FILEPATH)
%     Decodes a px4log
%
%     Copyright (c) 2017, Daisuke Iwakura
%     All rights reserved.

%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions 
% are met:
%
% 1. Redistributions of source code must retain the above copyright
% notice, this list of conditions and the following disclaimer.
%
% 2. Redistributions in binary form must reproduce the above copyright
% notice, this list of conditions and the following disclaimer in the
% documentation and/or other materials provided with the distribution.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
% "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
% LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
% A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
% HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
% SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
% TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
% PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
% LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
% NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
% SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function result = decode_px4log(filepath)

% define first for better performance
temp(1).time     = [];
temp(1).buff     = [];
temp(1:127) = temp(1);
% don't use struct to keep performance
temp_count(1:127)    = 0; % number of messages stored in buffer
temp_len_body(1:127) = 0; % length of message excluding header
temp_size(1:127)     = 0; % size of buffer

ALLOC_UNIT = 256; % size of block allocation

LOG_PACKET_HEADER_LEN = 3;

HEAD_BYTE1 = 163; % 0xA3
HEAD_BYTE2 = 149; % 0x95

LOG_FORMAT_MSG     = 128; % 0x80
LOG_TIME_MSG       = 129; % 0x81
LOG_VER_MSG        = 130; % 0x82
LOG_PARM_MSG       = 131; % 0x83
LOG_FORMAT_MSG_LEN = 86;
LOG_TIME_MSG_LEN   = 8;
LOG_VER_MSG_LEN    = 80;
LOG_PARM_MSG_LEN   = 20;

% performance check
perf_check = false;

if perf_check
    disp('---Start---');
    tic
end

%%
if ~exist(filepath, 'file')
    error('file not found');
end

fid = fopen(filepath);
buff = fread(fid, 'uint8=>uint8')';
fclose(fid);

if perf_check
    toc
    tic
    disp('Check point 1');
end

%%

time_cur   = 0;  % current time
time(1:ALLOC_UNIT) = 0;
time_count = 0;  % time data count

parm_count = 0;
parm_null.name = '';
parm_null.value = 0;
parm(1:ALLOC_UNIT) = parm_null;

ver.arch   = '';
ver.fw_git = '';

i = 1;
i_end = 1 + length(buff) - LOG_PACKET_HEADER_LEN;

while i <= i_end
    if buff(i) == HEAD_BYTE1 && buff(i+1) == HEAD_BYTE2   
        msg_type = buff(i+2);
        i = i + LOG_PACKET_HEADER_LEN;

        if msg_type < 128
            % business-level messages, ID < 0x80
            
            % make 2D array for vector processing
            count = temp_count(msg_type) + 1;
            temp_count(msg_type) = count;
            temp(msg_type).time(count) = time_cur;
            i_next = i + temp_len_body(msg_type);
            temp(msg_type).buff(count,:) = buff(i:i_next - 1);
            i = i_next;
            
            % block reallocation
            if count == temp_size(msg_type)
                temp(msg_type).time(count+ALLOC_UNIT) = 0;
                temp(msg_type).buff(count+ALLOC_UNIT,:) = 0;
                temp_size(msg_type) = count + ALLOC_UNIT;
            end
            
        elseif msg_type == LOG_TIME_MSG
            time_cur = decode_time(buff, i);
            i = i + LOG_TIME_MSG_LEN;
            
            time_count = time_count + 1;
            time(time_count) = time_cur;
            
            % block reallocation
            if mod(time_count, ALLOC_UNIT) == 0
                time(time_count+ALLOC_UNIT) = 0;
            end
            
        elseif msg_type == LOG_PARM_MSG
            parm_count = parm_count + 1;
            parm(parm_count) = decode_parm(buff, i);
            i = i + LOG_PARM_MSG_LEN;
            
            % block reallocation
            if mod(parm_count, ALLOC_UNIT) == 0
                parm(parm_count+1:parm_count+ALLOC_UNIT) = parm_null;
            end
            
        elseif msg_type == LOG_FORMAT_MSG
            fm = decode_format(buff, i);
            fm.parser = fmt2parser(fm);
            fmt(fm.type) = fm;
            i = i + LOG_FORMAT_MSG_LEN;
            
            % initial allocation
            temp_count(fm.type) = 0;
            temp_size(fm.type) = ALLOC_UNIT;
            temp_len_body(fm.type) = fm.length - LOG_PACKET_HEADER_LEN;
            temp(fm.type).time = zeros(ALLOC_UNIT, 1);
            temp(fm.type).buff = uint8(zeros(ALLOC_UNIT, temp_len_body(fm.type)));
            
        elseif msg_type == LOG_VER_MSG
            ver = decode_ver(buff, i);
            i = i + LOG_VER_MSG_LEN;
            
        else
            warning('Unknown msg_type %d', msg_type);
        end
    else
        warning('Invalid packet');
        i = i + 1;
    end
end

% remove unnessesary elements
for i = 1 : length(temp)
    if ~isempty(temp_count(i))
        temp(i).time = temp(i).time(1:temp_count(i));
        temp(i).buff = temp(i).buff(1:temp_count(i),:);
    end
end

time = time(1:time_count);

if perf_check
    toc
    tic
    disp('Check point 2');
end

%%
% vector processing

for i = 1 : length(temp)
    if ~isempty(temp(i).time)
        k = 1;
        for j = 1 : length(fmt(i).parser)
            [data, k] = fmt(i).parser(j).func(temp(i).buff, k);
            result.log.(fmt(i).name).(fmt(i).parser(j).label) = [temp(i).time data];
        end
    end
end

for i = 1 : parm_count
    result.parm.(parm(i).name) = parm(i).value;
end

result.time = time';
result.ver  = ver;

if perf_check
    toc
    disp('---Finish---');
end

end % end of function

function ret = decode_format(buff, offset)
    ret.type   = buff(offset);
    ret.length = double(buff(offset+1));
    ret.name   = deblank(char(buff(offset+2:offset+5)));
    ret.format = char(buff(offset+6:offset+21));
    ret.labels = char(buff(offset+22:offset+85));
end

function ret = decode_time(buff, offset)
    ret = double(typecast(buff(offset:offset+7), 'uint64'));
end

function ret = decode_ver(buff, offset)
    ret.arch   = char(buff(offset:offset+15));
    ret.fw_git = char(buff(offset+16:offset+79));
end

function ret = decode_parm(buff, offset)
    ret.name  = char(buff(offset:offset+15));
    ret.value = double(typecast(buff(offset+16:offset+19), 'single'));
end

function ret = fmt2parser(format)
    
    labels = [format.labels char(0)]; % add null char
    j = 1;
    
    for i = 1 : length(format.format)
        c = format.format(i);
        
        switch c
            case 'b'
                elem.func = @parse_b;
            case 'B'
                elem.func = @parse_B;
            case 'h'
                elem.func = @parse_h;
            case 'H'
                elem.func = @parse_H;
            case 'i'
                elem.func = @parse_i;
            case 'I'
                elem.func = @parse_I;
            case 'f'
                elem.func = @parse_f;
            case 'n'
                elem.func = @parse_n;
            case 'N'
                elem.func = @parse_N;
            case 'Z'
                elem.func = @parse_Z;
            case 'c'
                elem.func = @parse_c;
            case 'C'
                elem.func = @parse_C;
            case 'e'
                elem.func = @parse_e;
            case 'E'
                elem.func = @parse_E;
            case 'L'
                elem.func = @parse_L;
            case 'M'
                elem.func = @parse_M;
            case 'q'
                elem.func = @parse_q;
            case 'Q'
                elem.func = @parse_Q;
            case 0
                break;
            otherwise
                error('Unknown format charcter : %c', c);
        end
        
        j_st = j;
        while j < length(labels)
            if labels(j) == ',' || labels(j) == 0;
                elem.label = labels(j_st:j-1);
                j = j + 1;
                break;
            end
            j = j + 1;
        end
        
        ret(i) = elem;
    end
end

function [val next] = parse_b(buff, offset)
    val = double(buff(:, offset)) - double(bitand(buff(:, offset), uint8(128))) * 2;
    next = offset + 1;
end

function [val next] = parse_B(buff, offset)
    val = double(buff(:, offset));
    next = offset + 1;
end

function [val next] = parse_h(buff, offset)
    tmp = uint16(buff(:, offset)) ...
        + uint16(buff(:, offset+1)) * uint16(256);
    val = double(tmp) - double(bitand(tmp, uint16(32768))) * 2;
    next = offset + 2;
end

function [val next] = parse_H(buff, offset)
    tmp = uint16(buff(:, offset))  ...
        + uint16(buff(:, offset+1)) * uint16(256);
    val = double(tmp);
    next = offset + 2;
end

function [val next] = parse_i(buff, offset)
    tmp = uint32(buff(:, offset)) ...
        + uint32(buff(:, offset+1)) * uint32(256) ...
        + uint32(buff(:, offset+2)) * uint32(65536) ...
        + uint32(buff(:, offset+3)) * uint32(16777216);
    val = double(tmp) - double(bitand(tmp, uint32(2147483648))) * 2;
    next = offset + 4;
end

function [val next] = parse_I(buff, offset)
    tmp = uint32(buff(:, offset)) ...
        + uint32(buff(:, offset+1)) * uint32(256) ...
        + uint32(buff(:, offset+2)) * uint32(65536) ...
        + uint32(buff(:, offset+3)) * uint32(16777216);
    val = double(tmp);
    next = offset + 4;
end

function [val next] = parse_f(buff, offset)
    % float-formatted byte array 
    % --- manual conversion ---> double-formed character string
    % --- hex2num ---> double value
    %
    % binary format of floating point values :
    % float  : seee eeee  efff ffff  ffff ffff  ffff ffff
    % double : sEEE EEEE  EEEE ffff  ffff ffff  ffff ffff  fff0 0000  ...
    % s : sign, e : float exponent, E : double exponent, f : fraction
    
    % convert float exponent to double exponent
    exp_f = bitshift(bitand(uint32(buff(:,offset+3)), uint32(127)), 1) ...
          + bitshift(uint32(buff(:,offset+2)), -7); % float exponent
    exp   = exp_f + uint32(896); % double exponent
    
    % in case of float exponent is zero, double should be zero
    not_zero = uint32(exp_f ~= uint32(0));
    exp = exp .* not_zero;
    
    % NaN/Inf
    % in case of float exponent is 255, double should be 2047
    nan_inf = uint32(exp_f == uint32(255));
    exp = exp .* (uint32(1) - nan_inf) + uint32(2047) * nan_inf;
    
    % extract a sign
    sign = bitand(uint32(buff(:,offset+3)), uint32(128));
    
    % sign-exponent part and fraction part
    sgn_exp = bitshift(sign, 4) + exp + uint32(4096); % 1xxx
    frac = bitshift(bitand(uint32(buff(:,offset+2)), uint32(127)), 17) ...
         + bitshift(uint32(buff(:,offset+1)), 9) ...
         + bitshift(uint32(buff(:,offset)), 1); % xxxxxx
    
    % make hexadecimal character string
    % (double can contain 52bit integer)
    hex = dec2hex(double(sgn_exp) * 16777216 + double(frac));
    
    % convert string to value
    val = hex2num(hex(:,2:10));

    next = offset + 4;
end

function [val next] = parse_n(buff, offset)
    val = char(buff(offset:offset+3));
    next = offset + 4;
end

function [val next] = parse_N(buff, offset)
    val = char(buff(offset:offset+15));
    next = offset + 16;
end

function [val next] = parse_Z(buff, offset)
    val = char(buff(offset:offset+63));
    next = offset + 64;
end

function [val next] = parse_c(buff, offset)
    tmp = uint16(buff(:, offset))  ...
        + uint16(buff(:, offset+1)) * uint16(256);
    val = (double(tmp) - double(bitand(tmp, uint16(32768))) * 2) * 100;
    next = offset + 2;
end

function [val next] = parse_C(buff, offset)
    tmp = uint16(buff(:, offset))  ...
        + uint16(buff(:, offset+1)) * uint16(256);
    val = double(tmp) * 100;
    next = offset + 2;
end

function [val next] = parse_e(buff, offset)
    tmp = uint32(buff(:, offset)) * uint32(16777216) ...
        + uint32(buff(:, offset+1)) * uint32(65536) ...
        + uint32(buff(:, offset+2)) * uint32(256) ...
        + uint32(buff(:, offset+3));
    val = (double(tmp) - double(bitand(tmp, uint32(2147483648))) * 2) * 100;
    next = offset + 4;
end

function [val next] = parse_E(buff, offset)
    tmp = uint32(buff(:, offset)) ...
        + uint32(buff(:, offset+1)) * uint32(256) ...
        + uint32(buff(:, offset+2)) * uint32(65536) ...
        + uint32(buff(:, offset+3)) * uint32(16777216);
    val = double(tmp) * 100;
    next = offset + 4;
end

function [val next] = parse_L(buff, offset)
    tmp = uint32(buff(:, offset)) ...
        + uint32(buff(:, offset+1)) * uint32(256) ...
        + uint32(buff(:, offset+2)) * uint32(65536) ...
        + uint32(buff(:, offset+3)) * uint32(16777216);
    val = (double(tmp) - double(bitand(tmp, uint32(2147483648))) * 2) * 1e-7;
    next = offset + 4;
end

function [val next] = parse_M(buff, offset)
    val = zeros(size(buff,1), 1);
    next = offset + 1;
end

function [val next] = parse_q(buff, offset)
    val(1:size(buff,1),1) = int64(0);
    for i = 1 : size(buff,1)
        val(i) = typecast(buff(i,offset:offset+7), 'int64');
    end
    next = offset + 8;
end

function [val next] = parse_Q(buff, offset)
    val(1:size(buff,1),1) = uint64(0);
    for i = 1 : size(buff,1)
        val(i) = typecast(buff(i,offset:offset+7), 'uint64');
    end
    next = offset + 8;
end