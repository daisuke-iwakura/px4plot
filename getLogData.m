% THIS IS ALPHA VERSION. THIS MIGHT CHANGE IN FUTURE VERSION.
%
% -- [DATA, TIME] = getLogData(LOG, DATANAME)
%     Gets data and time values using DATANAME character string from the
%     LOG struct.
%
%     Example :
%       log = decode_px4log('01_02_03.px4log');
%       [Roll t_Roll] = getLogData(log, 'ATT.Roll');
%     => The function returns 'ATT.Roll' data series to the variable Roll.
%     Also the time is returned to the variable t_Roll.
%
%       log = decode_px4log('01_02_03.px4log');
%       [ATT t_ATT] = getLogData(log, 'ATT');
%     => The function returns 'ATT' message series to the struct ATT.
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

function [data, time] = getLogData(log, dataname)

if isempty(dataname)
    error('invalid argument');
end

pos   = strfind(dataname, '.');
if ~isempty(pos)
    % 'XXXX.YYYY'  --> left : 'XXXX' / right : 'YYYY'
    % '.YYYY'      --> left : []     / right : 'YYYY'
    left  = dataname(1:pos-1);
    right = dataname(pos+1:end);
else
    % 'XXXX'       --> left : 'XXXX' / right : []
    left  = dataname;
    right = [];
end

if strncmp(dataname, '.time', 5)
    % '.time'
    data = (log.time - log.time(1)) / 1000^2;
    time = data;
elseif strncmp(dataname, '.parm.', 6)
    % '.parm.XXXX'
    
    name = dataname(7:end);
    
    if ~isempty(name) && isfield(log.parm, name)
        data = log.parm.(name);
        time = [];
    else
        data = [];
        time = [];
    end
    
elseif isempty(right) && isfield(log.log, left)
    % 'XXXX' format
    % returns the "data" from the struct log.log.XXXX excluding time
    
    names = fieldnames(log.log.(left));
    
    if numel(names) >= 1
        for i = 1 : numel(names)
            data.(names{i}) = log.log.(left).(names{i})(:,2);
        end
        time = (double(log.log.(left).(names{1})(:,1)) - log.time(1)) / 1000^2;
    else
        data = [];
        time = [];
    end
    
elseif isfield(log.log, left) && isfield(log.log.(left), right)
    % 'XXXX.YYYY' format
    % returns the field log.log.XXXX.YYYY
    
    data = log.log.(left).(right)(:,2);
    time = (log.log.(left).(right)(:,1) - log.time(1)) / 1000^2;
else
    data = [];
    time = [];
end