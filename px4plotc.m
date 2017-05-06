% THIS IS ALPHA VERSION. THIS MIGHT CHANGE IN FUTURE VERSION.
%
% -- px4plotc(FILEPATH)
% -- px4plotc(FILEPATH, '|XXXs')
%     Plot of .px4log binary file logged by px4 autopilot.
%     (*)The function makes a cache file for fast reloading.
%
%     Useage :
%       Same as px4plot. Type
%         help px4plot
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

function px4plotc(filename, varargin)

path_cache = [filename '.mat'];

MODE_CACHE    = 1;
MODE_ORIGINAL = 2;

% |  cache  |original |identity|
% |---------+---------+--------|
% |  found  |  found  |   OK   | -> load cache
% |  found  |  found  |   NG   | -> load original
% |  found  |not found|   --   | -> load cache (with warning)
% |not found|  found  |   --   | -> load original
% |not found|not found|   --   | -> error

if exist(path_cache, 'file');
    % cache file found
    
    load(path_cache);
    
    if exist(filename, 'file')
        % original file found
        
        % check identity
        if isfield(cache_data, 'identifier') && ...
           isfield(cache_data.identifier, 'datenum')
            
            fileInfo = dir(filename);
            
            if cache_data.identifier.datenum == fileInfo.datenum
                % valid cache : load the cache file
                mode = MODE_CACHE;
            else
                % invalid cache : 
                mode = MODE_ORIGINAL;
            end
        else
            mode = MODE_ORIGINAL;
        end
    else
        % original file not found
        warning('original file does not exist');
        
        mode = MODE_CACHE;
    end
    
else
    % no cache exist
    
    mode = MODE_ORIGINAL;
end

if mode == MODE_ORIGINAL
    cache_data.body = decode_px4log(filename);
    
    fileInfo = dir(filename);
    cache_data.identifier.datenum = fileInfo.datenum;
    
    save(path_cache, 'cache_data');
end

px4plot(cache_data.body, varargin{:});

end % end of function
