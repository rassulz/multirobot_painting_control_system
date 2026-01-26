function printlog(message, varargin)
if nargin > 1
    message = sprintf(message, varargin{:});
end
fprintf('[%s]: %s\n', datestr(datetime('now')), message);
end

