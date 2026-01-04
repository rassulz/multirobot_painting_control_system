% Connect to robot
robotIP = '192.168.1.10';  % Rasul Replace this with the robot IP Address
port = 7000;
t = tcpclient(robotIP, port, 'Timeout', 10);

% Function to write a variable (e.g., set target_pos)
function writeVar(tcp, varName, varValue)
    msgId = uint16(1);  % Arbitrary ID, increment if needed
    mode = uint8(1);   % 1 = Write
    varNameBytes = uint8(varName);
    varNameLen = uint16(length(varNameBytes));
    varValueBytes = uint8(varValue);
    varValueLen = uint16(length(varValueBytes));
    
    % Build message: [Id (2B)] [ContentLen (2B)] [Mode (1B)] [VarNameLen (2B)] [VarName] [VarValueLen (2B)] [VarValue]
    contentLen = uint16(1 + 2 + length(varNameBytes) + 2 + length(varValueBytes));
    msg = [typecast(swapbytes(msgId), 'uint8') ...  % Big-endian
           typecast(swapbytes(contentLen), 'uint8') ...
           mode ...
           typecast(swapbytes(varNameLen), 'uint8') ...
           varNameBytes ...
           typecast(swapbytes(varValueLen), 'uint8') ...
           varValueBytes];
    
    write(tcp, msg);
    
    % Read response (optional, to check success)
    response = read(tcp, tcp.NumBytesAvailable);
    if ~isempty(response)
        tail = response(end-2:end);
        if all(tail == [0 1 1])
            disp('Write successful');
        else
            disp('Write failed');
        end
    end
end

% Example: Set target position to {X 600, Y 100, Z 400, A 0, B 90, C 0}
posStr = '{X 600,Y 100,Z 400,A 0,B 90,C 0}';  % KRL POS format
writeVar(t, 'target_pos', posStr);

% Trigger movement
writeVar(t, 'move_trigger', 'TRUE');

% Clean up
clear t;