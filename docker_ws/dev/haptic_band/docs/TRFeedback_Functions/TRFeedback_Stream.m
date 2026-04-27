function TRFeedback_Stream(statusStream, bt)
    % Communication 
    START_COM = 62;      % '>' in decimal
    LEN = 0x04;          % Command length (0x04)
    S = 83;              % 'S' in ASCII
    T = 84;              % 'T' in ASCII
    x = statusStream == 1;  % 1 to start (0x01), 0 to stop (0x00)
    END_COM = 60;        % '<' in decimal
    
    % Set the status byte (0x01 for start, 0x00 for stop)
    if x == 1
        statusByte = 0x01;
    else
        statusByte = 0x00;
    end
    
    % Calculate CS (checksum)
    CS = mod(sum([LEN, S, T, statusByte]), 256);
    
    % Construct the command sequence
    completeCommand = [START_COM, LEN, S, T, statusByte, CS, END_COM];
    
    % Send command via Bluetooth
    write(bt, completeCommand, 'uint8');
end
