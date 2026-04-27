function TRFeedback_MichelangeloSearch(bt, onoff)
    % Command to disable the automatic search for the Michelangelo hand
    if onoff < 0 || onoff > 1
        error('ON/OFF input must be 0 = OFF, or 1 == ON');
    end
    command = uint8([62, 83, 72, 59, onoff, 60]);  % >SH;0< in hex: 0x3E 0x53 0x48 0x3B 0x00 0x3C
    disp("Sending command to disable Michelangelo search...");
    
    write(bt, command, "uint8");  % Send the command
    
    % Wait for the response from the Bluetooth device
    % timeout = 10; % Timeout in seconds
    % startTime = tic; % Start timer
    % 
    % while toc(startTime) < timeout
    %     % Check if there is a response available
    %     if bt.NumBytesAvailable > 0
    %         % Read the response from the Bluetooth device
    %         response = read(bt, bt.NumBytesAvailable);
    %         response_char = char(response);
    %         disp("Response from the device: " + response_char);
    %         break; % Exit the loop if response is received
    %     else
    %         pause(0.1); % Short pause before checking again
    %     end
    % end
    % 
    % if toc(startTime) >= timeout
    %     disp("No response from the device within the timeout period.");
    % end
    
    pause(0.1);  % Small delay to ensure completion
end
