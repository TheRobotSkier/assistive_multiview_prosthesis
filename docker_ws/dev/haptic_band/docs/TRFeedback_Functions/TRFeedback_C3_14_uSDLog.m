function TRFeedback_C3_14_uSDLog(on_off, bt)
    % Set x = 1 to start the data logging on the SD card, use 0 to stop
    S = 83;
    D = 68;
    % Construct the full command
    if on_off == 1
        disp("Sending command to start logging data!")
    elseif on_off == 0
        disp("Sending command to stop logging data!")
    end
    command = uint8([62, S, D, on_off, 60]);  % 62 (start byte '>'), 60 (end byte '<')
    
    % Write command to start data stream
    write(bt, command, "uint8");

    % Wait for the response from the Bluetooth device
    timeout = 10; % Timeout in seconds
    startTime = tic; % Start timer
    
    while toc(startTime) < timeout
        % Check if there is a response available
        if bt.NumBytesAvailable > 0
            % Read the response from the Bluetooth device
            response = read(bt, bt.NumBytesAvailable);
            response_char = char(response);
            disp("Response from the device: " + response_char);
            break; % Exit the loop if response is received
        else
            pause(0.1); % Short pause before checking again
        end
    end
    
    if toc(startTime) >= timeout
        disp("No response from the device within the timeout period.");
    end
end

