function TRFeedback_VibrationMotor_ONOFF(motorStates, bt)
    % motorStates: an array of 8 elements where each element is 0 (off) or 1 (on)
    
    if length(motorStates) ~= 8
        error('motorStates must be an array of 8 elements (0 or 1).');
    end
    
    % Calculate the x value
    powersOfTwo = 2.^(0:7);  % Array representing [2^0, 2^1, ..., 2^7]
    x = sum(motorStates .* powersOfTwo);  % Calculate x based on motorStates

    % Set command parameters
    LEN = 5;
    V = 86;
    B = 66;
    W = 87;

    % Calculate checksum (CS)
    CS = mod(sum([LEN, V, B, W, x]), 256);

    % Construct the full command
    if x > 0
        disp("Turning vibration motors ON according to motorStates!")
    else
        disp("Turning all vibration motors OFF!")
    end
    
    command = uint8([62, LEN, V, B, W, x, CS, 60]);  % 62 (start byte '>'), 60 (end byte '<')
    
    % Write command to Bluetooth device
    pause(0.1);  % Small delay to ensure command transmission
    
    % Send command to Bluetooth device
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
