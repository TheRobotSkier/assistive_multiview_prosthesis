function TRFeedback_VibrationMotor_Duration(motorNumber, duration, bt)
    % motorNumber: a value between 1 and 8 representing the motor to control
    % duration: a value between 0 (infinite) and 100 (0 to 10 seconds in tenths of a second)
    
    % Validate input parameters
    if motorNumber < 1 || motorNumber > 8
        error('motorNumber must be between 1 and 8.');
    end
    if duration < 0 || duration > 100
        error('duration must be between 0 and 100.');
    end

    % Set command parameters
    LEN = 6;
    V = 86;
    B = 66;
    D = 68;
    x1 = motorNumber;  % Motor to control
    x2 = duration;     % Duration (0 for infinite, 1-100 for 0.1s to 10s)

    % Calculate checksum (CS)
    CS = mod(sum([LEN, V, B, D, x1, x2]), 256);

    % Construct the full command
    if x2 == 0
        disp(['Setting infinite vibration for motor ', num2str(x1)]);
    else
        disp(['Setting vibration for motor ', num2str(x1), ' to ', num2str(x2 / 10), ' seconds']);
    end
    
    command = uint8([62, LEN, V, B, D, x1, x2, CS, 60]);  % 62 (start byte '>'), 60 (end byte '<')
    
    % Write command to Bluetooth device
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
