function TRFeedback_VibrationMotor_Intensity(motorNumber, intensity, bt)
    % motorNumber: a value between 1 and 8 representing the motor to control
    % intensity: a value between 1 and 100 representing the vibration intensity (1% to 100%)
    
    % Validate input parameters
    if motorNumber < 1 || motorNumber > 8
        error('MotorNumber must be between 1 and 8.');
    end
    if intensity < 0 || intensity > 100
        error('Intensity must be between 0 and 100.');
    end

    % Set command parameters
    LEN = 6;
    V = 86;
    B = 66;
    I = 73;  % ASCII for 'I'
    x1 = motorNumber;  % Motor to control
    x2 = intensity;    % Intensity (1 for 1% to 100 for 100%)

    % Calculate checksum (CS)
    CS = mod(sum([LEN, V, B, I, x1, x2]), 256);

    % Construct the full command
    disp(['Setting vibration intensity for motor ', num2str(x1), ' to ', num2str(x2), '%']);
    
    command = uint8([62, LEN, V, B, I, x1, x2, CS, 60]);  % 62 (start byte '>'), 60 (end byte '<')
    
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
