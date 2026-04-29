function TRFeedback_LRA_Config(bt, voltage, frequency)
    % voltage: Voltage for LRA motor in Volts (0 to 5V)
    % frequency: Frequency for LRA motor in Hz (125 to 300 Hz)

    % Ensure voltage is within the allowed range
    if voltage < 0 || voltage > 5
        error('Voltage must be between 0 and 5V.');
    end

    % Ensure frequency is within the allowed range
    if frequency < 125 || frequency > 300
        error('Frequency must be between 125 and 300 Hz.');
    end
    
    % Convert voltage to two ASCII bytes (multiplied by 10)
    voltageValue = floor(voltage * 10);  % Scale voltage by 10
    x1 = floor(voltageValue / 10) + 48;  % ASCII for tens place
    x2 = mod(voltageValue, 10) + 48;     % ASCII for ones place

    % Convert frequency to three ASCII bytes
    frequencyStr = num2str(frequency, '%03d');  % Convert frequency to string with 3 digits
    x3 = uint8(frequencyStr(1));  % First digit (hundreds place)
    x4 = uint8(frequencyStr(2));  % Second digit (tens place)
    x5 = uint8(frequencyStr(3));  % Third digit (ones place)
    
    % Construct the command
    command = uint8([62, 76, 82, 65, 59, x1, x2, 59, x3, x4, x5, 60]);  % >LRA; x1 x2 ; x3 x4 x5 <
    
    disp("Sending LRA configuration command...");
    
    % Write the command to the Bluetooth device
    pause(0.05);  % Small delay before sending
    
    write(bt, command, "uint8");  % Send the command
    
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
