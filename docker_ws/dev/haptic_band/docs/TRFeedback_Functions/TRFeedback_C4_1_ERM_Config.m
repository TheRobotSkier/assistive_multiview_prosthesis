function TRFeedback_ERM_Config(bt, voltage)
    % voltage: Voltage for ERM motor in Volts (0 to 5V)

    % Ensure voltage is within the allowed range
    if voltage < 0 || voltage > 5
        error('Voltage must be between 0 and 5V.');
    end

    % Convert voltage to two ASCII bytes (multiplied by 10)
    voltageValue = floor(voltage * 10);  % Scale voltage by 10
    x1 = floor(voltageValue / 10) + 48;  % ASCII for tens place
    x2 = mod(voltageValue, 10) + 48;     % ASCII for ones place
    
    % Construct the command
    command = uint8([62, 69, 82, 77, 59, x1, x2, 60]);  % >ERM; x1 x2 <
    
    disp("Sending ERM configuration command...");
    
    pause(0.05);  % Small delay before sending
    
    write(bt, command, "uint8");  % Send the command
    
    % Wait for the response from the Bluetooth device
    pause(0.1);  % Allow time for the device to respond
    
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
    
    pause(0.1);  % Small delay to ensure completion
end