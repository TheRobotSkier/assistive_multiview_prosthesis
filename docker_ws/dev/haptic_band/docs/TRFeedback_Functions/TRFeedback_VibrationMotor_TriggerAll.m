function TRFeedback_VibrationMotor_TriggerAll(bt, intensities)
    % Triggers all vibromotors with their respective amplitude (intensity).
    % intensities: an 8-element array where each value corresponds to a motor's intensity (0-100).
    % bt: Bluetooth device object.

    % Validate input
    if length(intensities) ~= 8
        error('Intensities must be an 8-element array representing each motors intensity');
    end
    if any(intensities < 0) || any(intensities > 100)
        error('Each intensity must be between 0 and 100.');
    end
    
    % Command structure (decimal values)
    start_byte = 62;  % '>'
    V = 86;  % 'V'
    B = 66;  % 'B'
    A = 65;  % 'A'
    separator = 59;  % ';'
    end_byte = 60;  % '<'

    % Convert intensities to hex range (0x00 to 0x64)
    intensities = floor(intensities);
    intensity_bytes = uint8(intensities);

    command = [start_byte, V, B, A, separator, intensity_bytes, end_byte];

    % Display the command being sent
    %disp(['Triggering all motors with intensities: ', num2str(intensities)]);

    % Send the command to the Bluetooth device
    write(bt, command, "uint8");

end
