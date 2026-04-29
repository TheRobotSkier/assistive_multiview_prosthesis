clc;
clear;
close all;

% Clear any existing serial connections
serialobjects = instrfind('Type', 'serial');
if ~isempty(serialobjects)
   fclose(serialobjects);
   delete(serialobjects);
end

%% List available Bluetooth devices
disp("Searching for bluetooth devices")
btDevices = bluetoothlist;

if isempty(btDevices)
    error('No Bluetooth devices found.');
else
    disp("Found 1 or more devices")
    disp(btDevices)
end

%% Connect to the Bluetooth device 1 using its address
disp("Connecting to the TR-FB device 1")
btAddress1 = '842E1409E14E';  % Bluetooth address
maxRetries = 5;  % Number of attempts
connectionSuccess = false;  % Flag to track connection status

for i = 1:maxRetries
    try
        % Attempt to connect to the Bluetooth device
        bt1 = bluetooth(btAddress1, 1);  % Channel 1 is usually the default
        disp('Connected to TR-FB device 1 successfully!');
        connectionSuccess = true;
        break;
    catch ME
        disp(['Attempt ', num2str(i), ' failed: ', ME.message]);
        disp('Retrying connection...');
        pause(1); 
    end
end
if ~connectionSuccess
    error("Unable to establish Bluetooth connection to the TR-FB device 1 after " + maxRetries + " tries.");
end

% Connect to the Bluetooth device 2 using its address
disp("Connecting to the TR-FB device 2")
btAddress2 = '8CF681F32E12';  % Bluetooth address
maxRetries = 5;  % Number of attempts
connectionSuccess = false;  % Flag to track connection status

for i = 1:maxRetries
    try
        % Attempt to connect to the Bluetooth device
        bt2 = bluetooth(btAddress2, 1);  % Channel 1 is usually the default
        disp('Connected to TR-FB device 2 successfully!');
        connectionSuccess = true;
        break;
    catch ME
        disp(['Attempt ', num2str(i), ' failed: ', ME.message]);
        disp('Retrying connection...');
        pause(1); 
    end
end
if ~connectionSuccess
    error("Unable to establish Bluetooth connection to the TR-FB device 2 after " + maxRetries + " tries.");
end

% Turn OFF Michelanglo search
TRFeedback_MichelangeloSearch(bt1, 0);
TRFeedback_MichelangeloSearch(bt2, 0);

%%
TRFeedback_Stream(1, bt1)
TRFeedback_Stream(1, bt2)
pause(0.1)
counter = 0;
empty_data1 = read(bt1, bt1.NumBytesAvailable, "uint8");
empty_data2 = read(bt2, bt2.NumBytesAvailable, "uint8");

startTime = tic;
while toc(startTime) < 10
    %continue;
end
data1 = read(bt1, bt1.NumBytesAvailable, "uint8");
data2 = read(bt2, bt2.NumBytesAvailable, "uint8");

disp("Stopping")
toc(startTime)
TRFeedback_Stream(0, bt1); % Stop streaming data from bt1
TRFeedback_Stream(0, bt2); % Stop streaming data from bt2

%%
% Prepare Timescope
imuScopeAcc1 = timescope(...
    'NumInputPorts', 3, ...
    'Name', 'IMU Acceleration 1', ...
    'SampleRate', 100, ...  
    'TimeSpanSource', 'Property', ...
    'TimeSpan', 2, ...
    'YLimits', [-3*10^4, 3*10^4], ...
    'ShowGrid', true, ...
    'ChannelNames', {'AccX1', 'AccY1', 'AccZ1'});

imuScopeAcc2 = timescope(...
    'NumInputPorts', 3, ...
    'Name', 'IMU Acceleration 2', ...
    'SampleRate', 100, ...  
    'TimeSpanSource', 'Property', ...
    'TimeSpan', 2, ...
    'YLimits', [-3*10^4, 3*10^4], ...
    'ShowGrid', true, ...
    'ChannelNames', {'AccX2', 'AccY2', 'AccZ2'});

TRFeedback_Stream(1, bt1); % Start streaming data from bt1
TRFeedback_Stream(1, bt2); % Start streaming data from bt2

startTime = tic;
counter = 0;

while toc(startTime) < 10  % Run for 1 second
    % Check if bt1 has available data
    if bt1.NumBytesAvailable > 0
        disp(['Data available from bt1: ', num2str(bt1.NumBytesAvailable), ' bytes']);
        counter = counter + 1;
        data1 = read(bt1, 21);  % Read the available data

        % Extract acceleration data (assuming little-endian and 16-bit values)
        accX1 = typecast(uint8([data1(2), data1(3)]), 'int16');
        accY1 = typecast(uint8([data1(4), data1(5)]), 'int16');
        accZ1 = typecast(uint8([data1(6), data1(7)]), 'int16');

        
        % Push values to the timescope
        if ~isempty(imuScopeAcc1)
            imuScopeAcc1(accX1, accY1, accZ1);
        end
    end
    
    % Check if bt2 has available data
    if bt2.NumBytesAvailable > 0
        counter = counter + 1;
        disp(['Data available from bt2: ', num2str(bt2.NumBytesAvailable), ' bytes']);
        data2 = read(bt2, 21);  % Read the available data

        % Extract acceleration data (assuming little-endian and 16-bit values)
        accX2 = typecast(uint8([data2(2), data2(3)]), 'int16');
        accY2 = typecast(uint8([data2(4), data2(5)]), 'int16');
        accZ2 = typecast(uint8([data2(6), data2(7)]), 'int16');

        
        % Push values to the timescope
        if ~isempty(imuScopeAcc2)
            imuScopeAcc2(accX2, accY2, accZ2);
        end
    end
end

disp("Stopping");
toc(startTime);
TRFeedback_Stream(0, bt1); % Stop streaming data from bt1
TRFeedback_Stream(0, bt2); % Stop streaming data from bt2


%% Prepare timescopes

imuScopeAcc1 = timescope(...
    'NumInputPorts', 3, ...
    'Name', 'IMU Acceleration 1', ...
    'SampleRate', 100, ...  
    'TimeSpanSource', 'Property', ...
    'TimeSpan', 2, ...
    'YLimits', [-1100, 1100], ...
    'ShowGrid', true, ...
    'ChannelNames', {'AccX1', 'AccY1', 'AccZ1'});

imuScopeAcc2 = timescope(...
    'NumInputPorts', 3, ...
    'Name', 'IMU Acceleration 2', ...
    'SampleRate', 100, ...  
    'TimeSpanSource', 'Property', ...
    'TimeSpan', 2, ...
    'YLimits', [-1100, 1100], ...
    'ShowGrid', true, ...
    'ChannelNames', {'AccX2', 'AccY2', 'AccZ2'});

imuScopeAcc1(0, 0, 0);
imuScopeAcc2(0, 0, 0);

%% Read data
TRFeedback_Stream(1, bt1)
TRFeedback_Stream(1, bt2)
streaming = 1;

flushinput(bt1);
flushinput(bt2);
pause(0.5)

streaming = 1;
disp("Starting");
startTime = tic;

while toc(startTime) < 10
    if bt1.BytesAvailable >= 12
        rawData1 = fread(bt1, bt1.BytesAvailable);  % Read available data
        imuData1 = rawData1';
        
        if numel(imuData1) >= 12
            accX1 = typecast(uint8(imuData1(7:8)), 'int16');
            accY1 = typecast(uint8(imuData1(9:10)), 'int16');
            accZ1 = typecast(uint8(imuData1(11:12)), 'int16');
            imuScopeAcc1(accX1, accY1, accZ1);
        end
    end

    if bt2.BytesAvailable >= 12
        rawData2 = fread(bt2, bt2.BytesAvailable);  % Read available data
        imuData2 = rawData2';
        
        if numel(imuData2) >= 12
            accX2 = typecast(uint8(imuData2(7:8)), 'int16');
            accY2 = typecast(uint8(imuData2(9:10)), 'int16');
            accZ2 = typecast(uint8(imuData2(11:12)), 'int16');
            imuScopeAcc2(accX2, accY2, accZ2);
        end
    end
end
disp("Stopping...");
toc(startTime);
TRFeedback_Stream(0, bt1)
TRFeedback_Stream(0, bt2)

%% Turn vibration motors on/off - Doesn't work
motorStates = [0 0 0 0 0 0 0 0];  % 1 = on, 0 = off for each motor
TRFeedback_VibrationMotor_ONOFF(motorStates, bt);

%% Vibration motors duration - Doesn't work
motorNumber = 1;
duration = 50;  % 50 tenths of a second = 5 seconds
TRFeedback_VibrationMotor_Duration(motorNumber, duration, bt);

%% Vibration motors intensity
motorNumber = 1;
intensity = 0;  % intensity
TRFeedback_VibrationMotor_Intensity(motorNumber, intensity, bt);

%% Trigger all vibration motors
vibromotors_intensities = [0, 0, 0, 0, 0, 0, 0, 0];
TRFeedback_VibrationMotor_TriggerAll(bt1, vibromotors_intensities)

%% Turn OFF all vibration motors
vibromotors_intensities = [0, 0, 0, 0, 0, 0, 0, 0];
TRFeedback_VibrationMotor_TriggerAll(bt, vibromotors_intensities)

%% Stream data
TRFeedback_Stream(1, bt1) % 1 = start, 0 = stop

%% Log data on SD card - Doesn't work
TRFeedback_uSDLog(0, bt); %Log data = 1, stop logging = 0

%% Stream and collect data example
TRFeedback_Stream(1, bt1) % 1 = start, 0 = stop
streamingDuration = 10;

collectedData = {};

startTime = tic;  % Start the timer

% Read data
disp("Reading data for 10 seconds...");
while toc(startTime) < streamingDuration
    data = read(bt1, 27); %Data stream package length is currently 27 long
    if isempty(data)
        collectedData{end+1} = nan;
    else
        collectedData{end+1} = data;
    end
    disp(data);
end

disp("10 seconds elapsed. Stopping data stream...");

% Stop streaming
TRFeedback_Stream(0, bt1) % 1 = start, 0 = stop
