%% TRFeedback 16 Vibro motors
clc
clear
close all;

%% Connnect
disp("Connecting...")
for i = 1:5
    try
        btAddress1 = '842E1409E14E';  % Bluetooth address
        bt1 = bluetooth(btAddress1, 1);
        disp("Bt1 Connected")
        break;
    catch ME
        disp("Bt1 Connection failed")
    end
end

for i = 1:5
    try
        btAddress2 = '8CF681F32E12';  % Bluetooth address
        bt2 = bluetooth(btAddress2, 1);
        disp("Bt2 Connected")
        break;
    catch ME
        disp("Bt2 Connection failed")
    end
end

%% Test sample rate
duration = 10;
startCmd = uint8(['>' 4 'S' 'T' 1 0 '<']);
stopCmd = uint8(['>' 4 'S' 'T' 0 0 '<']);
disp('Started streaming...');
buffer = [];
timestamps = [];
parsedData = [];  % Store decoded values
packages = 0;

% Start streaming
write(bt2, startCmd, "uint8");

pause(0.5)
% Empty the buffer
if bt2.NumBytesAvailable > 0
    data = read(bt2, bt2.NumBytesAvailable, "uint8");
else
    data = [];  % Nothing was available to read
end
startTime = tic;
while toc(startTime) < duration
    if bt2.NumBytesAvailable > 0
        newData = read(bt2, bt2.NumBytesAvailable, "uint8");
        buffer = [buffer; newData];  % Append new data
    end

    % Process all complete 21-byte packets
    while length(buffer) >= 21
        if buffer(1) ~= '62'
            buffer(1) = [];  % Skip bad start byte
            continue;
        end
        if buffer(21) ~= '60'
            buffer(1) = [];  % Skip until next potential start
            continue;
        end

        % Extract and remove the message
        msg = buffer(1:21);
        buffer(1:21) = [];

        % Parse the message
        accX = typecast(uint8(msg(2:3)), 'int16');
        accY = typecast(uint8(msg(4:5)), 'int16');
        accZ = typecast(uint8(msg(6:7)), 'int16');
        gyrX = typecast(uint8(msg(8:9)), 'int16');
        gyrY = typecast(uint8(msg(10:11)), 'int16');
        gyrZ = typecast(uint8(msg(12:13)), 'int16');
        eulHeading = typecast(uint8(msg(14:15)), 'int16');
        eulRoll    = typecast(uint8(msg(16:17)), 'int16');
        eulPitch   = typecast(uint8(msg(18:19)), 'int16');
        batterySOC = msg(20);  % Battery is 1 byte (uint8)

        % Store the parsed data
        parsedData(end+1, :) = [accX, accY, accZ, gyrX, gyrY, gyrZ, ...
                                eulHeading, eulRoll, eulPitch, batterySOC];

        timestamps(end+1) = toc(startTime);
        packages = packages + 1;
    end
end

% Stop streaming
write(bt2, stopCmd, "uint8");
disp('Stopped streaming');

elapsedTime = toc(startTime)
sampleRate = packages / elapsedTime;
fprintf('Sample rate: %.2f Hz\n', sampleRate);


%% Plot live data
% Prepare Timescope
imuScopeAcc1 = timescope(...
    'NumInputPorts', 3, ...
    'Name', 'IMU Acceleration 1', ...
    'SampleRate', 100, ...  
    'TimeSpanSource', 'Property', ...
    'TimeSpan', 5, ...
    'YLimits', [-3*10^4, 3*10^4], ...
    'ShowGrid', true, ...
    'ChannelNames', {'AccX1', 'AccY1', 'AccZ1'});

imuScopeAcc2 = timescope(...
    'NumInputPorts', 3, ...
    'Name', 'IMU Acceleration 2', ...
    'SampleRate', 100, ...  
    'TimeSpanSource', 'Property', ...
    'TimeSpan', 5, ...
    'YLimits', [-3*10^4, 3*10^4], ...
    'ShowGrid', true, ...
    'ChannelNames', {'AccX2', 'AccY2', 'AccZ2'});

imuScopeAcc1(0, 0, 0);
imuScopeAcc2(0, 0, 0);

startCmd = uint8(['>' 4 'S' 'T' 1 0 '<']);
write(bt1, startCmd, "uint8");
write(bt2, startCmd, "uint8");
pause(0.5)
empty_data1 = read(bt1, bt1.NumBytesAvailable, "uint8"); %Empty the buffer
empty_data2 = read(bt2, bt2.NumBytesAvailable, "uint8"); %Empty the buffer
disp('Started streaming...');

startTime = tic;
counter = 0;

% Constants
MSG_LEN = 21;

% Main loop
while toc(startTime) < 20
    % === Device 1 ===
    if bt1.NumBytesAvailable >= MSG_LEN
        data1 = read(bt1, floor(bt1.NumBytesAvailable / MSG_LEN) * MSG_LEN, "uint8");

        for i = 1:MSG_LEN:length(data1)
            msg = data1(i:i+MSG_LEN-1);
            if msg(1) ~= '>'
                continue; % Skip if start byte is not correct
            end

            accX1 = typecast(uint8([msg(2), msg(3)]), 'int16');
            accY1 = typecast(uint8([msg(4), msg(5)]), 'int16');
            accZ1 = typecast(uint8([msg(6), msg(7)]), 'int16');
            imuScopeAcc1(accX1, accY1, accZ1);
        end
    end

    % === Device 2 ===
    if bt2.NumBytesAvailable >= MSG_LEN
        data2 = read(bt2, floor(bt2.NumBytesAvailable / MSG_LEN) * MSG_LEN, "uint8");

        for i = 1:MSG_LEN:length(data2)
            msg = data2(i:i+MSG_LEN-1);
            if msg(1) ~= '>'
                continue;
            end

            accX2 = typecast(uint8([msg(2), msg(3)]), 'int16');
            accY2 = typecast(uint8([msg(4), msg(5)]), 'int16');
            accZ2 = typecast(uint8([msg(6), msg(7)]), 'int16');
            imuScopeAcc2(accX2, accY2, accZ2);
        end
    end
end


disp("Stopping");
toc(startTime);
stopCmd = uint8(['>' 4 'S' 'T' 0 0 '<']);
write(bt1, stopCmd, "uint8");
write(bt2, stopCmd, "uint8");

%% Plot live data in another way
% Setup
MSG_LEN = 21;
maxSamples = 1000;  % For plotting buffer

% Buffers
accX1_buf = [];
accY1_buf = [];
accZ1_buf = [];
time_buf = [];

% Start command
startCmd = uint8(['>' 4 'S' 'T' 1 0 '<']);
write(bt1, startCmd, "uint8");
write(bt2, startCmd, "uint8");
pause(0.5)

% Clear buffers
read(bt1, bt1.NumBytesAvailable, "uint8");
read(bt2, bt2.NumBytesAvailable, "uint8");

disp('Started streaming...');

% Plot Initialization
figure('Name', 'Live AccX1');
hPlot = plot(nan, nan);
xlabel('Time (s)');
ylabel('AccX1 (g)');
grid on;
ylim([-3e4, 3e4]);  % Adjust as needed
hold on;

% Main Loop
startTime = tic;
while toc(startTime) < 20
    tNow = toc(startTime);

    % === Read from bt1 ===
    if bt1.NumBytesAvailable >= MSG_LEN
        data1 = read(bt1, floor(bt1.NumBytesAvailable / MSG_LEN) * MSG_LEN, "uint8");

        for i = 1:MSG_LEN:length(data1)
            msg = data1(i:i+MSG_LEN-1);
            if msg(1) ~= '>'
                continue;
            end

            accX1 = typecast(uint8([msg(2), msg(3)]), 'int16');
            accY1 = typecast(uint8([msg(4), msg(5)]), 'int16');
            accZ1 = typecast(uint8([msg(6), msg(7)]), 'int16');

            % Append to buffer
            accX1_buf = [accX1_buf; accX1];
            accY1_buf = [accY1_buf; accY1];
            accZ1_buf = [accZ1_buf; accZ1];
            time_buf = [time_buf; tNow];

            % Limit buffer
            if numel(accX1_buf) > maxSamples
                accX1_buf = accX1_buf(end-maxSamples+1:end);
                accY1_buf = accY1_buf(end-maxSamples+1:end);
                accZ1_buf = accZ1_buf(end-maxSamples+1:end);
                time_buf = time_buf(end-maxSamples+1:end);
            end
        end

        % Update plot
        set(hPlot, 'XData', time_buf, 'YData', accX1_buf);
        drawnow limitrate;
    end

    % === bt2 (optional) ===
    if bt2.NumBytesAvailable >= MSG_LEN
        data2 = read(bt2, floor(bt2.NumBytesAvailable / MSG_LEN) * MSG_LEN, "uint8");
        % You can parse and use accX2/Y2/Z2 as needed
    end
end

% Sample Rate Estimation
if numel(time_buf) > 1
    dt = diff(time_buf);
    sampleRate = 1 / mean(dt);
    fprintf('Estimated Sample Rate (bt1): %.2f Hz\n', sampleRate);
end

% Stop Streaming
disp("Stopping");
stopCmd = uint8(['>' 4 'S' 'T' 0 0 '<']);
write(bt1, stopCmd, "uint8");
write(bt2, stopCmd, "uint8");



%%
% 1. Set intensity to 80% for motor 1
cmd_intensity = uint8([ ...
    hex2dec('3E'), ...
    hex2dec('06'), ...
    uint8('V'), ...
    uint8('B'), ...
    uint8('I'), ...
    hex2dec('01'), ...    % Motor 1
    hex2dec('0'), ...    % 80% intensity
    hex2dec('00'), ...
    hex2dec('3C')
]);
write(bt2, cmd_intensity, 'uint8');
write(bt2, cmd_intensity, 'uint8');
pause(0.05);

if bt2.NumBytesAvailable > 0
    data = read(bt2, bt2.NumBytesAvailable, "uint8");
    disp(data);
end

% 2. Set duration to 2 seconds (20 x 100ms) for motor 1
cmd_duration = uint8([ ...
    hex2dec('3E'), ...
    hex2dec('06'), ...
    uint8('V'), ...
    uint8('B'), ...
    uint8('D'), ...
    hex2dec('01'), ...    % Motor 1
    hex2dec('14'), ...    % 2 seconds
    hex2dec('00'), ...
    hex2dec('3C')
]);
write(bt2, cmd_duration, 'uint8');
write(bt2, cmd_duration, 'uint8');
pause(0.05);

% 3. Activate motor 1 (bitmask: 0b00000001 = 0x01)
cmd_activate = uint8([ ...
    hex2dec('3E'), ...
    hex2dec('05'), ...
    uint8('V'), ...
    uint8('B'), ...
    uint8('W'), ...
    hex2dec('01'), ...    % Bitmask for motor 1
    hex2dec('00'), ...
    hex2dec('3C')
]);
write(bt2, cmd_activate, 'uint8');
write(bt2, cmd_activate, 'uint8');
