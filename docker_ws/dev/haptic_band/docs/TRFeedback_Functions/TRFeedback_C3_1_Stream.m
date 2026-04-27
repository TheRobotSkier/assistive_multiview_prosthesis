
%% Scans for available Bluetooth devices
disp("Searching for Bluetooth devices...");
btDevices = bluetoothlist;  % Scan for devices

if isempty(btDevices)
    error('No Bluetooth devices found.');
else
    disp("Found Bluetooth devices:");
    disp(btDevices);  % Display the list (table format)
end

%% Prompts the User to Select the Two Devices
deviceIndex1 = input("Enter the device index for Device 1: ");
deviceIndex2 = input("Enter the device index for Device 2: ");

if deviceIndex1 == deviceIndex2
    error("The selected indices are the same. Please select two different devices.");
end

%% Extracts Bluetooth Addresses for Both Devices
selectedDevice1 = btDevices(deviceIndex1, :);
selectedDevice2 = btDevices(deviceIndex2, :);

% Addresses are stored in a cell array; extract as string
btAddress1 = selectedDevice1.Address{1};
btAddress2 = selectedDevice2.Address{1};

disp(['Selected Device 1 address: ' btAddress1]);
disp(['Selected Device 2 address: ' btAddress2]);

%% Retry Mechanism
maxRetries = 5;  % Maximum number of connection attempts

btDevice1 = BluetoohConnect(btAddress1, 1, maxRetries);
btDevice2 = bluetooth(btAddress2, 1, maxRetries);

% Store the connected devices in an array for further operations.
btDevicesArray = [btDevice1, btDevice2];

%% Streaming Function

% X - Prompts for turning Data stream ON or OFF

disp("Please Press either 1 to start the data stream and 0 to end it");

statusStream = input("Do you wish to turn the data stream ON (Press 1) or OFF (Press 0) ");

% Communication 

    START_COM = 62;   % '>' in decimal
    LEN = 4;           % LEN = Command length(7 - 3 = 4)
    S = 83;               % 'S' in ASCII
    T = 84;               % 'T' in ASCII
    x = statusStream;      % 1 to start, 0 to stop
    END_COM = 60;     % '<' in decimal

% Calculate CS

CS = mod(sum([LEN, S, T, x]), 256);

% TRFeedback_Stream(1, btDevicesArray);  % Start streaming

completeCommand = [START_COM, LEN, S, T, x, CS, END_COM];

% Send command via Bluetooth
write(btDevice1, completeCommand, 'uint8');

% Read the Stream?
read(btDevice1,80)