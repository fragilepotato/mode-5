% RESET ZEDBOARD IP SCRIPT (Robust Version)
clear s; delete(instrfind); % Force-close old connections first!
disp('--- Starting IP Reset via USB ---');

% CHANGE THIS to match what you saw in Device Manager
target_com_port = "COM8"; 

try
    % 1. Connect
    disp(["Connecting to " + target_com_port + "..."]);
    s = serialport(target_com_port, 115200); 
    configureTerminator(s, "LF");
    
    % 2. Force the IP Address
    writeline(s, "ifconfig eth0 192.168.1.10 netmask 255.255.255.0");
    pause(1); 
    
    % 3. Verify
    writeline(s, "ifconfig eth0");
    pause(0.5);
    
    while s.NumBytesAvailable > 0
        disp(readline(s));
    end
    
    clear s; 
    disp('--- Success! Try pinging 192.168.1.10 now. ---');

catch ME
    disp('ERROR: Still cannot connect.');
    disp('1. Check Device Manager: Is the port actually ' + target_com_port + '?');
    disp('2. Unplug/Replug the USB cable.');
    disp('3. Restart MATLAB if it keeps failing.');
end