%RUN_MOTOR_TESTS  Records the bench motor tests to matlab/data/*.csv.
%   Run ONE section at a time (Ctrl+Enter), after uploading the sketch it names.
%   Close the Arduino Serial Monitor first - only one program can use the port.

port   = "COM5";          % <-- change to your port (Arduino IDE: Tools > Port)
datadir = fullfile(fileparts(mfilename("fullpath")), "data");
if ~exist(datadir, "dir"), mkdir(datadir); end

%% 1. Upload test_deadband, then run this section (~4 min)
capture_serial(port, fullfile(datadir, "deadband.csv"), "", 300);

%% 2. Upload test_step, then run this section (~1.5 min)
% Steps with brake stop, max speed (255), reverse, and two coast stops.
cmds = ["100", "150", "200", "255", "-150", "c150", "c255"];
for c = cmds
    capture_serial(port, fullfile(datadir, "step_" + c + ".csv"), c, 20);
    pause(1);
end

%% 3. Upload test_speed_curve, then run this section (~1.5 min)
capture_serial(port, fullfile(datadir, "speed_curve.csv"), "", 120);

%% 4. Analyse everything
analyse_motor_tests(datadir);
