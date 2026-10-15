%% log_trial.m -- run one trial on the ESP32 and save it as CSV
%
% Starts a trial (sends 's'), records every row the firmware streams
% until END, then saves the window from PRE_S before the motor turns on
% to POST_S after the shaft has come to rest with the motor off.
%
% Close the PlatformIO serial monitor first: only one program can have
% the port open. If a run is interrupted (Ctrl+C), just run it again;
% the old port object is cleared below.

clear s

%% Settings
PORT      = "";       % "" = auto-detect, or e.g. "/dev/cu.usbserial-0001" / "COM3"
BAUD      = 115200;   % must match Serial.begin() in src/main.cpp
PRE_S     = 0.25;     % keep this long before the motor turns on (firmware logs 0.5 s)
POST_S    = 0.5;      % keep this long after the shaft comes to rest
STILL_DEG = 0.5;      % shaft is at rest once the angle moves less than this
STILL_S   = 0.25;     % ... for this long (one encoder count = 0.37 deg at 20.4:1)
NAME_TAG  = "";       % added to the file name, e.g. "Kp10"
OUT_DIR   = fullfile(fileparts(mfilename("fullpath")), "data");

%% Open the port
if PORT == ""
    ports = serialportlist("available");
    pick = ports(contains(ports, ["usbserial", "SLAB", "wchusbserial", "usbmodem", "ttyUSB", "ttyACM"]) ...
                 & ~startsWith(ports, "/dev/tty."));   % macOS: use cu.*, not tty.*
    if isempty(pick) && ispc
        pick = ports;                                   % Windows lists plain COMx
    end
    if numel(pick) ~= 1
        error("Found %d possible ESP32 ports (available: %s). Set PORT by hand.", ...
              numel(pick), strjoin(ports, ", "));
    end
    PORT = pick;
end

s = serialport(PORT, BAUD, "Timeout", 3);
configureTerminator(s, "CR/LF");

% Release the auto-reset lines so the ESP32 runs normally (it may reboot here)
setDTR(s, false);
setRTS(s, false);

%% Start the trial
% Send 's' on every READY prompt until the CSV header comes back. READY is
% printed every 2 s while idle, so a missed 's' (board still booting) is
% retried, and an extra 's' after the trial starts is ignored by the firmware.
fprintf("Waiting for ESP32 on %s...\n", PORT);
waitStart = tic;
headerLine = "";
while headerLine == ""
    if toc(waitStart) > 10
        clear s
        error("No response from the ESP32 on %s. Is the firmware uploaded and the serial monitor closed?", PORT);
    end
    line = readline(s);
    if isempty(line)
        continue
    elseif contains(line, "READY")
        write(s, "s", "char");
    elseif startsWith(line, "time_s")
        headerLine = line;
    end
end
header = split(headerLine, ",")';

%% Record until END
nCol = numel(header);
raw  = strings(0, 1);     % rows exactly as the ESP32 printed them
data = zeros(0, nCol);

fprintf("Recording, t = ");
lastSec = -1;
while true
    line = readline(s);
    if isempty(line)
        warning("Serial stream stopped before END; keeping what was received.");
        break
    end
    if startsWith(line, "END")
        break
    end

    v = sscanf(line, "%f,")';
    if numel(v) ~= nCol
        continue          % not a complete data row
    end
    raw(end+1, 1)  = line;
    data(end+1, :) = v;

    if floor(v(1)) > lastSec
        lastSec = floor(v(1));
        fprintf("%d ", lastSec);
    end
end
fprintf("\n");
clear s                    % close the port

if isempty(data)
    error("No data rows were received.");
end

%% Find motor on, motor off, and shaft at rest
T   = array2table(data, "VariableNames", header);
t   = T.time_s;
ang = T.angle_deg;
pwm = T.pwm;

iOn = find(pwm ~= 0, 1);
if isempty(iOn)
    error("The motor never turned on (pwm = 0 in every row).");
end

% Motor off = first row after which pwm stays at 0
iOff  = find(pwm ~= 0, 1, "last") + 1;
iRest = [];
if iOff > numel(t)
    warning("Motor was still on in the last row, so there is no motor-off data. " + ...
            "Upload the current src/main.cpp (it logs POST_STOP_US with the motor off).");
else
    % At rest = first row after motor off where the angle then stays
    % within STILL_DEG for STILL_S
    nStill = round(STILL_S / median(diff(t)));
    for k = iOff : numel(t) - nStill
        w = ang(k : k + nStill);
        if max(w) - min(w) <= STILL_DEG
            iRest = k;
            break
        end
    end
    if isempty(iRest)
        warning("Shaft never came to rest after the motor turned off; keeping all data after it.");
    end
end

iStart = find(t >= t(iOn) - PRE_S, 1);
if isempty(iRest)
    iEnd = numel(t);
else
    iEnd = find(t <= t(iRest) + POST_S, 1, "last");
end
keep  = (iStart:iEnd)';
trial = T(keep, :);

%% Save
if ~isfolder(OUT_DIR)
    mkdir(OUT_DIR);
end
name = "trial_" + string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
if NAME_TAG ~= ""
    name = name + "_" + NAME_TAG;
end
file = fullfile(OUT_DIR, name + ".csv");

fid = fopen(file, "w");
fprintf(fid, "%s\n", headerLine);
fprintf(fid, "%s\n", raw(keep));
fclose(fid);

fprintf("Motor on at %.3f s", t(iOn));
if iOff <= numel(t)
    fprintf(", off at %.3f s", t(iOff));
end
if ~isempty(iRest)
    fprintf(", at rest at %.3f s", t(iRest));
end
fprintf("\nSaved %d rows (%.3f to %.3f s) to %s\n", numel(keep), t(iStart), t(iEnd), file);
