function analyse_motor_tests(datadir)
%ANALYSE_MOTOR_TESTS  Motor model from the bench tests.
%   Reads datadir/step_*.csv, deadband.csv, speed_curve.csv (any can be missing).
%   Fits each step to a first-order model  w(t) = w_ss * (1 - exp(-t/tau)),
%   measures stopping time for brake vs coast, and fits the speed curve.
%   Prints a summary, saves plots and summary.txt into datadir.

if nargin < 1
    datadir = fullfile(fileparts(mfilename("fullpath")), "data");
end
CPR = 2800;                          % counts per wheel rev (700 x 4) - confirm!
out = strings(0, 1);
say = @(varargin) fprintf("%s\n", sprintf(varargin{:}));

%% Step tests -------------------------------------------------------------
files = dir(fullfile(datadir, "step_*.csv"));
if ~isempty(files)
    fig = figure("Name", "Step responses", "Color", "w"); light(fig); hold on
    out(end + 1) = "STEP TESTS (first-order fit)";
    out(end + 1) = sprintf("%-10s %6s %10s %8s %12s %10s %12s", ...
        "file", "pwm", "w_ss_rpm", "tau_s", "K_rpm/pwm", "stop", "stop_time_s");
    for f = files'
        [t, rpm, pwm, stopMode, stepPwm] = read_step(fullfile(f.folder, f.name), CPR);
        on  = find(pwm ~= 0, 1, "first");
        off = find(pwm == 0 & (1:numel(pwm))' > on, 1, "first");
        if isempty(on) || isempty(off), continue, end

        % first-order fit on the driven part
        tt = t(on:off - 1) - t(on);
        ww = rpm(on:off - 1);
        wss0 = mean(ww(tt > tt(end) - 0.5));
        cost = @(x) sum((ww - x(1) * (1 - exp(-tt / max(x(2), 1e-3)))).^2);
        x = fminsearch(cost, [wss0, 0.3]);
        wss = x(1); tau = x(2);

        % stopping time: first time after stop that |rpm| < 3% of w_ss
        tStop = t(off:end) - t(off);
        idx = find(abs(rpm(off:end)) < 0.03 * abs(wss), 1, "first");
        stopTime = NaN;
        if ~isempty(idx), stopTime = tStop(idx); end

        stopTxt = sprintf("%.2f", stopTime);
        if isnan(stopTime), stopTxt = sprintf("> %.1f", tStop(end)); end
        out(end + 1) = sprintf("%-10s %6d %10.1f %8.3f %12.3f %10s %12s", ...
            erase(f.name, [".csv", "step_"]), stepPwm, wss, tau, wss / stepPwm, ...
            stopMode, stopTxt); %#ok<AGROW>
        plot(t - t(on), rpm, "LineWidth", 1.5, ...
            "DisplayName", sprintf("PWM %d, %s", stepPwm, stopMode));
    end
    xlabel("Time from step (s)"); ylabel("Wheel speed (rpm)");
    title("Step response and stopping (brake vs coast)", "FontWeight", "normal");
    grid on; legend("Location", "eastoutside", "Box", "off");
    saveas(fig, fullfile(datadir, "step_responses.png"));
end

%% Deadband ---------------------------------------------------------------
f = fullfile(datadir, "deadband.csv");
if isfile(f)
    T = readtable(f, "CommentStyle", "#", "TextType", "string");
    out(end + 1) = "";
    out(end + 1) = "DEADBAND (mean of trials)";
    for d = ["forward", "reverse"]
        r = T(T.direction == d, :);
        out(end + 1) = sprintf("%-8s breakaway PWM %5.1f (range %d-%d), stop PWM %5.1f", ...
            d, mean(r.breakaway_pwm), min(r.breakaway_pwm), max(r.breakaway_pwm), ...
            mean(r.stop_pwm)); %#ok<AGROW>
    end
end

%% Speed curve ------------------------------------------------------------
f = fullfile(datadir, "speed_curve.csv");
if isfile(f)
    T = readtable(f, "CommentStyle", "#");
    fig = figure("Name", "Speed curve", "Color", "w"); light(fig); hold on
    plot(T.pwm, T.rpm, "o-", "LineWidth", 1.5, "DisplayName", "measured");
    out(end + 1) = "";
    out(end + 1) = "SPEED CURVE (straight-line fit where the wheel turns)";
    for s = [1, -1]
        r = T(sign(T.pwm) == s & abs(T.rpm) > 5, :);
        if height(r) < 2, continue, end
        c = polyfit(r.pwm, r.rpm, 1);
        out(end + 1) = sprintf("%-8s rpm = %.3f * pwm %+.1f  -> zero-speed PWM %.0f, max %.0f rpm", ...
            ternary(s > 0, "forward", "reverse"), c(1), c(2), -c(2) / c(1), ...
            max(abs(r.rpm))); %#ok<AGROW>
        p = linspace(min(r.pwm), max(r.pwm), 50);
        plot(p, polyval(c, p), "--", "DisplayName", "fit " + ternary(s > 0, "fwd", "rev"));
    end
    xlabel("PWM (signed)"); ylabel("Steady wheel speed (rpm)");
    title("Wheel speed vs PWM", "FontWeight", "normal");
    grid on; legend("Location", "northwest", "Box", "off");
    saveas(fig, fullfile(datadir, "speed_curve.png"));
end

%% Summary ----------------------------------------------------------------
if isempty(out)
    say("No test data found in %s", datadir);
    return
end
fprintf("\n%s\n", strjoin(out, newline));
writelines(out, fullfile(datadir, "summary.txt"));
say("\nSaved summary.txt and plots to %s", datadir);
end

%% ---- helpers ----
function [t, rpm, pwm, stopMode, stepPwm] = read_step(file, CPR)
txt = fileread(file);
tok = regexp(txt, "# step pwm=(-?\d+) stop=(\w+)", "tokens", "once");
stepPwm = str2double(tok{1});
stopMode = string(tok{2});
T = readtable(file, "CommentStyle", "#");
t = T.t_ms / 1000;
pwm = T.pwm;
rpm = [0; diff(T.count) ./ diff(t)] / CPR * 60;
rpm = movmean(rpm, 5);                        % smooth 25 ms of encoder steps
end

function light(f)
try, theme(f, "light"); catch, end
end

function v = ternary(c, a, b)
if c, v = a; else, v = b; end
end
