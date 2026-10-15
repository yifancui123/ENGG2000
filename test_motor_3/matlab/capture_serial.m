function lines = capture_serial(port, outfile, command, timeout_s)
%CAPTURE_SERIAL  Record one test from the Nano straight into a CSV file.
%   capture_serial("COM5", "data/step_150.csv", "150", 15)
%   port      - Arduino IDE: Tools > Port (close the Serial Monitor first!)
%   command   - text to send after the Nano resets ("" for auto-start sketches)
%   timeout_s - give up after this many seconds
%   Stops when the sketch prints "# end".

if nargin < 3, command = ""; end
if nargin < 4, timeout_s = 30; end

s = serialport(port, 115200, "Timeout", 1);
configureTerminator(s, "CR/LF");
cleanup = onCleanup(@() delete(s));

pause(2.5);                          % opening the port resets the Nano
flush(s);
if strlength(command) > 0
    writeline(s, command);
end

lines = strings(0, 1);
t0 = tic;
while toc(t0) < timeout_s
    if s.NumBytesAvailable == 0
        pause(0.01);
        continue
    end
    L = readline(s);
    if isempty(L), continue, end
    L = strtrim(L);
    lines(end + 1, 1) = L; %#ok<AGROW>
    if L == "# end", break, end
end

if isempty(lines) || lines(end) ~= "# end"
    warning("No '# end' received from %s - is the right sketch uploaded?", port);
end
writelines(lines, outfile);
fprintf("Saved %d lines to %s\n", numel(lines), outfile);
end
