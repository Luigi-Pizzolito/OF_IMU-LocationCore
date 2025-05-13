% CONFIGURATION
serial_port = "/dev/tty";
baud_rate = 115200;
timeout = 20;
max_points = 20000;

% Open serial
s = serial(serial_port, baud_rate, "timeout", timeout);
fopen(s);

% Initialize time and data buffers
t = [];
accel_x = []; accel_y = []; accel_z = [];
of_x = []; of_y = [];
quat_w = []; quat_x = []; quat_y = []; quat_z = [];

% Prepare 3 vertically stacked subplots
figure;
subplot(3,1,1); accel_plot = plot(NaN,NaN,'-r', NaN,NaN,'-g', NaN,NaN,'-b');
ylabel('Accel'); legend('x','y','z'); grid on;

subplot(3,1,2); of_plot = plot(NaN,NaN,'--m', NaN,NaN,'--k');
ylabel('Optical Flow'); legend('x','y'); grid on;

subplot(3,1,3); quat_plot = plot(NaN,NaN,'-r', NaN,NaN,'-g', NaN,NaN,'-b', NaN,NaN,'-k');
ylabel('Quat'); legend('w','x','y','z'); xlabel('Time (s)'); grid on;

disp("Reading and plotting serial data...");

while ishandle(accel_plot(1))
    line = fgetl(s);
    if isempty(line) || ~ischar(line)
        continue;
    endif

    disp(line);

##    tokens = strsplit(line, ",");
##    if length(tokens) < 11
##        continue;  % skip malformed
##    endif
##
##    millis = str2double(tokens{2});
##    if isnan(millis)
##        continue;
##    endif
##    t(end+1) = millis / 1000;
##
##    % Parse and fall back to previous value or 0
##    ax = parse_or_last(tokens{3}, accel_x);
##    ay = parse_or_last(tokens{4}, accel_y);
##    az = parse_or_last(tokens{5}, accel_z);
##
##    qw = parse_or_last(tokens{6}, quat_w);
##    qx = parse_or_last(tokens{7}, quat_x);
##    qy = parse_or_last(tokens{8}, quat_y);
##    qz = parse_or_last(tokens{9}, quat_z);
##
##    ofx = parse_or_last(tokens{10}, of_x);
##    ofy = parse_or_last(tokens{11}, of_y);
##
##    accel_x(end+1) = ax; accel_y(end+1) = ay; accel_z(end+1) = az;
##    quat_w(end+1) = qw; quat_x(end+1) = qx; quat_y(end+1) = qy; quat_z(end+1) = qz;
##    of_x(end+1) = ofx; of_y(end+1) = ofy;
##
##    % Trim to fixed buffer
##    [t, accel_x, accel_y, accel_z, of_x, of_y, quat_w, quat_x, quat_y, quat_z] = ...
##        trim_buffers(max_points, t, accel_x, accel_y, accel_z, of_x, of_y, quat_w, quat_x, quat_y, quat_z);
##
##    % Update each subplot
##    subplot(3,1,1);
##    set(accel_plot(1), 'XData', t, 'YData', accel_x);
##    set(accel_plot(2), 'XData', t, 'YData', accel_y);
##    set(accel_plot(3), 'XData', t, 'YData', accel_z);
##
##    subplot(3,1,2);
##    set(of_plot(1), 'XData', t, 'YData', of_x);
##    set(of_plot(2), 'XData', t, 'YData', of_y);
##
##    subplot(3,1,3);
##    set(quat_plot(1), 'XData', t, 'YData', quat_w);
##    set(quat_plot(2), 'XData', t, 'YData', quat_x);
##    set(quat_plot(3), 'XData', t, 'YData', quat_y);
##    set(quat_plot(4), 'XData', t, 'YData', quat_z);
##
##    drawnow;
endwhile



% Cleanup
fclose(s);
delete(s);
clear s;
