% REAL-TIME ANIMATED Ball Levitation with Performance Metrics
% -----------------------------------------------------------
% - Animation: Synced to Real-Time (20 seconds duration)
% - Calculates Rise Time, Overshoot, Settling Time, SS Error at the end.

clear; clc; close all;

%% 1. Configuration
Kp = 2.0;   
Ki = 0.2;  
Kd = 0.50; 

Setpoint = 15.0;     % Target distance from TOP (cm)
TubeHeight = 70.0;   

% Fan Settings
MIN_PWM = 20; 
MAX_PWM = 255;

%% 2. Physics Setup
dt = 0.02;           % Time step (20ms)
T_final = 20;        % Run for 20 seconds
steps = T_final / dt;

m = 0.0027;          
g = 9.81;            

y = TubeHeight;      % Start at Bottom
v = 0;               

integral = 0;
lastError = 0;

force_factor = (m * g) / 30; 

%% 3. Visualization Setup
figure('Color', 'w', 'Name', 'Real-Time Simulation', 'Position', [100, 100, 1000, 600]);

% --- Left Panel: Animation ---
subplot(1, 2, 1);
hold on;
axis([-10 10 0 TubeHeight+5]); 
set(gca, 'YDir', 'reverse');    
set(gca, 'Color', [0.95 0.95 0.95]);
xlabel('Tube'); 
ylabel('Distance from Sensor (cm)');
title('Live Animation (Real-Time)');

plot([-6 -6], [0 TubeHeight], 'k', 'LineWidth', 3);
plot([6 6], [0 TubeHeight], 'k', 'LineWidth', 3);
rectangle('Position', [-3, -2, 6, 2], 'FaceColor', 'b');
rectangle('Position', [-6, TubeHeight, 12, 3], 'FaceColor', [0.2 0.2 0.2]);
text(0, TubeHeight+2, 'FAN', 'Color', 'w', 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
plot([-6 6], [Setpoint Setpoint], 'g--', 'LineWidth', 2);
text(-7, Setpoint, 'Target', 'Color', 'g', 'HorizontalAlignment', 'right', 'FontWeight', 'bold');

hBall = plot(0, y, 'ro', 'MarkerSize', 25, 'MarkerFaceColor', 'r');

% --- Right Panel: Graph ---
subplot(1, 2, 2);
hGraph = plot(0,0,'b', 'LineWidth', 2);
hold on;
plot([0 T_final], [Setpoint Setpoint], 'g--', 'LineWidth', 1.5);
grid on;
title('Response vs Time');
xlabel('Time (s)');
ylabel('Distance (cm)');
xlim([0 T_final]);
ylim([0 TubeHeight]);
set(gca, 'YDir', 'reverse'); 

%% 4. Main Simulation Loop
time_hist = [];
pos_hist = [];

disp('Starting Real-Time Animation...');

for i = 1:steps
    tic; % Start timer for this frame
    
    % --- PID CALCULATION ---
    input = y;
    error = input - Setpoint; 
    
    integral = integral + (error * dt);
    
    % Increased limit so I-term works correctly
    integral = max(min(integral, 1000), -1000); 
    
    derivative = (error - lastError) / dt;
    
    output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    
    pwm = max(min(output, MAX_PWM), MIN_PWM);
    
    lastError = error;
    
    % --- PHYSICS ---
    F_gravity = m * g;
    F_fan = pwm * force_factor; 
    F_net = F_gravity - F_fan;
    
    accel = F_net / m;
    accel = accel - (1.0 * v); 
    
    v = v + accel * dt;
    y = y + v * dt;
    
    if y > TubeHeight, y = TubeHeight; v = 0; end
    if y < 2, y = 2; v = 0; end
    
    % --- STORE DATA ---
    time_hist = [time_hist, i*dt];
    pos_hist = [pos_hist, y];
    
    % --- ANIMATION ---
    set(hBall, 'YData', y);
    set(hGraph, 'XData', time_hist, 'YData', pos_hist);
    
    % Force update
    drawnow; 
    
    % --- REAL-TIME SYNC ---
    % Wait if the calculation was faster than 0.02s
    comp_time = toc;
    if comp_time < dt
        pause(dt - comp_time);
    end
end

disp('Simulation Finished. Calculating Metrics...');

%% 5. CALCULATE PERFORMANCE METRICS
start_val = TubeHeight;
final_val = Setpoint;
total_step = start_val - final_val; 

% A. RISE TIME (10% to 90%)
threshold_10 = start_val - 0.10 * total_step;
threshold_90 = start_val - 0.90 * total_step;

idx_10 = find(pos_hist <= threshold_10, 1);
idx_90 = find(pos_hist <= threshold_90, 1);

if ~isempty(idx_10) && ~isempty(idx_90)
    RiseTime = time_hist(idx_90) - time_hist(idx_10);
else
    RiseTime = NaN; 
end

% B. OVERSHOOT
min_val = min(pos_hist);
if min_val < Setpoint
    Overshoot_Val = Setpoint - min_val;
    Overshoot_Percent = (Overshoot_Val / total_step) * 100;
else
    Overshoot_Val = 0;
    Overshoot_Percent = 0;
end

% C. SETTLING TIME (2%)
tolerance = 0.02 * total_step;
upper_limit = Setpoint + tolerance;
lower_limit = Setpoint - tolerance;

last_bad_index = 0;
for k = length(pos_hist):-1:1
    if pos_hist(k) > upper_limit || pos_hist(k) < lower_limit
        last_bad_index = k;
        break;
    end
end

if last_bad_index < length(pos_hist) && last_bad_index > 0
    SettlingTime = time_hist(last_bad_index + 1);
else
    SettlingTime = NaN; 
end

% D. STEADY STATE ERROR
num_samples = 1.0 / dt; 
avg_final_pos = mean(pos_hist(end-num_samples:end));
SS_Error = avg_final_pos - Setpoint;

%% 6. DISPLAY RESULTS
fprintf('\n---------------------------------\n');
fprintf(' SYSTEM PERFORMANCE METRICS\n');
fprintf('---------------------------------\n');
fprintf('Rise Time (10-90%%):   %.3f s\n', RiseTime);
fprintf('Overshoot:            %.2f %%\n', Overshoot_Percent);
fprintf('Settling Time (2%%):   %.3f s\n', SettlingTime);
fprintf('Steady State Error:   %.3f cm\n', SS_Error);
fprintf('---------------------------------\n');

dim = [0.6 0.2 0.3 0.3];
str = {['Rise Time: ' num2str(RiseTime, '%.2f') ' s'], ...
       ['Overshoot: ' num2str(Overshoot_Percent, '%.1f') ' %'], ...
       ['Settling Time: ' num2str(SettlingTime, '%.2f') ' s'], ...
       ['SS Error: ' num2str(SS_Error, '%.3f') ' cm']};
annotation('textbox', dim, 'String', str, 'FitBoxToText', 'on', 'BackgroundColor', 'w');