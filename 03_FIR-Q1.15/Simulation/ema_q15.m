clear all;
close all;
clc;

% --- Parameters ---
fs = 48000;                  % Sampling rate [Hz]
f_sig = 500;                 % Signal frequency [Hz]
N = round(1000 * 500 / f_sig); % Number of samples (adjusted to constant time span)
alpha_f = 0.5;              % EMA smoothing factor (float)

% --- Time axis and input signal (float, not Q15) ---
t = (0:N-1) / fs;
x = 0.9 * sin(2 * pi * f_sig * t) + 1.0;  % Sine wave with amplitude ±0.9 V and 1 V offset

% --- FLOATING-POINT EMA ---
y_float = zeros(1, N);
y_float(1) = 1;
for n = 2:N
    y_float(n) = alpha_f * x(n) + (1 - alpha_f) * y_float(n - 1);
end

% --- Q15 FIXED-POINT EMA ---
alpha_q15 = int16(round(alpha_f * 32768));     % alpha in Q15 format
x_q15 = int16(round((x - 1.0) / 0.9 * 32768));        % Scale float input to Q15 range [-1, 1]

y_q15 = zeros(1, N, 'int16');
y_q15(1) = int16(0);
for n = 2:N
    mult1 = int32(alpha_q15) * int32(x_q15(n));
    mult2 = int32(32767 - alpha_q15) * int32(y_q15(n - 1));
    tmp = bitshift(mult1 + mult2, -15);
    tmp = min(max(tmp, -32768), 32767);
    y_q15(n) = int16(tmp);
end

% --- Convert Q15 output back to float for plotting ---
y_q15_float = double(y_q15) / 32768 * 0.9 + 1.0;  % Scale back 

% --- Plot: input and both filtered outputs ---
figure;
plot(t * 1e3, x, 'k:', 'DisplayName', 'Input x[n]');
hold on;
plot(t * 1e3, y_float, 'b-', 'LineWidth', 1.2, 'DisplayName', 'EMA Float');
plot(t * 1e3, y_q15_float, 'r--', 'LineWidth', 1.2, 'DisplayName', 'EMA Q15');
xlim([0 max(t*1e3)]);
legend;
xlabel('Time [ms]');
ylabel('Amplitude [V]');
title(['EMA Filter on ' num2str(f_sig) ' Hz Sine @ ' num2str(fs/1000) ' kHz']);
grid on;

% --- Calculate peak-to-peak values ---
vpp_x = max(x) - min(x);
vpp_float = max(y_float) - min(y_float);
vpp_q15 = max(y_q15_float) - min(y_q15_float);

% --- Prepare text content ---
str = sprintf('Vpp Input: %.3f V\nVpp Float: %.3f V\nVpp Q15: %.3f V', ...
              vpp_x, vpp_float, vpp_q15);

% --- Position in bottom right corner ---
xpos = t(end)*1e3 * 0.95;  % 95% of time range
ymin = min([x, y_float, y_q15_float]);
ymax = max([x, y_float, y_q15_float]);
ypos = ymin + 0.05 * (ymax - ymin);  % 5% above the bottom

% --- Display text in the plot ---
text(xpos, ypos, str, ...
    'FontSize', 12, ...
    'HorizontalAlignment', 'right', ...
    'VerticalAlignment', 'bottom', ...
    'BackgroundColor', 'w', ...
    'EdgeColor', [0.8 0.8 0.8], ...
    'Margin', 6);

% --- Plot error between float and Q15 output ---
e = y_float - y_q15_float;
figure;
plot(t * 1e3, e);
title('Error: y\_float - y\_q15');
xlabel('Time [ms]');
ylabel('Error [V]');
grid on;

