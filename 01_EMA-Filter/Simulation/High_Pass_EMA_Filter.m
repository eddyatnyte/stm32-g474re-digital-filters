clear all;
close all;
clc;

% Coefficients for the model
Ts = 1/(48e3);

% Smoothing factor
alpha = 0.4;

% EMA filter coefficients
b = 1 - alpha;
a = [1 (1-alpha)];

% Calculate frequency response with freqz
[H, w] = freqz(b, a);

% Magnitude in dB
H_dB = 20*log10(abs(H));

% Maximum magnitude (at 0 Hz)
max_dB = max(H_dB);

% 3 dB cutoff threshold
cutoff_dB = max_dB - 3;

% Find index closest to 3 dB point
[~, idx] = min(abs(H_dB - cutoff_dB));

% Normalized cutoff frequency (w/pi)
f_3dB_norm = w(idx)/pi;

% Plot magnitude and phase using freqz
figure;
freqz(b, a);
hold on;

% Add red circle at 3 dB cutoff on magnitude plot
subplot(2,1,1) % Magnitude plot
plot(f_3dB_norm, H_dB(idx), 'ro', 'MarkerSize', 10, 'LineWidth', 2);

% Add legend
legend('Frequency Response', '3 dB Cutoff Frequency');

% Defines the test signal
t = (0:Ts:0.15);

f_passband = 19200;
f_stopband = 500;

signal_passband = sin(2*pi*f_passband*t);
signal_stopband = sin(2*pi*f_stopband*t);

% Simulates the values of the ADC
x = sin(2*pi*f_passband*t) + sin(2*pi*f_stopband*t);

% Test the filter
y_LP = ema_filter(x, alpha);
y_HP = x - y_LP;

% Plot the results
figure;
subplot(2, 1, 1);
plot(t, x);
xlabel('Time [s]');
ylabel('Amplitude');
title('Input signal');
xlim([0 0.01]);

subplot(2, 1, 2);
plot(t, y_HP);
xlabel('Time [s]');
ylabel('Amplitude');
xlim([0 0.01]);
title('Output signal');

% l)
% The lower frequency (here: 500 Hz) is attenuated and the higher frequency
% (here: 8 kHz) is preserved

% m)
% A larger alpha leads to an opposite behvavior in comparison to the
% low pass filter. If alpha increases, the low components of the signal are
% attenuated and only the high components get through the filter. A
% decreasing alpha also decreases the effect of the HP-Filter which leads
% to preserving the shape of the original signal.
% The reason for this behavior is the nominator of the HP-Filter (1 - alpha),
% whereas the nominator for the LP-Filter is only alpha. This results in an
% inverted behavior.




