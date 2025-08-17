clear all;
close all;
clc;

% Coefficients for the model
Ts = 1/(48e3);

% Smoothing factor
alpha = 0.1;

% EMA filter coefficients
b = alpha;
a = [1 -(1-alpha)];

% Calculate frequency response with freqz
[H, w] = freqz(b, a, 2048);

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
t = (0:Ts:0.5);

f_passband = 500;
f_stopband = 8000;

signal_passband = sin(2*pi*f_passband*t);
signal_stopband = sin(2*pi*f_stopband*t);

% Simulates the values of the ADC
x = signal_passband;
%x = uint16(((sin(2*pi*f_passband*t) + sin(2*pi*f_stopband*t)) / 2) * 2047 + 2048);

% Test the filter
y = ema_filter(x, alpha);

% Plot the results
figure;
subplot(2, 1, 1);
plot(t, x);
xlabel('Time [s]');
ylabel('Amplitude');
title('Input signal');
xlim([0 0.01]);

subplot(2, 1, 2);
plot(t, y);
xlabel('Time [s]');
ylabel('Amplitude');
xlim([0 0.01]);
title('Output signal');

% Exports input signal as array in C
x_short = x(1:1024);

% Öffne eine neue Datei (optional, oder direkt in der Konsole ausgeben)
fprintf('uint16_t adc_values[1024] = {\n');
for i = 1:1024
    if mod(i, 16) == 1
        fprintf('    '); % Einrückung
    end
    fprintf('%d', x_short(i));
    if i < 1024
        fprintf(', ');
    end
    if mod(i, 16) == 0
        fprintf('\n');
    end
end
fprintf('\n};\n');

% e)
% The higher frequency (here: 8kHz) is attenuated and the lower frequency
% (here: 500Hz) is preserved

% f)
% When alpha gets larger:
% - The filter responds faster
% - The smoothing effect decreases
% - Higher cutoff frequency -> more frequency components pass through
% - smaller phase delay
% Reason: With a smaller alpha, the filter relies more on the past outputs
% (1-alpha). A larger alpha gives more weight to the current input.