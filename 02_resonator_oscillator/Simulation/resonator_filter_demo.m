clear; clc; close all;

%% Filter Parameters
fs = 48000;             % Sampling rate [Hz]
f0 = 500;               % Resonance frequency [Hz]
w0 = 2*pi*f0/fs;        % Resonance frequency in radians/sample
a = 0.99;               % Pole radius (0 < a < 1), controls bandwidth
b0 = 1/100;             % Gain factor (arbitrary, can be normalized later)

% Calculate denominator coefficients (feedback)
a1 = -2 * a * cos(w0);
a2 = a^2;

% Numerator coefficients (feedforward)
b = [b0, 0, -b0];

% Denominator coefficients
a_coeffs = [1, a1, a2];

%% Plot frequency response
[H, f] = freqz(b, a_coeffs, 2048, fs);
figure;
plot(f, 20*log10(abs(H)));
title('Frequency Response of the Resonator Filter');
xlabel('Frequency (Hz)');
ylabel('Amplitude (dB)');
grid on;
xlim([0 fs/2]);

%% Generate test signal (two sine waves)
t = 0:1/fs:0.15;   % Time vector for ~150 ms
x = uint16(((sin(2*pi*500*t) + sin(2*pi*8000*t)) / 2) * 2047 + 2048);  
% Simulates 12-bit ADC input signal (0–4095 range)

%% Apply filter
y = filter(b, a_coeffs, x);

%% Plot time-domain signals
figure;
subplot(2,1,1);
plot(t, x);
title('Original Signal');
xlabel('Time (s)');
ylabel('Amplitude');
xlim([0 0.01]);
grid on;

subplot(2,1,2);
plot(t, y);
title('Filtered Signal (Resonator)');
xlabel('Time (s)');
ylabel('Amplitude');
xlim([0 0.01]);
grid on;

%% Plot frequency spectra
nfft = 2^nextpow2(length(x));
X = fft(x, nfft);
Y = fft(y, nfft);
f_axis = fs*(0:nfft/2-1)/nfft;

figure;
plot(f_axis, 20*log10(abs(X(1:nfft/2))), 'b'); hold on;
plot(f_axis, 20*log10(abs(Y(1:nfft/2))), 'r');
legend('Original', 'Filtered');
title('Spectrum Before and After Filtering');
xlabel('Frequency (Hz)');
ylabel('Amplitude (dB)');
grid on;
