clear all;
close all;
clc;

%% Filter parameters
fs = 48000;            % Sampling rate [Hz]
f0 = 500;              % Frequency [Hz]
w0 = 2*pi*f0/fs;       % Angular frequency [rad/sample]

% Numerator coefficients
a = [0 sin(w0)];

% Denominator coefficients
b = [1 -2*cos(w0) 1];

%% Oscillator with impulse excitation (resonance at f0)
N = 0.15 * fs;         % Length in samples (~150 ms)
x = zeros(1, N);       
x(1) = 2042;              % Impulse excitation at n = 0

%% Calculate oscillator signal (recursive)
y = filter(a, b, x);

%% Scale to 12-bit range [0, 4095]

%% Plot result
t = (0:N-1)/fs;

figure;
subplot(2,1,1);
plot(t, x);
title('Excitation (Impulse)');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;

subplot(2,1,2);
plot(t, y + 2042);
title('Oscillator Output (Scaled to 12-bit)');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;

%% Frequency-domain visualization
nfft = 2^nextpow2(N);
Y = fft(y, nfft);
f_axis = fs*(0:nfft/2-1)/nfft;

figure;
plot(f_axis, 20*log10(abs(Y(1:nfft/2))), 'r');
title('Oscillator Spectrum');
xlabel('Frequency (Hz)');
ylabel('Amplitude (dB)');
grid on;