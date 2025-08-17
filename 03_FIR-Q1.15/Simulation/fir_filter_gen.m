clear all;
close all;
clc;

% FIR filter parameters
N = 16;        % Order of the FIR filter
fc = 2000;      % Cutoff frequency [Hz]
fs = 48000;     % Sampling frequency [Hz]

% Generate FIR filter coefficients
b = fir1(N-1, fc/(fs/2));

% Frequency response
freqz(b, 1);         

% Convert the coefficients into C code
filename = 'fir_coeffs.txt';  % oder 'fir_coeffs.h'
fid = fopen(filename, 'w');

fprintf(fid, 'const float coeffs[N] = {\n');

for i = 1:length(b)
    if i < length(b)
        fprintf(fid, '    %.10ff,\n', b(i));  % mit f-Suffix für float
    else
        fprintf(fid, '    %.10ff\n', b(i));   % ohne Komma beim letzten
    end
end

fprintf(fid, '};\n');
fclose(fid);