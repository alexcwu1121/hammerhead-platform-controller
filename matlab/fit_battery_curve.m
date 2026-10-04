clear all; close all;

% Fit battery curve
% HRB 3300mAh 4S LiPo battery pack. No datasheet...

% cutoff voltage at which SOC is considered 0.
v_cutoff = 12.4;

% Battery data series
% Consider SOC to just be discharged energy at constant load normalized to
% discharged energy at the cutoff voltage.
% Run experiment recording voltage periodically for a constant load.

% I made this up because I don't want to run a discharge test on this
% battery
voltage_data = [
    16.8000;
    15.8000;
    15.2000;
    15.1000;
    15.0000;
    14.9692;
    14.9385;
    14.9077;
    14.8769;
    14.8462;
    14.8154;
    14.7846;
    14.7538;16.2
    14.7231;
    14.6923;
    14.6615;
    14.6308;
    14.4000;
    13.6000;
    12.4000;
    12.2
    ];

% strip out voltages less than cutoff
voltage_data(voltage_data < v_cutoff) = [];

% Assume battery started at 100% SOC and consider voltage cutoff as 0% SOC
soc = linspace(1, 0, size(voltage_data, 1));

v_soc_coeffs = polyfit(voltage_data, soc, 4);
soc_v_coeffs = polyfit(soc, voltage_data, 4);

v_sample = linspace(min(voltage_data), max(voltage_data), 100);
v_soc_plot = polyval(v_soc_coeffs, v_sample);

soc_sample = linspace(1, 0, 100);
soc_v_plot = polyval(soc_v_coeffs, soc_sample);

figure(1);
plot(v_sample, v_soc_plot);
hold on;
scatter(voltage_data, soc);
title("V by SOC");
xlabel("Voltage (V)");
ylabel("SOC (%)");
set(gca, 'XDir', 'reverse');

% figure(2);
% plot(soc_sample, soc_v_plot);
% hold on;
% scatter(soc, voltageData);
% title("SOC by V");
% xlabel("SOC (%)");
% ylabel("Voltage (V)");
% set(gca, 'XDir', 'reverse');

% logistic fit
% sigModel = fittype('a / (1 + exp(-b * (x - c)))', ...
%     'coefficients', {'a', 'b', 'c'}, 'independent', 'x');
% [fitResult, gof] = fit(voltageData, soc', sigModel, 'StartPoint', [1, 1, 5]);
%
% figure(3);
% plot(fitResult, voltageData, soc);
