% Sameer Bhatti
% bhatti.sa@northeastern.edu
% 2/18/2021
% gpsAnalysis.m
%
% Plots and conducts statistical analysis of GPS data
clc
clear
close all

%% Declarations
% Read Bags
movingBag = rosbag('moving_data_parsed.bag');
statBag = rosbag('stationary_data_parsed.bag');

%% Extract Moving GPS Data

movSel = select(movingBag,'Topic','/gps');
movStructs = readMessages(movSel,'DataFormat','struct');

movEast = cellfun(@(m) double(m.UtmEasting),movStructs);
movNorth = cellfun(@(m) double(m.UtmNorthing),movStructs);

%% Extract Stationary GPS Data

statSel = select(statBag,'Topic','/gps');
statStructs = readMessages(statSel,'DataFormat','struct');

statEast = cellfun(@(m) double(m.UtmEasting),statStructs);
statNorth = cellfun(@(m) double(m.UtmNorthing),statStructs);
statEast(655) = [];
statNorth(655) = [];

%% Plots
% Interpolate northing - moving
p = polyfit(movEast,movNorth,1);
f1 = polyval(p,movEast);

% UTM - Moving plot
figure
plot(movEast,movNorth)
hold on
plot(movEast,f1)
title('UTM - Moving')
xlabel('Easting')
ylabel('Northing')
legend('GPS Data', 'Interpolated','mean')
hold on
plot(mean(movEast),mean(movNorth),'o')

% Northing error
err = abs(movNorth-f1);
figure
plot(err)
hold on

% Interpolate Easting - moving
p = polyfit(movNorth,movEast,1);
f1 = polyval(p,movNorth);
err = abs(movEast-f1);
plot(err)
title('Error in UTM - Moving')
xlabel('Seq')
ylabel('Error (utm)')
legend('Northing error','Easting error')

% plot stationary utm
figure
scatter(statEast,statNorth)
xlim([3.305415e5 3.305417e5])
ylim([4.6754245e6 4.6754265e6])
title('UTM - Stationary')
xlabel('Easting')
ylabel('Northing')
hold on

% Interpolate Northing - stationary
p2 = polyfit(statEast,statNorth,1);
f2 = polyval(p2,statEast);
plot(statEast,f2)
hold on
plot(mean(statEast),mean(statNorth),'o')
legend('GPS Data', 'Interpolated','mean')

figure
err = abs(statNorth-f2);
plot(err)

hold on
p = polyfit(statNorth,statEast,1);
f1 = polyval(p,statNorth);
err = abs(statEast-f1);
plot(err)
title('Error in UTM - stationary')
xlabel('Seq')
ylabel('Error (utm)')
legend('Northing error','Easting error')