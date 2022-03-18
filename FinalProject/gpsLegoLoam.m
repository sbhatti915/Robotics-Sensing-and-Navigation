% Sameer Bhatti
% bhatti.sa@northeastern.edu
% 4/21/2021
% gpsLegoLoam.m
%
% Plots GPS data
clc
clear
close all

%% Declarations
% Read Bags
gpsBag = rosbag('nu_gps.bag');
gpsSel = select(gpsBag,'Topic','/vehicle/gps/fix');
gpsStructs = readMessages(gpsSel,'DataFormat','struct');

lat = cellfun(@(m) double(m.Latitude),gpsStructs);
lon = cellfun(@(m) double(m.Longitude),gpsStructs);
alt = cellfun(@(m) double(m.Altitude),gpsStructs);
sec = cellfun(@(m) double(m.Header.Stamp.Sec),gpsStructs);
nsec = cellfun(@(m) double(m.Header.Stamp.Nsec),gpsStructs);

time = sec+nsec*10^-9;

%% Calculations
% Find Time
time = sec+nsec*10^-9;
time = time - time(1);

% Convert to utm
[easting,northing] = ll2utm(lat,lon); % François Beauducel (2021). LL2UTM and UTM2LL (https://www.mathworks.com/matlabcentral/fileexchange/45699-ll2utm-and-utm2ll), MATLAB Central File Exchange. Retrieved April 23, 2021.

%% Plot
plot(easting,northing)
title('Car Data GPS')
xlabel('Easting (m)','FontSize',12)
ylabel('Northing (m)','FontSize',12)

figure
plot(time,alt)
title('Altitude GPS')
xlabel('time (s)','FontSize',12)
ylabel('Altitude (m)','FontSize',12)