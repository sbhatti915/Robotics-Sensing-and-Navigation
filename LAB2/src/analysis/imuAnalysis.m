% Sameer Bhatti
% bhatti.sa@northeastern.edu
% 3/12/2021
% imuAnalysis.m
%
% Plots imu data
clc
clear
close all

%% Declarations
% Read Bags
magBag = rosbag('mag.bag');
imuBag = rosbag('imu.bag');
movingBag = rosbag('gps_imu.bag');

%% Extract imu Data

imuSel = select(imuBag,'Topic','/imu');
imuStructs = readMessages(imuSel,'DataFormat','struct');

linAccel = cellfun(@(m) struct(m.LinearAcceleration),imuStructs);
orientation = cellfun(@(m) struct(m.Orientation),imuStructs);
angVel = cellfun(@(m) struct(m.AngularVelocity),imuStructs);
sec = cellfun(@(m) double(m.Header.Stamp.Sec),imuStructs);
nsec = cellfun(@(m) double(m.Header.Stamp.Nsec),imuStructs);

xAccel = extractfield(linAccel,'X');
yAccel = extractfield(linAccel,'Y');
zAccel = extractfield(linAccel,'Z');

wQuat = extractfield(orientation,'W');
xQuat = extractfield(orientation,'X');
yQuat = extractfield(orientation,'Y');
zQuat = extractfield(orientation,'Z');

for k = 1:length(wQuat)
    eul(k,:) = quat2eul([wQuat(k) xQuat(k) yQuat(k) zQuat(k)],'XYZ');
end

roll = eul(:,1);
pitch = eul(:,2);
yaw = eul(:,3);

xGyro = extractfield(angVel,'X');
yGyro = extractfield(angVel,'Y');
zGyro = extractfield(angVel,'Z');

imuTime = sec - min(sec);
nsec = nsec*1e-9;

imuTime = imuTime + nsec;

zAccelAvg = mean(zAccel);
stdDevZAccel = std(zAccel);

%% Extract Magnetometer Data

magSel = select(magBag,'Topic','/magnet');
magStructs = readMessages(magSel,'DataFormat','struct');

magField = cellfun(@(m) struct(m.MagneticField),magStructs);
header = cellfun(@(m) struct(m.Header),magStructs);
xMagField = extractfield(magField,'X');
yMagField = extractfield(magField,'Y');
zMagField = extractfield(magField,'Z');
mag = [xMagField' yMagField' zMagField'];

%% Extract Moving GPS Data

movSel = select(movingBag,'Topic','/gps');
movStructs = readMessages(movSel,'DataFormat','struct');

movEast = cellfun(@(m) double(m.UtmEasting),movStructs);
movNorth = cellfun(@(m) double(m.UtmNorthing),movStructs);
movLat = cellfun(@(m) double(m.Latitude),movStructs);
movLon = cellfun(@(m) double(m.Longitude),movStructs);

gpsSec = cellfun(@(m) double(m.Header.Stamp.Sec),movStructs);
gpsNsec = cellfun(@(m) double(m.Header.Stamp.Nsec),movStructs);

gpsNsec = gpsNsec*1e-9;

gpsTime = gpsSec + gpsNsec;

gpsTime = gpsTime - min(gpsTime);

endCircle = round(imuTime(814));

endCircleInd = find(gpsTime > endCircle - 1 & gpsTime < endCircle);

%% Part 1
D(:,1) = xMagField(268:814);
D(:,2) = yMagField(268:814);
D(:,3) = zMagField(268:814);

% Ohad Gal (2021). fit_ellipse (https://www.mathworks.com/matlabcentral/fileexchange/3215-fit_ellipse), MATLAB Central File Exchange. Retrieved March 11, 2021.
[ellipse] = fit_ellipse(D(:,1),D(:,2));
yawMag = unwrap(-atan2(yMagField-ellipse.Y0_in,xMagField-ellipse.X0_in));
yawMag = yawMag + (yaw(1) - yawMag(1));
yawMag(1:814) = [];

yaw = unwrap(yaw);
yaw(1:814) = [];
yaw = yaw - yaw(1);
time = imuTime;
time(1:814) = [];
zGyro(1:814) = [];
fs = 40;

time = time-min(time);

yawGyro = unwrap(deg2rad(cumtrapz(time,-zGyro)));
yawGyroFilt = highpass(yawGyro,0.5,fs);
yawMagFilt = lowpass(yawMag,0.5,fs);

% https://sites.google.com/site/myimuestimationexperience/filters/complementary-filter
combinedFilt = 0.98*yawMagFilt + 0.02*yawGyroFilt;
x = find(time > 2 & time < 3);
combinedFilt = combinedFilt - combinedFilt(x(3));
%% Part 2
xAccel(1:814) = [];
yAccel(1:814) = [];
zAccel(1:814) = [];

Vel = cumtrapz(time,xAccel);

x = xAccel;

x = x-mean(xAccel);
p = find(diff(x) == 0);
x(p) = 0;
        
xVel = cumtrapz(double(time),x);
for k = 1:length(xVel)
    if xVel(k) < 0
        xVel(k) = 0;
    end
end

%GPS Velocity
eastDt = diff(movEast);
northDt = diff(movNorth);
gpsTimeDt = diff(gpsTime);

gpsVelocity = sqrt(eastDt.^2 + northDt.^2)./ gpsTimeDt;

gpsVelocity(1:endCircleInd) = [];
gpsVelocity = gpsVelocity-gpsVelocity(1);

gpsTime(1:endCircleInd) = [];
gpsTime = gpsTime - min(gpsTime);

gpsVelDt = diff(gpsVelocity);
restInd = find(gpsVelDt == 0);
restVel = mean(gpsVelocity(restInd));

gpsVelocity = gpsVelocity - restVel; % Subtract bias

%% Part 3 - Dead Reckoning
omega = zGyro .* xVel;
omega = unwrap(omega);
yAccel = yAccel - mean(yAccel);

ve = sin(yaw') .* xVel;
vn = cos(yaw') .* xVel;

xn = cumtrapz(time,vn);
xe = cumtrapz(time,ve);
%% Plots
figure
geoplot(movLat,movLon,'LineWidth', 1)
saveas(gcf,'Geoplot.jpg')

% Mag calibrated
figure
subplot(1,2,1)
scatter(D(:,1)-ellipse.X0_in,D(:,2)-ellipse.Y0_in)
hold on
% Richard Brown (2021). fitellipse.m (https://www.mathworks.com/matlabcentral/fileexchange/15125-fitellipse-m), MATLAB Central File Exchange. Retrieved March 12, 2021.
plotellipse([0.000001;0.000001], ellipse.a, ellipse.b, ellipse.phi)
title('Calibrated Mag')
xlabel('x Mag (gauss)','FontSize',12)
ylabel('y Mag (gauss)','FontSize',12)

subplot(1,2,2)
scatter(D(:,1),D(:,2))
title('Uncalibrated Mag')
xlabel('x Mag (gauss)','FontSize',12)
ylabel('y Mag (gauss)','FontSize',12)
axis equal
saveas(gcf,'calibration.jpg')

% Yaw plots
yawMag = yawMag - yawMag(1);
yawGyro = yawGyro - yawGyro(1);
figure
plot(time,yawMag,'LineWidth', 1)
hold on
plot(time,yaw,'LineWidth', 1)
hold on
plot(time,yawGyro,'LineWidth', 1)
legend('Mag Yaw','Sensor Yaw','Yaw Gyro')
title('Magnetometer vs Gyro')
xlabel('time (sec)','FontSize',12)
ylabel('yaw (radians)','FontSize',12)
saveas(gcf,'Comparison.jpg')

figure
plot(time,combinedFilt,'LineWidth', 1)
hold on
plot(time,yaw,'LineWidth', 1)
title('Complementary Filter of Yaw vs Measured Yaw')
xlabel('time (sec)','FontSize',12)
ylabel('yaw (radians)','FontSize',12)
legend('Combined Filter','Sensor Yaw')
saveas(gcf,'Filtered.jpg')

% Forward Velocity
figure
plot(time,Vel,'LineWidth', 1)
hold on
plot(gpsTime(1:end-1),gpsVelocity,'LineWidth', 1)
title('Integrated Forward Velocity')
xlabel('time (sec)','FontSize',12)
ylabel('Velocity (m/s)','FontSize',12)
legend('Integrated from xAccel','GPS Velocity','Location','southwest')
saveas(gcf,'Integrated Forward Velocity.jpg')

figure
plot(time,xVel,'LineWidth', 1)
hold on
plot(gpsTime(1:end-1),gpsVelocity,'LineWidth', 1)
title('Forward Velocity')
xlabel('time (sec)','FontSize',12)
ylabel('Velocity (m/s)','FontSize',12)
legend('Integrated from xAccel','GPS Velocity')
saveas(gcf,'Forward Velocity.jpg')

% Dead Reckoning
figure
plot(time,omega,'LineWidth', 1)
hold on
plot(time,yAccel,'LineWidth',1)
title('Y Acceleration Comparison')
xlabel('time (sec)','FontSize',12)
ylabel('Acceleration (m/s^2)','FontSize',12)
legend('omega * xDot','Y Acceleration')
saveas(gcf,'Y Acceleration Comparison.jpg')

figure
plot(xe,xn);
hold on
plot(movEast-min(movEast)-235.375,movNorth-min(movNorth)-18.5)
title('Dead Reckoning')
xlabel('Easting','FontSize',12)
ylabel('Northing','FontSize',12)
legend('Estimated Trajectory','GPS track')
saveas(gcf,'Dead Reckoning.jpg')