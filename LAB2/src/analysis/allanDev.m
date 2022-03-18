% Sameer Bhatti
% bhatti.sa@northeastern.edu
% 2/25/2021
% imuAnalysis.m
%
% Plots imu data
clc
clear
close all

imuBag = rosbag('imuStationary.bag');

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

time = imuTime;
time(1:814) = [];
time = time-min(time);

xAccel(1:814) = [];

%% Allen Deviation
Fs = 40;
t0 = 1/Fs;
omega = xAccel';
theta = cumsum(omega, 1)*t0;

maxNumM = 100;
L = size(theta, 1);
maxM = 2.^floor(log2(L/2));
m = logspace(log10(1), log10(maxM), maxNumM).';
m = ceil(m); % m must be an integer.
m = unique(m); % Remove duplicates.

tau = m*t0;

avar = zeros(numel(m), 1);
for i = 1:numel(m)
    mi = m(i);
    avar(i,:) = sum((theta(1+2*mi:L) - 2*theta(1+mi:L-mi) + theta(1:L-2*mi)).^2, 1);
end
avar = avar ./ (2*tau.^2 .* (L - 2*m));

adev = sqrt(avar);

figure
loglog(tau, adev)
grid on
axis equal
hold on

% Find the index where the slope of the log-scaled Allan deviation is equal
% to the slope specified.
slope = -0.5;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the angle random walk coefficient from the line.
logN = slope*log(1) + b;
N = 10^logN

% Plot the results.
tauN = 1;
lineN = N ./ sqrt(tau);
figure
loglog(tau, adev, tau, lineN, '--', tauN, N, 'o')
title('Allan Deviation with Angle Random Walk')
xlabel('\tau')
ylabel('\sigma(\tau)')
legend('\sigma', '\sigma_N')
text(tauN, N, 'N')
grid on
axis equal
hold on

% Find the index where the slope of the log-scaled Allan deviation is equal
% to the slope specified.
slope = 0.5;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the rate random walk coefficient from the line.
logK = slope*log10(3) + b;
K = 10^logK

% Plot the results.
tauK = 3;
lineK = K .* sqrt(tau/3);
figure
loglog(tau, adev, tau, lineK, '--', tauK, K, 'o')
title('Allan Deviation with Rate Random Walk')
xlabel('\tau')
ylabel('\sigma(\tau)')
legend('\sigma', '\sigma_K')
text(tauK, K, 'K')
grid on
axis equal

hold off
% Find the index where the slope of the log-scaled Allan deviation is equal
% to the slope specified.
slope = 0;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the bias instability coefficient from the line.
scfB = sqrt(2*log(2)/pi);
logB = b - log10(scfB);
B = 10^logB

% Plot the results.
tauB = tau(i);
lineB = B * scfB * ones(size(tau));
figure
loglog(tau, adev, tau, lineB, '--', tauB, scfB*B, 'o')
title('Allan Deviation with Bias Instability')
xlabel('\tau')
ylabel('\sigma(\tau)')
legend('\sigma', '\sigma_B')
text(tauB, scfB*B, '0.664B')
grid on
axis equal

tauParams = [tauN, tauK, tauB];
params = [N, K, scfB*B];
figure
loglog(tau, adev, tau, [lineN, lineK, lineB], '--', ...
    tauParams, params, 'o')
title('Allan Deviation with Noise Parameters - x Accel')
xlabel('\tau')
ylabel('\sigma(\tau)')
legend('\sigma', '\sigma_N', '\sigma_K', '\sigma_B')
text(tauParams, params, {'N', 'K', '0.664B'})
grid on
axis equal