clc
clear all
close all
format long;

file_path = "C:\Users\shahd\OneDrive\Documents\MATLAB\5_hrs_data_2022-02-26-14-14-35.bag"

bag = rosbag(file_path);
bagInfo = rosbag('info',file_path);
bSel = select(bag,'Topic','imu_data_lab3')
msgStructs = readMessages(bSel,'DataFormat','struct');
msgStructs{1};
% Orientation_X = cellfun(@(m) double(m.Imu.Orientation.X), msgStructs);
% Orientation_Y = cellfun(@(m) double(m.Imu.Orientation.Y), msgStructs);
% Orientation_Z = cellfun(@(m) double(m.Imu.Orientation.X), msgStructs);
AngularVelocity_x = cellfun(@(m) double(m.Imu.AngularVelocity.X), msgStructs);
AngularVelocity_y = cellfun(@(m) double(m.Imu.AngularVelocity.Y), msgStructs);
AngularVelocity_z = cellfun(@(m) double(m.Imu.AngularVelocity.Z), msgStructs);
LinearAcceleration_x = cellfun(@(m) double(m.Imu.LinearAcceleration.X), msgStructs);
LinearAcceleration_y = cellfun(@(m) double(m.Imu.LinearAcceleration.Y), msgStructs);
LinearAcceleration_z = cellfun(@(m) double(m.Imu.LinearAcceleration.Z), msgStructs);
timePoints_index = cellfun(@(m) int64(m.Imu.Header.Seq),msgStructs);
timePoints_index = timePoints_index - min(timePoints_index);


t0 = 1/40;
thetax = cumsum(AngularVelocity_z, 1)*t0;

maxNumMx = 100;
Lx = size(thetax, 1);
maxMx= 2.^floor(log2(Lx/2)); 
mx = logspace(log10(1), log10(maxMx), maxNumMx).';
mx = ceil(mx); % m must be an integer.
mx = unique(mx); % Remove duplicates.

taux = mx*t0;

avarx = zeros(numel(mx), 1);
for i = 1:numel(mx)
    mi = mx(i);
    avarx(i,:) = sum( ...
        (thetax(1+2*mi:Lx) - 2*thetax(1+mi:Lx-mi) + thetax(1:Lx-2*mi)).^2, 1);
end
avarx = avarx ./ (2*taux.^2 .* (Lx - 2*mx));

adevx = sqrt(avarx);

figure
loglog(taux, adevx)
title('Allan Deviation')
xlabel('\tau');
ylabel('\sigma(\tau)')
grid on


% Find the index where the slope of the log-scaled Allan deviation is equal
% to the slope specified.
slope = -0.5;
logtau = log10(taux);
logadev = log10(adevx);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the angle random walk coefficient from the line.
logN = slope*log(1) + b;
N = 10^logN

% Plot the results.
tauN = 1;
lineN = N ./ sqrt(taux);
figure
loglog(taux, adevx, taux, lineN, '--', tauN, N, 'o')
title('Allan Deviation with Angle Random Walk')
xlabel('\tau')
ylabel('\sigma(\tau)')
legend('\sigma', '\sigma_N')
text(tauN, N, 'N')
grid on
axis equal

slope = 0.5;
logtau = log10(taux);
logadev = log10(adevx);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the rate random walk coefficient from the line.
logK = slope*log10(3) + b;
K = 10^logK

% Plot the results.
tauK = 3;
lineK = K .* sqrt(taux/3);
figure
loglog(taux, adevx, taux, lineK, '--', tauK, K, 'o')
title('Allan Deviation with Rate Random Walk')
xlabel('\tau')
ylabel('\sigma(\tau)')
legend('\sigma', '\sigma_K')
text(tauK, K, 'K')
grid on
axis equal


slope = 0;
logtau = log10(taux);
logadev = log10(adevx);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the bias instability coefficient from the line.
scfB = sqrt(2*log(2)/pi);
logB = b - log10(scfB);
B = 10^logB

% Plot the results.
tauB = taux(i);
lineB = B * scfB * ones(size(taux));
figure
loglog(taux, adevx, taux, lineB, '--', tauB, scfB*B, 'o')
title('Allan Deviation with Bias Instability')
xlabel('\tau')
ylabel('\sigma(\tau)')
legend('\sigma', '\sigma_B')
text(tauB, scfB*B, '0.664B')
grid on
axis equal

%
% Now that all the noise parameters have been calculated, plot the Allan
% deviation with all of the lines used for quantifying the parameters.
tauParams = [tauN, tauK, tauB];
params = [N, K, scfB*B];
figure
loglog(taux, adevx, taux, [lineN, lineK, lineB], '--', ...
    tauParams, params, 'o')
title('Allan Deviation with Noise Parameters')
xlabel('\tau')
ylabel('\sigma(\tau)')
legend('$\sigma (rad/s)$', '$\sigma_N ((rad/s)/\sqrt{Hz})$', ...
    '$\sigma_K ((rad/s)\sqrt{Hz})$', '$\sigma_B (rad/s)$', 'Interpreter', 'latex')
text(tauParams, params, {'N', 'K', '0.664B'})
grid on
axis equal

generateSimulatedData = false;

if generateSimulatedData
    % Set the gyroscope parameters to the noise parameters determined 
    % above. 
    gyro = gyroparams('NoiseDensity', N, 'RandomWalk', K, ...
        'BiasInstability', B);
    omegaSim = helperAllanVarianceExample(Lx, 40, gyro);
else
    load('SimulatedSingleAxisGyroscope', 'omegaSim')
end


[avarSim, tauSim] = allanvar(omegaSim, 'octave', 40);
adevSim = sqrt(avarSim);
adevSim = mean(adevSim, 2); % Use the mean of the simulations.

figure
loglog(taux, adevx, tauSim, adevSim, '--')
title('Allan Deviation of HW and Simulation')
xlabel('\tau');
ylabel('\sigma(\tau)')
legend('HW', 'SIM')
grid on
axis equal