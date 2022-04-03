clc
clear all
close all
format long;

file_path = "C:\Users\shahd\OneDrive\Documents\MATLAB\15min.bag"

bag = rosbag(file_path);
bagInfo = rosbag('info',file_path);
bSel = select(bag,'Topic','IMUTopic')
msgStructs = readMessages(bSel,'DataFormat','struct');
msgStructs{1};
Orientation_X = cellfun(@(m) double(m.Imudata.Orientation.X), msgStructs);
Orientation_Y = cellfun(@(m) double(m.Imudata.Orientation.Y), msgStructs);
Orientation_Z = cellfun(@(m) double(m.Imudata.Orientation.Z), msgStructs);
LinearAcceleration_x = cellfun(@(m) double(m.Imudata.LinearAcceleration.X), msgStructs);
LinearAcceleration_y = cellfun(@(m) double(m.Imudata.LinearAcceleration.Y), msgStructs);
LinearAcceleration_z = cellfun(@(m) double(m.Imudata.LinearAcceleration.Z), msgStructs);
AngularVelocity_x = cellfun(@(m) double(m.Imudata.AngularVelocity.X), msgStructs);
AngularVelocity_y = cellfun(@(m) double(m.Imudata.AngularVelocity.Y), msgStructs);
AngularVelocity_z = cellfun(@(m) double(m.Imudata.AngularVelocity.Z), msgStructs);
Magx= cellfun(@(m) double(m.Mag.MagneticField_.X), msgStructs);
Magy= cellfun(@(m) double(m.Mag.MagneticField_.Y), msgStructs);
Magz= cellfun(@(m) double(m.Mag.MagneticField_.Z), msgStructs);
timePoints_index = cellfun(@(m) int64(m.Imudata.Header.Seq),msgStructs);
timePoints_index = timePoints_index - min(timePoints_index);

Lxmean = mean(LinearAcceleration_x)
lxstd = std(LinearAcceleration_x)
lx = normpdf(LinearAcceleration_x,Lxmean,lxstd);

figure
hold on
plot(timePoints_index, LinearAcceleration_x)
title('Linear Acceleration in X vs Time')
xlabel('Time');
ylabel('Linear Acceleration X')
grid on
hold off

figure
plot(LinearAcceleration_x, lx)
title('Linear Acceleration Gaussian Noise Plot')
xlabel('Linear Acceleration X');
ylabel('')
grid on

Lymean = mean(LinearAcceleration_y)
lystd = std(LinearAcceleration_y)
ly= normpdf(LinearAcceleration_y,Lymean,lystd);

figure
hold on
plot(timePoints_index, LinearAcceleration_y)
title('Linear Acceleration in Y vs Time')
xlabel('Time');
ylabel('Linear Acceleration Y')
grid on
hold off

figure
plot(LinearAcceleration_y, ly)
title('Linear Acceleration Gaussian Noise Plot')
xlabel('Linear Acceleration Y');
ylabel('')
grid on

Lzmean = mean(LinearAcceleration_z)
lzstd = std(LinearAcceleration_z)
lz= normpdf(LinearAcceleration_z,Lzmean,lzstd);

figure
hold on
plot(timePoints_index, LinearAcceleration_y)
title('Linear Acceleration in Z vs Time')
xlabel('Time');
ylabel('Linear Acceleration Z')
grid on
hold off

figure
plot(LinearAcceleration_z, lz)
title('Linear Acceleration Gaussian Noise Plot')
xlabel('Linear Acceleration Y');
ylabel('')
grid on


AXmean = mean(AngularVelocity_x)
AXstd = std(AngularVelocity_x)
Ax = normpdf(AngularVelocity_x,AXmean,AXstd);

figure
hold on
plot(timePoints_index, AngularVelocity_x)
title('Angular Velocity X vs Time')
xlabel('Time');
ylabel('Angular Velocity X')
grid on
hold off

figure
plot(AngularVelocity_x, Ax)
title('Angular Velocity Gaussian Noise')
xlabel('AngularVelocity X');
ylabel("")
grid on


AYmean = mean(AngularVelocity_y)
AYstd = std(AngularVelocity_y)
Ay = normpdf(AngularVelocity_y,AYmean,AYstd);

figure
plot(timePoints_index, AngularVelocity_y)
title('AngularVelocity Y vs Time')
xlabel('Time');
ylabel('AngularVelocity Y')
grid on 

figure
plot(AngularVelocity_y, Ay)
title('Angular Velocity Gaussian Noise')
xlabel('AngularVelocity Y');
ylabel("")
grid on

AZmean = mean(AngularVelocity_z)
AZstd = std(AngularVelocity_z)
AZ = normpdf(AngularVelocity_z,AZmean,AZstd);


figure
plot(timePoints_index, AngularVelocity_z)
title('Angular Velocity Z vs Time')
xlabel('Time');
ylabel('Angular Velocity Z')
grid on

figure
plot(AngularVelocity_z, AZ)
title('Angular Velocity Gaussian Noise')
xlabel('AngularVelocity Z');
ylabel("")
grid on

figure 
plot3(AngularVelocity_x,AngularVelocity_y,AngularVelocity_z)
title('Angular Velocity Point Cloud')
xlabel('AngularVelocity X');
ylabel('AngularVelocity Y')
zlabel("AngularVelocity Z")
axis equal 
grid on

Magxmean = mean(Magx)
Magxstd = std(Magx)
MagX = normpdf(Magx,Magxmean,Magxstd);


figure
plot(timePoints_index, Magx)
title('Magnetic Field vs Time')
xlabel('Time');
ylabel('Magnetic Field X')
grid on

figure
plot(Magx, MagX)
title('Magnetic Field Gaussian Noise')
xlabel('Magnetic Field X');
ylabel("")
grid on

Magymean = mean(Magy)
Magystd = std(Magy)
MagY = normpdf(Magy,Magymean,Magystd);


figure
plot(timePoints_index, Magy)
title('Magnetic Field vs Time')
xlabel('Time');
ylabel('Magnetic Field Y')
grid on

figure
plot(Magy, MagY)
title('Magnetic Field Gaussian Noise')
xlabel('Magnetic Field Y');
ylabel("")
grid on

Magzmean = mean(Magz)
Magzstd = std(Magz)
MagZ = normpdf(Magz,Magzmean,Magzstd);


figure
plot(timePoints_index, Magz)
title('Magnetic Field vs Time')
xlabel('Time');
ylabel('Magnetic Field Z')
grid on

figure
plot(Magz, MagZ)
title('Magnetic Field Gaussian Noise')
xlabel('Magnetic Field Z');
ylabel("")
grid on

figure 
plot3(Magx, Magy, Magz)
title('MAgnetic Field Point Cloud')
xlabel('Magnetic Field X');
ylabel('Magnetic Field Y')
zlabel("Magnetic Field Z")
axis equal 
grid on

%%
% Allan Variance
% t0 = 1/40;
% thetax = cumsum(AngularVelocity_x, 1)*t0;
% 
% maxNumMx = 100;
% Lx = size(thetax, 1);
% maxMx= 2.^floor(log2(Lx/2)); 
% mx = logspace(log10(1), log10(maxMx), maxNumMx).';
% mx = ceil(mx); % m must be an integer.
% mx = unique(mx); % Remove duplicates.
% 
% taux = mx*t0;
% 
% avarx = zeros(numel(mx), 1);
% for i = 1:numel(mx)
%     mi = mx(i);
%     avarx(i,:) = sum( ...
%         (thetax(1+2*mi:Lx) - 2*thetax(1+mi:Lx-mi) + thetax(1:Lx-2*mi)).^2, 1);
% end
% avarx = avarx ./ (2*taux.^2 .* (Lx - 2*mx));
% 
% adevx = sqrt(avarx);
% 
% t0 = 1/40;
% thetay = cumsum(AngularVelocity_y, 1)*t0;
% 
% maxNumMy = 100;
% Ly = size(thetay, 1);
% maxMy= 2.^floor(log2(Ly/2)); 
% my = logspace(log10(1), log10(maxMy), maxNumMy).';
% my = ceil(my); % m must be an integer.
% my = unique(my); % Remove duplicates.
% 
% tauy = my*t0;
% 
% avary = zeros(numel(my), 1);
% for i = 1:numel(my)
%     mi = my(i);
%     avary(i,:) = sum( ...
%         (thetay(1+2*mi:Ly) - 2*thetay(1+mi:Ly-mi) + thetay(1:Ly-2*mi)).^2, 1);
% end
% avary = avary ./ (2*taux.^2 .* (Ly - 2*my));
% 
% adevy = sqrt(avary);
% 
% t0 = 1/40;
% thetaz = cumsum(AngularVelocity_z, 1)*t0;
% 
% maxNumMz = 100;
% Lz = size(thetaz, 1);
% maxMz= 2.^floor(log2(Lz/2)); 
% mz = logspace(log10(1), log10(maxMz), maxNumMz).';
% mz = ceil(mz); % m must be an integer.
% mz = unique(mz); % Remove duplicates.
% 
% tauz = mz*t0;
% 
% avarz = zeros(numel(mz), 1);
% for i = 1:numel(mz)
%     mi = mz(i);
%     avarz(i,:) = sum( ...
%         (thetaz(1+2*mi:Lz) - 2*thetaz(1+mi:Lz-mi) + thetaz(1:Lz-2*mi)).^2, 1);
% end
% avarz= avarz ./ (2*tauz.^2 .* (Lz - 2*mz));
% 
% adevz = sqrt(avarz);
% 
% figure
% loglog(taux, adevx)
% title('Allan Deviation')
% xlabel('\tau');
% ylabel('\sigma(\tau)')
% grid on
% 
% hold on
% loglog(tauy, adevy)
% title('Allan Deviation')
% xlabel('\tau');
% ylabel('\sigma(\tau)')
% grid on
% 
% hold on
% loglog(tauz, adevz)
% title('Allan Deviation')
% xlabel('\tau');
% ylabel('\sigma(\tau)')
% grid on
% legend("x","y","z")
% axis equal
% hold off
% 
% % Find the index where the slope of the log-scaled Allan deviation is equal
% % to the slope specified.
% slope = -0.5;
% logtau = log10(taux);
% logadev = log10(adevx);
% dlogadev = diff(logadev) ./ diff(logtau);
% [~, i] = min(abs(dlogadev - slope));
% 
% % Find the y-intercept of the line.
% b = logadev(i) - slope*logtau(i);
% 
% % Determine the angle random walk coefficient from the line.
% logN = slope*log(1) + b;
% N = 10^logN
% 
% % Plot the results.
% tauN = 1;
% lineN = N ./ sqrt(taux);
% figure
% loglog(taux, adevx, taux, lineN, '--', tauN, N, 'o')
% title('Allan Deviation with Angle Random Walk')
% xlabel('\tau')
% ylabel('\sigma(\tau)')
% legend('\sigma', '\sigma_N')
% text(tauN, N, 'N')
% grid on
% axis equal
% 
% slope = 0.5;
% logtau = log10(taux);
% logadev = log10(adevx);
% dlogadev = diff(logadev) ./ diff(logtau);
% [~, i] = min(abs(dlogadev - slope));
% 
% % Find the y-intercept of the line.
% b = logadev(i) - slope*logtau(i);
% 
% % Determine the rate random walk coefficient from the line.
% logK = slope*log10(3) + b;
% K = 10^logK
% 
% % Plot the results.
% tauK = 3;
% lineK = K .* sqrt(taux/3);
% figure
% loglog(taux, adevx, taux, lineK, '--', tauK, K, 'o')
% title('Allan Deviation with Rate Random Walk')
% xlabel('\tau')
% ylabel('\sigma(\tau)')
% legend('\sigma', '\sigma_K')
% text(tauK, K, 'K')
% grid on
% axis equal
% 
% 
% slope = 0;
% logtau = log10(taux);
% logadev = log10(adevx);
% dlogadev = diff(logadev) ./ diff(logtau);
% [~, i] = min(abs(dlogadev - slope));
% 
% % Find the y-intercept of the line.
% b = logadev(i) - slope*logtau(i);
% 
% % Determine the bias instability coefficient from the line.
% scfB = sqrt(2*log(2)/pi);
% logB = b - log10(scfB);
% B = 10^logB
% 
% % Plot the results.
% tauB = taux(i);
% lineB = B * scfB * ones(size(taux));
% figure
% loglog(taux, adevx, taux, lineB, '--', tauB, scfB*B, 'o')
% title('Allan Deviation with Bias Instability')
% xlabel('\tau')
% ylabel('\sigma(\tau)')
% legend('\sigma', '\sigma_B')
% text(tauB, scfB*B, '0.664B')
% grid on
% axis equal
% 
% %% 
% %
% % Now that all the noise parameters have been calculated, plot the Allan
% % deviation with all of the lines used for quantifying the parameters.
% tauParams = [tauN, tauK, tauB];
% params = [N, K, scfB*B];
% figure
% loglog(taux, adevx, taux, [lineN, lineK, lineB], '--', ...
%     tauParams, params, 'o')
% title('Allan Deviation with Noise Parameters')
% xlabel('\tau')
% ylabel('\sigma(\tau)')
% legend('$\sigma (rad/s)$', '$\sigma_N ((rad/s)/\sqrt{Hz})$', ...
%     '$\sigma_K ((rad/s)\sqrt{Hz})$', '$\sigma_B (rad/s)$', 'Interpreter', 'latex')
% text(tauParams, params, {'N', 'K', '0.664B'})
% grid on
% axis equal