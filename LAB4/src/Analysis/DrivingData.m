clc
clear all
close all
format long;

file_path ="C:\Users\shahd\OneDrive\Documents\MATLAB\driving_data.bag"

bag = rosbag(file_path);
bagInfo = rosbag('info',file_path);
bSelgps = select(bag,'Topic','gps_data')
bSelimu = select(bag,'Topic','imu_data')
msgStructs1 = readMessages(bSelgps,'DataFormat','struct');
msgStructs1{1};
msgStructs2 = readMessages(bSelimu,'DataFormat','struct');
msgStructs1{1};
msgStructs2{1};
Yaw = cellfun(@(m) double(m.Yaw), msgStructs2);
Pitch = cellfun(@(m) double(m.Pitch), msgStructs2);
Roll = cellfun(@(m) double(m.Roll), msgStructs2);
AngularVelocity_x = cellfun(@(m) double(m.IMU.AngularVelocity.X), msgStructs2);
AngularVelocity_y = cellfun(@(m) double(m.IMU.AngularVelocity.Y), msgStructs2);
AngularVelocity_z = cellfun(@(m) double(m.IMU.AngularVelocity.Z), msgStructs2);
LinearAcceleration_x = cellfun(@(m) double(m.IMU.LinearAcceleration.X), msgStructs2);
LinearAcceleration_y = cellfun(@(m) double(m.IMU.LinearAcceleration.Y), msgStructs2);
LinearAcceleration_z = cellfun(@(m) double(m.IMU.LinearAcceleration.Z), msgStructs2);
MagX = cellfun(@(m) double(m.Magnetic.MagneticField_.X), msgStructs2);
MagY = cellfun(@(m) double(m.Magnetic.MagneticField_.Y), msgStructs2);
MagZ = cellfun(@(m) double(m.Magnetic.MagneticField_.Z), msgStructs2);
timePoints_index_imu = cellfun(@(m) int64(m.IMU.Header.Seq),msgStructs2);
timePoints_index_imu = timePoints_index_imu - min(timePoints_index_imu);
latitudePoints = cellfun(@(m) double(m.Latitude),msgStructs1);
longitudePoints = cellfun(@(m) double(m.Longitude),msgStructs1);
altitudePoints = cellfun(@(m) double(m.Altitude),msgStructs1);
NumberOfSatellites = cellfun(@(m) double(m.GpsNumbSat),msgStructs1);
HDOP = cellfun(@(m) double(m.Hdop),msgStructs1);
utmEastingPoints = cellfun(@(m) double(m.UtmEasting),msgStructs1);
utmNorthingPoints = cellfun(@(m) double(m.UtmNorthing),msgStructs1);
timePoints_index_gps = cellfun(@(m) int64(m.Header.Seq),msgStructs1);
timePoints_index_gps = timePoints_index_gps - min(timePoints_index_gps);

% 
% k = boundary(utmEastingPoints, utmNorthingPoints);
% hold on;
% plot(utmEastingPoints(k),utmNorthingPoints(k),'-')
% for kk = 1:length(utmEastingPoints)
% t = text(utmEastingPoints(kk),utmNorthingPoints(kk),num2str(kk));
% t.Color = [1 0 0];
% end
% 
% hold on

figure
plot(utmEastingPoints(112:180),utmNorthingPoints(112:180))
% 
% 
figure
plot(MagX(4400:7200),MagY(4400:7200))
grid on
axis equal
% 
% % % figure
% % % plot(MagX,MagY)
% % % axis equal
% 
% figure 
% plot(timePoints_index_imu(4400:7200), Yaw(4400:7200))
% 
% figure 
% plot(timePoints_index_imu(4400:7200), MagX(4400:7200))
% hold on
% plot(timePoints_index_imu(4400:7200), MagZ(4400:7200))
% hold off
% 
% figure 
% plot(MagX(4400:7200), MagY(4400:7200))
% axis equal
% grid on
% 
% 
figure
plot(utmEastingPoints(250:1071),utmNorthingPoints(250:1071))
axis equal
% 
% geoplot(latitudePoints(250:1071),longitudePoints(250:1071))
% 
% unique(NumberOfSatellites)
% 
% gravitation  = mean(sqrt((LinearAcceleration_x).^2 + (LinearAcceleration_y).^2 + (LinearAcceleration_z).^2))


Bank_angle = atan(mean(LinearAcceleration_y(10:4000))/(mean(LinearAcceleration_z(10:4000))));
Elevation_angle  = atan(mean(LinearAcceleration_x(10:4000))/(mean(LinearAcceleration_z(10:4000))));
%%
%%%%%%%%%%%%% Compensation for Elevation and bank angle %%%%%%%

R = [cos(Elevation_angle) 0 -sin(Elevation_angle);
    sin(Bank_angle)*sin(Elevation_angle) cos(Bank_angle) sin(Bank_angle)*cos(Elevation_angle);
    cos(Bank_angle)*sin(Elevation_angle) -sin(Bank_angle) cos(Bank_angle)*cos(Elevation_angle)]

for i = 1:length(MagX)
MagXYZ_tilt_correction = R*[MagX(i) ; MagY(i) ; MagZ(i)];
MagX_tilt_correction(i,1) = MagXYZ_tilt_correction(1);
MagY_tilt_correction(i,1) = MagXYZ_tilt_correction(2);
MagZ_tilt_correction(i,1) = MagXYZ_tilt_correction(3);
end

% figure 
% plot(MagX(4400:7200), MagZ(4400:7200))
% hold on
% 
% % figure 
% % plot(timePoints_index_imu(4400:7200), MagX_tilt_correction(4400:7200))
% % hold on
% % plot(timePoints_index_imu(4400:7200), MagZ_tilt_correction(4400:7200))
% % hold off
% % 
% % grid on 
% % axis equal
% plot(MagX_tilt_correction(4400:7200),MagY_tilt_correction(4400:7200))
%%
%%%%%%%%%%%%%%%%%%%%%% Compensating for Hard Iron Errors %%%%%%%%%%
%%% Index Starts at 4400 and ends at 7200 the time taken for 4 Calibration
%%% Rounds %%%%%%%%%%%%%%%%%%%%%%%%%%%%

alpha = (max(MagX_tilt_correction(4400:7200))+min(MagX_tilt_correction(4400:7200)))/2;
beta = (max(MagY_tilt_correction(4400:7200))+min(MagY_tilt_correction(4400:7200)))/2;
gama = (max(MagZ_tilt_correction(4400:7200))+min(MagZ_tilt_correction(4400:7200)))/2;

MagX_correctedHard = MagX_tilt_correction - alpha;
MagY_correctedHard = MagY_tilt_correction - beta;
MagZ_correctedHard = MagZ_tilt_correction - gama;
% 
% figure 
% plot(timePoints_index_imu(4400:7200), MagX_correctedHard(4400:7200))
% hold on
% plot(timePoints_index_imu(4400:7200), MagZ_correctedHard(4400:7200))
% hold off
% 
% figure
% plot(MagX_correctedHard(4400:7200),MagY_correctedHard(4400:7200))

%%
%%%%%%%%%%%%%%%% Compensating for Soft Iron Errors %%%%%%%%%%%%%%

rx = sqrt((MagX_correctedHard(4400:7200)).^2 + (MagY_correctedHard(4400:7200)).^2);
[M,Imu_velocity] = max(rx);
Thetax = asin(MagY_correctedHard(5069)/MagX_correctedHard(5069));
R_soft = [cos(Thetax) sin(Thetax);
      -sin(Thetax) cos(Thetax)]

for i = 1:length(MagX_correctedHard)
      
        MagXY_soft_correction = R_soft * [MagX_correctedHard(i); MagY_correctedHard(i)];
        MagX_soft_correction(i,1) = MagXY_soft_correction(1);
        MagY_soft_correction(i,1) = MagXY_soft_correction(2);
        ;
       

end
  

% figure 
% plot(timePoints_index_imu(4400:7200), MagX_soft_correction(4400:7200))
% hold on
% plot(timePoints_index_imu(4400:7200), MagZ_tilt_correction(4400:7200))
% hold off
% 
figure
hold on 
plot(MagX_soft_correction(4400:7200),MagY_soft_correction(4400:7200),"r")
plot(MagX_correctedHard(4400:7200),MagY_correctedHard(4400:7200),"b")
grid on
xlabel("Magnetometer Y (Gauss)")
ylabel("Magnetometer X (Gauss)")
plot(MagX(4400:7200),MagY(4400:7200),"g")
legend("Final","Hard Corrected","Original")
grid on 
hold off

%%
%%%%%%%%% Yaw From Magnetometer And Gyro %%%%%
Yaw = cellfun(@(m) double(m.Yaw), msgStructs2);
Yawd = deg2rad(Yaw);
Yawd = unwrap(Yawd);



yaw_from_mag_uncorrected = atan2(-MagY, MagX);
yaw_from_mag_uncorrected = unwrap(yaw_from_mag_uncorrected)
% plot(timePoints_index_imu,yaw_from_mag_uncorrected,"b")

yaw_from_mag = atan2(-MagY_soft_correction, MagX_soft_correction) ;
% yaw_from_mag = deg2rad(yaw_from_mag)
yaw_from_mag = unwrap(yaw_from_mag);
yaw_from_mag_scaled = yaw_from_mag - 1.8*Yawd(1);

figure
plot(timePoints_index_imu,yaw_from_mag_scaled,"g")
hold on
% plot(timePoints_index_imu, Yawd,"r")
plot(timePoints_index_imu,yaw_from_mag_uncorrected,"b")
grid on 
axis on 
xlabel("Time")
ylabel("Yaw(Radians)")
hold off 
legend("Yaw From Magnetometer Scaled and Corrected", "Yaw from Magnetometer Uncorrected")


AngularVelocity_z1 = AngularVelocity_z
imu_time = 0:.025:1072.275
yaw_from_gyro = cumtrapz(double(timePoints_index_imu), AngularVelocity_z1)/40
% yaw_from_gyrot = deg2rad(yaw_from_gyro)
% yaw_from_gyrot = unwrap(yaw_from_gyro)
yaw_from_gyro_scaled = yaw_from_gyro + 1* Yawd(1) 

figure
hold on
plot(timePoints_index_imu, yaw_from_gyro_scaled, "b")
plot(timePoints_index_imu,yaw_from_mag_scaled,"g")
grid on 
axis on 
xlabel("Time")
ylabel("Yaw(Radians)")
hold off 
legend("Yaw from GyroData Scaled","Yaw from Magnetometer Corrected and Scaled")
hold off
%%
%%%%%% Complimentary Filter %%%%%%%
% fs = 40
% hpf = 0.9999999997
% lpf = 1 - hpf
% 
% x = highpass(yaw_from_gyro,hpf)
% y = lowpass(yaw_from_mag_scaled,lpf)
% a = (x+y)
% plot(timePoints_index_imu(1:42800),a(1:42800),"b",timePoints_index_imu,Yawd,"r",timePoints_index_imu,yaw_from_mag_scaled,"y")
%%
% Filtering

tau = 0.025;
alpha = tau/(tau+0.025); %alpha=(tau)/(tau+dt)

yaw_from_mag_filtered(1,1) = yaw_from_mag_scaled(1,1)

for n = 2: length(yaw_from_mag_scaled)
    yaw_from_mag_filtered(n,1)=(1-alpha)*yaw_from_mag_scaled(n,1) + alpha*yaw_from_mag_filtered(n-1,1);     
end

% figure;
% hold on
% plot(timePoints_index_imu,yaw_from_mag_filtered,'yellow')
% hold on
% plot(timePoints_index_imu, yaw_from_mag_scaled,'b')
% legend

tau = 0.00000001;
alpha = tau/(tau+0.025);

yaw_from_gyro_filtered(1,1) = yaw_from_gyro_scaled(1,1);

for n = 2:length(yaw_from_gyro_scaled)
    yaw_from_gyro_filtered(n,1) = (1-alpha)*yaw_from_gyro_filtered(n-1,1) + (1-alpha)*(yaw_from_gyro_scaled(n,1) - yaw_from_gyro_scaled(n-1,1));
end

% figure;
% hold on
% plot(timePoints_index_imu,yaw_from_gyro_filtered,'DisplayName','Gyro heading filtered (integrated)','Color','r')
% hold on
% plot(timePoints_index_imu,yaw_from_gyro_scaled,'DisplayName','Gyro heading unfiltered (integrated)','Color','b')
% legend

alpha = 0.3;

final_yaw = alpha*yaw_from_mag_filtered + (1-alpha)*yaw_from_gyro_filtered;
% final_yaw = wrapToPi(final_yaw);

figure;
hold on
plot(timePoints_index_imu,yaw_from_mag_filtered,'DisplayName','Magnetometer heading filtered','Color','red')
hold on
plot(timePoints_index_imu,yaw_from_gyro_filtered,'DisplayName','Gyro heading filtered','Color','blue')
hold on
plot(timePoints_index_imu,final_yaw,'DisplayName','Final heading estimate','Color','green')
hold on
xlabel("Time")
ylabel("Yaw(Radians)")
plot(timePoints_index_imu, Yawd,'DisplayName','Yaw from IMU','Color','black')
legend
%%
%%%%%%%%%%%% Velocity from Acceleration


Imu_Velocity(10401,1) = 0;
for i = 10401:length(timePoints_index_imu);
    Imu_Velocity(i,1) = Imu_Velocity(i-1,1) + LinearAcceleration_x(i,1)/400;
end



%%%%%%% Velocity from GPS


dt = 1

for i = 2:length(utmEastingPoints)
    delta_easting = (utmEastingPoints(i) - utmEastingPoints(i-1));
    delta_northing = (utmNorthingPoints(i)- utmNorthingPoints(i-1));
    delta (i) = sqrt(delta_easting^2 + delta_northing^2);
%     vel(i) = delta(i)/dt;
%     velocity_gps(i) = vel(i);
end

for i = 1:length(utmNorthingPoints)
    for j = 1:40
        velocity_gps(40*(i-1) + j) = delta(i);
    end
end


figure
plot(imu_time, Imu_Velocity, 'DisplayName','IMU Velocity Estimate',"color","Red")
hold on
plot(timePoints_index_imu(1:42840),velocity_gps,'DisplayName','GPS Velocity estimate','Color','blue')
legend
xlabel("Time")
ylabel("Velocity(m/s)")
hold on
% plot(timePoints_index_imu,velocity_gps_final)

LinearAcceleration_x_adjusted = LinearAcceleration_x;
LinearAcceleration_x_adjusted = LinearAcceleration_x_adjusted - 0.60009;

time_window = 5;
time_window = time_window * 40;
LinearAcceleration_x_filtered = LinearAcceleration_x_adjusted;

for i = 0:length(LinearAcceleration_x_filtered)-time_window
    set_acc_zero = 1;
    for j = 1:time_window
        if abs(LinearAcceleration_x_filtered(i+j,1)) <= 0.5
            set_acc_zero = set_acc_zero*1; %  Yes
        else
            set_acc_zero = set_acc_zero*0; %  No
        end
    end
    if set_acc_zero == 1
        for j = 1:time_window
            LinearAcceleration_x_filtered(i+j,1) = 0;
        end
    end
end

velocity_imu_new(7700,1) = 0; %When circle ended

for i = 7700+1:length(timePoints_index_imu)
    velocity_imu_new(i,1) = velocity_imu_new(i-1,1) + LinearAcceleration_x_filtered(i,1)/40;
end


figure;

plot(timePoints_index_imu,velocity_imu_new,'DisplayName','IMU Velocity','Color','red')
xlabel('Time')
ylabel('Velocity (m/s)')
hold on
plot(timePoints_index_imu(1:42840),velocity_gps,'DisplayName','GPS Velocity estimate','Color','blue')
legend

hold on
% plot(timePoints_index_imu,LinearAcceleration_x_filtered(1:42892),'DisplayName','Acc x filtered','Color','black')
grid on
%%
% omega x dot 

omega_x_dot = AngularVelocity_z.*velocity_imu_new


figure;
plot(timePoints_index_imu,omega_x_dot,"o",'DisplayName','Omega x dot','Color','blue')
xlabel('Time')
ylabel('Acceleration (m/s^2)')
grid on
legend

hold on
plot(timePoints_index_imu,LinearAcceleration_y,'DisplayName','Y double dot','Color',"yellow")



%% Dead Recknoning

%Plotting GPS data/ displacement

utmEastingPoints = utmEastingPoints - utmEastingPoints(1);
utmNorthingPoints = utmNorthingPoints - utmNorthingPoints(1);

% for kk = 1:length(utmEastingPoints)
% t = text(utmEastingPoints(kk),utmNorthingPoints(kk),num2str(kk));
% t.Color = [1 0 0];
% end
% 
% k = boundary(utmEastingPoints, utmNorthingPoints);
% hold on;
% plot(utmEastingPoints(k),utmNorthingPoints(k),'-')


figure;
plot(utmEastingPoints,utmNorthingPoints,'DisplayName','GPS Plot','Color','green')
title('UTM Easting vs UTM Northing')
xlabel('UTM Easting (m)')
ylabel('UTM Northing (m)')
grid on
legend

%Calculating distance from IMU velocity
t = 0:0.025:42891/40;
distance_from_imu = cumtrapz(t,velocity_imu_new);

imu_x(1,1) = 0;
imu_y(1,1) = 0;
correction_angle = 143*pi/180;
heading_mag1 = yaw_from_mag_filtered + correction_angle;

for i = 2:length(yaw_from_mag_filtered) 
      imu_x(i,1) = imu_x(i-1,1) + (distance_from_imu(i) - distance_from_imu(i-1))*cos(unwrap(heading_mag1(i)));
      imu_y(i,1) = imu_y(i-1,1) + (distance_from_imu(i) - distance_from_imu(i-1))*-sin(unwrap(heading_mag1(i))); 
end 

imu_x = imu_x - imu_x(1);
imu_y = imu_y - imu_y(1);

imu__x_scaled = imu_x*(0.8)
imu_y_scaled = imu_y*(0.8)
% utmE_from_imu =sin(heading_mag_filtered).*distance_from_imu(circle_ei:length(MagX));
% utmN_from_imu =cos(heading_mag_filtered).*distance_from_imu(circle_ei:length(MagX));

hold on
plot(imu__x_scaled,imu_y_scaled,'DisplayName','Position Plot from IMU','Color','red')

%%

omega_dot(1,1) = 0;
for i = 2:length(AngularVelocity_z)
    omega_dot(i,1) = (AngularVelocity_z(i) - AngularVelocity_z(i-1))*40;
end


xc = (LinearAcceleration_y - omega_x_dot)./omega_dot;
% From velocity plot observation, car was stationary from time point 22000
% to 23000 i.e. from time 550 to 575, mean of abs(xc) is calculated which
% turns out to be -0.165962681533252 or 16.59 cm

mean(xc(22000:23000))


figure;
plot(timePoints_index_imu,AngularVelocity_z,'.','DisplayName','Omega z','Color','green')
%title('Omega x dot and Y double dot vs time','Interpreter','latex')
xlabel('Time')
ylabel('omega and omega dot(derivative)')
grid on
legend

hold on
plot(timePoints_index_imu,omega_dot,'.','DisplayName','omega dot (derivative)','Color','red')

hold on
plot(timePoints_index_imu,xc,'.','DisplayName','xc','Color','yellow')





