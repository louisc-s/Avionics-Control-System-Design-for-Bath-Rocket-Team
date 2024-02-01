
% load propulsion sub-system's simulation data
load('output.mat')
y = h.z(3,:);
a = diff(h.z(4,:)) / h.dt;
t = h.t;

%apply white Gaussian noise to data 
u_altimeter = awgn(y,30, 'measured');
u_accelerometer = awgn(a,10, 'measured');

%run the filter functions
z_1D = kalman_1D(u_altimeter);
z_1D = z_1D(1:end-1);
a_1D = kalman_1D(u_accelerometer);
a_1D = a_1D(1:end-1);
z_3D = kalman(u_altimeter, u_accelerometer,h.dt);

%run the code 25 times to find the mean error for each filter
for i = 1:25
    z_1D = kalman_1D(u_altimeter);
    z_1D = z_1D(1:end-1);
    a_1D = kalman_1D(u_accelerometer);
    a_1D = a_1D(1:end-1);
    z_3D = kalman(u_altimeter, u_accelerometer,h.dt);
    error_z1D(i) = mean(abs(z_1D-y));
    error_a1D(i) = mean(abs(a_1D-a));
    error_z3D(i) = mean(abs(z_3D(1,:)-y));
    error_a3D(i) = mean(abs(z_3D(3,2:end)-a));
end
averror_z1D = mean(error_z1D);
averror_a1D = mean(error_a1D);
averror_a3D = mean(error_z3D);
averror_z3D = mean(error_a3D);

%plot filter results 
tiledlayout(2,4)
ax1 = nexttile;
plot(t,z_1D)
ax2 = nexttile;
plot(t(1:end-1),a_1D)
ax3 = nexttile;
plot(t,z_3D(1,:))
ax4 = nexttile;
plot(t,z_3D(3,:))
ax5 = nexttile;
error_z1D = z_1D-y;
plot(t,error_z1D)
ax6 = nexttile;
error_a1D = a_1D-a;
plot(t(2:end),error_a1D)
ax7 = nexttile;
error_z3D = z_3D(1,:)-y;
plot(t,error_z3D)
ax8 = nexttile;
error_a3D = z_3D(3,2:end)-a;
plot(t(1:end-1),error_a3D)

title(ax1,'Kalman filtering using only altimeter')
title(ax2,'Kalman filtering only accelerometer')
title(ax3,'Kalman filtering using two sensors - altitude')
title(ax4,'Kalman filtering using two sensors - acceleration')
title(ax5,'Error using only alimeter')
title(ax6,'Error using only accelerometer')
title(ax7,'Error using two sensors - altitude')
title(ax8,'Error using two sensors - acceleration')

%intial Kalman filter that uses only type of sensor data  

function x_k = kalman_1D(u_altimeter);

%define initial variables and constants 
P_k = 0;
Q = 1e-1;
R = 1e-1;
x_k = zeros(1,length(u_altimeter));

%loop the Kalman filter algorithm
for i = 1:length(x_k)
    x_k(i+1) = x_k(i);
    P_k = P_k+ Q;
    K_k = P_k/(P_k+ R);
    x_k(i+1) = x_k(i+1) + K_k * (u_altimeter(i) - x_k(i+1));
    P_k = (1 - K_k)*P_k;
end

end

%developed Kalman filter that fuses altimeter and acclerometer data

function x = kalman(u_altimeter,u_accelerometer,dt);

% define noise standard deviation values
sigma_m = 1e-2; 
sigma_A = 1e-2;
sigma_I = 1e-2;

%define initial variables and constants 
P = zeros(3,3);
Q = [0,0,0;0,0,0;0,0,sigma_m^2];
R = [sigma_A^2,0,;0,sigma_I^2];
A = [1, dt, dt^2/2; 0, 1, dt; 0, 0, 1];
C = [1,0,0;0,0,1];
x = zeros(3,length(u_altimeter));

%loop the Kalman filter algorithm
for i = 1:length(x)-1
    x(:,i+1) = A*x(:,i);
    P = A*P*A'+ Q;
    K = P*C'/(C*P*C'+ R);
    x(:,i+1) = x(:,i+1) + K * ([u_altimeter(i);u_accelerometer(i)] - C*x(:,i+1));
    P = (eye(3) - K*C)*P;
    
end
end

