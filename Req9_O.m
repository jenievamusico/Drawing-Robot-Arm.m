clc;
clear;

% This code prints the angles, angular velocities, and angular
% accelerations of theta1 and theta2 over time for the trajectory of the
% letter O

% Define robot variables

L1 = 0.3;
L2 = 0.3;

% define waypoints for O trajectory
t = (0:0.2:10)'; % Time
count = length(t);
center = [0.2 0.2 0];
radius = 0.15;
theta = t*(2*pi/t(end));

wpts = center + radius*[cos(theta) sin(theta) zeros(size(theta))];

% use trapezoidal trajectory planning to get position, velocity, and
% acceleration of end effector
[q,qd,qdd,t] = trapveltraj(wpts,100,EndTime=1);

% define each dimension specifically
x = q(1,:)';
y = q(2,:)';
z = q(3,:)';
dx = qd(1,:)';
dy = qd(2,:)';
dz = qd(3,:)';
ddx = qdd(1,:)';
ddy = qdd(2,:)';
ddz = qdd(3,:)';
theta1 = [1,100];
theta2 = [1,100];
dtheta1 = [1,100];
dtheta2 = [1,100];
ddtheta1 = [1,100];
ddtheta2 = [1,100];

for i = 1:length(q)
    % Calculate the joint angles using inverse kinematics
    theta2(i) = acos(((x(i,1)*x(i,1))+(y(i,1)*y(i,1))-(L1*L1)-(L2*L2))/(2*L1*L2));
    theta1(i) = atan((y(i,1)*(L2*cos(theta2)+L1)-(x(i,1)*L2*sin(theta2)))/(x(i,1)*(L2*cos(theta2)+L1)+(y(i,1)*L2*sin(theta2))));

    % Calculate the joint velocities and accelerations
    dtheta1(i) = ((acos(theta1+theta2))/(L1*sin(theta2))*dx(i,1))+(((sin(theta1+theta2))/(L2*sin(theta2)))*dy(i,1));
    dtheta2(i) = ((((-L1*cos(theta1))-(L2*cos(theta1+theta2)))/(L1*L2*sin(theta2)))*dx(i,1))+((((-L1*sin(theta1))-(L2*sin(theta1+theta2)))/(L1*L2*sin(theta2)))*dy(i,1));



    
    % the following calculations for the joint acceleration were
    % unsuccessful

    % ddtheta1(i) = (((-sin(theta1+theta2)*(theta1+theta2))*dx(i,1)+(cos(theta1+theta2))*ddx(i,1)+(cos(theta1+theta2)*(dtheta1+dtheta2))*dy(i,1)+(sin(theta1+theta2))*ddy(i,1))*(L1*sin(theta2)))/((L1*sin(theta2))*(L1*sin(theta2)))-(((L1*theta2*cos(theta2))*(cos(theta1+theta2)*dx(i,1)+sin(theta1+theta2)*dy(i,1)))/((L1*sin(theta2))*(L1*sin(theta2))));
    %
    % ddtheta2_1 = ((-L1*(-dtheta1*dx(i,1)*sin(theta1)+ddx(i,1)*cos(theta1))-L2*((-sin(theta1+theta2)*(dtheta1+dtheta2))*dx(i,1)+cos(theta1+theta2)*ddx(i,1)))/((L1*L2*sin(theta2))*(L1*L2*sin(theta2))));
    % ddtheta2_2 = ((-L1*(dtheta1*dy(i,1)*cos(theta1)+ddy(i,1)*sin(theta1))-L2*(cos(theta1+theta2)*(dtheta1+dtheta2)*dy(i,1)+sin(theta1+theta2)*ddy(i,1))*(L1*L2*sin(theta2)))/((L1*L2*sin(theta2))*(L1*L2*sin(theta2))));
    % ddtheta2_3 = ((-(L1*L2*dtheta2*cos(theta2))*(dx(i,1)*(-L1*cos(theta1)-L2*cos(theta1+theta2))+dy(i,1)*(-L1*sin(theta1)-L2*sin(theta1+theta2))))/((L1*L2*sin(theta2))*(L1*L2*sin(theta2))));
    % 
    % ddtheta2(i) = (ddtheta2_1)+(ddtheta2_2)+(ddtheta2_3);

end

% Plot the joint angles, velocities, and accelerations
subplot(2,1,1);
plot(t, theta1, t, theta2);
xlabel('Time');
ylabel('Joint Angle');
title('Joint Angles vs Time for O');
legend('Joint 1', 'Joint 2');

subplot(2,1,2);
plot(t, dtheta1, t, dtheta2);
xlabel('Time');
ylabel('Joint Velocity');
title('Joint Velocities vs Time for O');
legend('Joint 1', 'Joint 2');

% subplot(3,1,3);
% plot(t, ddtheta1, t, ddtheta2);
% xlabel('Time');
% ylabel('Joint Acceleration');
% legend('Joint 1', 'Joint 2');
