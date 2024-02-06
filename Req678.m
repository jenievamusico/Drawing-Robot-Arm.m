clc;
clear;

% This code prints plots of the 2D trajectory for each letter, as well as
% the position, velocity and acceleration of the end effector over time

% Define a set of 2-D waypoints for the letter A and connect them using a trapezoidal
% profile where each segment has a duration of 1 second
wpts = [0.05 0.05 0; 0.2 0.35 0; 0.35 0.05 0; 0.275 0.2 0; 0.125 0.2 0]';
[q,qd,qdd,t] = trapveltraj(wpts,100,EndTime=1);

tpts = 0:size(wpts,2)-1;
helperPlotTaskSpaceTraj("EndTime = 1",t,q,qd,qdd,wpts,tpts);

% Define a set of 2-D waypoints for the letter O and connect them using a trapezoidal
% profile where each segment has a duration of 1 second
t = (0:0.2:10)'; % Time
count = length(t);
center = [0.2 0.2 0];
radius = 0.15;
theta = t*(2*pi/t(end));
wpts = [center + radius*[cos(theta) sin(theta) zeros(size(theta))]]';
[q,qd,qdd,t] = trapveltraj(wpts,100,EndTime=1);

tpts = 0:size(wpts,2)-1;
helperPlotTaskSpaceTraj("EndTime = 1",t,q,qd,qdd,wpts,tpts);

% function for plots
function helperPlotTaskSpaceTraj(titleText,t,q,qd,qdd,wpts,tpts)

% Plot 2D position 
figure
sgtitle(titleText);

subplot(3,3,[1 7]);
plot(q(1,:),q(2,:));
hold all
plot(wpts(1,:),wpts(2,:),'x','MarkerSize',7,'LineWidth',2);
xlim('padded');
ylim('padded');
xlabel('X');
ylabel('Y');
title('2-D Trajectory')

% Plot X and Y position with time
subplot(3,3,[2 3]);
plot(t,q);
if nargin > 5
    hold all
    plot(tpts,wpts,'x','MarkerSize',7,'LineWidth',2);
end
ylim('padded')
xlabel('Time');
ylabel('Position');
title('Position vs Time');
legend({'X','Y'})

% Plot X and Y velocity with time
subplot(3,3,[5 6]);
plot(t,qd)
ylim('padded')
xlabel('Time');
ylabel('Velocity');
title('Velocity vs Time');
legend({'X','Y'})

% Plot X and Y acceleration with time
subplot(3,3,[8 9]);
plot(t,qdd)
ylim('padded')
xlabel('Time');
ylabel('Acceleration');
title('Acceleration vs Time');
legend({'X','Y'})


end

