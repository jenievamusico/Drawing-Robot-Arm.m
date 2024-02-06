clc;
clear;

% The first section of this code defines the robot and its links using the
% rigidBodyTree and rigidBody classes. The robot has two links and a fixed
% end effector. The lengths of the links are specified as L1 and L2.

% The next section of the code sets up the inverse kinematics solver using
% the inverseKinematics class. The code then generates a series of points
% in space that the end effector will move to over time for the letter O.

% The code then uses a loop to perform inverse kinematics for each point in
% the trajectory, and stores the resulting joint angles in qs.

% Finally, the code uses a loop to show the robot moving to each point in
% the trajectory. The show function is used to display the robot at each
% configuration, and the rateControl function is used to control the speed
% of the animation.

% Define a rigid body tree to represent the robot
robot = rigidBodyTree('DataFormat','column','MaxNumBodies',3);

L1 = 0.3;
L2 = 0.3;

body = rigidBody('link1');
joint = rigidBodyJoint('joint1', 'revolute');
setFixedTransform(joint,trvec2tform([0 0 0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'base');

body = rigidBody('link2');
joint = rigidBodyJoint('joint2','revolute');
setFixedTransform(joint, trvec2tform([L1,0,0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'link1');

body = rigidBody('tool');
joint = rigidBodyJoint('fix1','fixed');
setFixedTransform(joint, trvec2tform([L2, 0, 0]));
body.Joint = joint;
addBody(robot, body, 'link2');

showdetails(robot)

% Set the initial configuration of the robot
q0 = homeConfiguration(robot);

% Get the number of degrees of freedom of the robot
ndof = length(q0);

ik = inverseKinematics('RigidBodyTree', robot);
weights = [0, 0, 0, 1, 1, 0];
endEffector = 'tool';

% Use the home configuration as the initial guess for the inverse kinematics solver
qInitial = q0;


% define trajectory points for the letter O
t = (0:0.2:10)'; % Time
count = length(t);
center = [0.2 0.2 0];
radius = 0.15;
theta = t*(2*pi/t(end));

points = center + radius*[cos(theta) sin(theta) zeros(size(theta))];

qs = zeros(count, ndof);  

figure
show(robot,qs(1,:)');
view(2)
ax = gca;
ax.Projection = 'orthographic';
hold on
plot(points(:,1),points(:,2),'k')
axis([-0.1 1 -0.3 0.5])

% define trajectory points for the robot to move from home to the first
% trajectory point on the O
x0_0 = (0.35:0.05:0.9)'; % Time
x0 = flip(x0_0);
count0 = length(x0);
y0 = -0.364*x0+0.3276;
z0 = zeros(length(x0), 1);

points = [x0 y0 z0];

qs = zeros(count0, ndof);   

for i = 1:count0
    % Solve for the configuration satisfying the desired end effector
    % position
    point = points(i,:);
    qSol = ik(endEffector,trvec2tform(point),weights,qInitial);
    % Store the configuration
    qs(i,:) = qSol;
    % Start from prior solution
    qInitial = qSol;
end

framesPerSecond = 10;
r = rateControl(framesPerSecond);

for i = 1:count0
    show(robot,qs(i,:)','PreservePlot',false);
    drawnow
    waitfor(r);
end

% define trajectory points for the letter O
t = (0:0.2:10)'; % Time
count = length(t);
center = [0.2 0.2 0];
radius = 0.15;
theta = t*(2*pi/t(end));

points = center + radius*[cos(theta) sin(theta) zeros(size(theta))];

qs = zeros(count, ndof);   

for i = 1:count
    % Solve for the configuration satisfying the desired end effector
    % position
    point = points(i,:);
    qSol = ik(endEffector,trvec2tform(point),weights,qInitial);
    % Store the configuration
    qs(i,:) = qSol;
    % Start from prior solution
    qInitial = qSol;
end

framesPerSecond = 15;
r = rateControl(framesPerSecond);

for i = 1:count
    show(robot,qs(i,:)','PreservePlot',false);
    drawnow
    waitfor(r);
end

% define trajectory points for the robot to move from the last
% trajectory point on the O back to home
x2 = (0.35:0.05:0.9)';
count2 = length(x2);
y2 = -0.364*x2+0.3276;
z2 = zeros(length(x2),1);

points = [x2 y2 z2];

qs = zeros(count2, ndof);   

for i = 1:count2
    % Solve for the configuration satisfying the desired end effector
    % position
    point = points(i,:);
    qSol = ik(endEffector,trvec2tform(point),weights,qInitial);
    % Store the configuration
    qs(i,:) = qSol;
    % Start from prior solution
    qInitial = qSol;
end

framesPerSecond = 10;
r = rateControl(framesPerSecond);

for i = 1:count2
    show(robot,qs(i,:)','PreservePlot',false);
    drawnow
    waitfor(r);
end

