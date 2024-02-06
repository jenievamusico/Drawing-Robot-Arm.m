clc;
clear;

% The first section of this code defines the robot and its links using the
% rigidBodyTree and rigidBody classes. The robot has two links and a fixed
% end effector. The lengths of the links are specified as L1 and L2.

% The next section of the code sets up the inverse kinematics solver using
% the inverseKinematics class. The code then generates a series of points
% in space that the end effector will move to over time for the letter A.

% The code then uses a loop to perform inverse kinematics for each point in
% the trajectory, and stores the resulting joint angles in qs.

% Finally, the code uses a loop to show the robot moving to each point in
% the trajectory. The show function is used to display the robot at each
% configuration, and the rateControl function is used to control the speed
% of the animation.

% Define a rigid body tree to represent the robot
robot = rigidBodyTree('DataFormat','column','MaxNumBodies',3);

% Define the length of the robot links
L1 = 0.3;
L2 = 0.3;

% Define the links of the robot
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

% define trajectory points for the letter A

% from home to the first trajectory point
t0_0 = (0.05:0.055:0.9)'; % Time
t0 = flip(t0_0);
count0 = length(t0);
y0 = -0.06*t0+0.0525;
z0 = zeros(length(t0), 1);

t1 = (0.05:0.01:0.2)'; % Time
count1 = length(t1);
y1 = 2*t1 - 0.05 ;
z1 = zeros(length(t1), 1);

t2 = (0.2:0.01:0.35)';
count2 = length(t2);
y2 = -2*t2+0.75;
z2 = zeros(length(t2),1);

t3_0 = (0.275:0.005:0.35)';
t3 = flip(t3_0);
count3 = length(t3);
y3 = -2*t3+0.75;
z3 = zeros(length(t3),1);

t4_0 = (0.125:0.01:0.275)';
t4 = flip(t4_0);
count4 = length(t4);
y4 = 0.2*ones(length(t4),1);
z4 = zeros(length(t4),1);

% from the last trajectory point back to home
t5 = (0.125:0.05:0.9)';
count5 = length(t5);
y5 = -0.258*t5+0.2258;
z5 = zeros(length(t5),1);

q0 = homeConfiguration(robot);
ndof = length(q0);

ik = inverseKinematics('RigidBodyTree', robot);
weights = [0, 0, 0, 1, 1, 0];
endEffector = 'tool';

qInitial = q0; % Use home configuration as the initial guess

points = [t0 y0 z0];

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

figure
show(robot,qs(1,:)');
view(2)
ax = gca;
ax.Projection = 'orthographic';
hold on

plot([0.05 0.2],[0.05 0.35], 'r')   % / line of A
plot([0.2 0.35],[0.35 0.05], 'r')   % \ line of A
plot([0.125 0.275],[0.2 0.2], 'r')  % - line of A

axis([-0.1 1 -0.3 0.5])

framesPerSecond = 10;
r = rateControl(framesPerSecond);

for i = 1:count0
    show(robot,qs(i,:)','PreservePlot',false);
    drawnow
    waitfor(r);
end

points = [t1 y1 z1];

qs = zeros(count1, ndof);   

for i = 1:count1
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

for i = 1:count1
    show(robot,qs(i,:)','PreservePlot',false);
    drawnow
    waitfor(r);
end

points = [t2 y2 z2];

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


for i = 1:count2
    show(robot,qs(i,:)','PreservePlot',false);
    drawnow
    waitfor(r);
end


points = [t3 y3 z3];

qs = zeros(count3, ndof);   

for i = 1:count3
    % Solve for the configuration satisfying the desired end effector
    % position
    point = points(i,:);
    qSol = ik(endEffector,trvec2tform(point),weights,qInitial);
    % Store the configuration
    qs(i,:) = qSol;
    % Start from prior solution
    qInitial = qSol;
end


for i = 1:count3
    show(robot,qs(i,:)','PreservePlot',false);
    drawnow
    waitfor(r);
end


points = [t4 y4 z4];

qs = zeros(count4, ndof);   

for i = 1:count4
    % Solve for the configuration satisfying the desired end effector
    % position
    point = points(i,:);
    qSol = ik(endEffector,trvec2tform(point),weights,qInitial);
    % Store the configuration
    qs(i,:) = qSol;
    % Start from prior solution
    qInitial = qSol;
end


for i = 1:count4
    show(robot,qs(i,:)','PreservePlot',false);
    drawnow
    waitfor(r);
end


points = [t5 y5 z5];

qs = zeros(count5, ndof);   

for i = 1:count5
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

for i = 1:count5
    show(robot,qs(i,:)','PreservePlot',false);
    drawnow
    waitfor(r);
end

