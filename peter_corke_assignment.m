%% INIT
addpath ~/peter-corke-toolbox/rtb ~/peter-corke-toolbox/common ~/peter-corke-toolbox/smtb

% L(1) = Link('d', 0.352, 'a', 0.07, 'alpha', -pi/2, 'm', 50, 'I', [0.3 0.3 0.3]);
% L(2) = Link('d', 0, 'a', 0.36, 'alpha', 0,'m',30, 'I', [0.3 0.3 0.3]);
% L(3) = Link('d', 0, 'a', 0.38, 'alpha', 0 ,'m',20, 'I', [0.3 0.3 0.3]);

L(1) = Link('d', 0.352, 'a', 0.07, 'alpha', -pi/2, 'm', 50);
L(2) = Link('d', 0, 'a', 0.36, 'alpha', 0,'m',30);
L(3) = Link('d', 0, 'a', 0.38, 'alpha', pi/2 ,'m',20);

robot = SerialLink(L,'name','robot');

%% Torque

q = deg2rad([0 20 30]); 

robot.plot(deg2rad([0 -90 90]))

grav = [0, 0, -9.81];
taug = robot.gravload(q, grav);
disp(taug./100)

% disp(robot.inertia(q)./100)

%% Path
start = deg2rad([20 20 30]);
goal = deg2rad([-10 40 0]);
t = jtraj(start,goal,100);
robot.plot(t)

%% Fall
tau_fun = @(r,t,q,qd) zeros(1,3);
[t, q, qd] = robot.fdyn(10, tau_fun, zeros(1,3),zeros(1,3));
robot.plot(q)