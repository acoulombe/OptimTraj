clc; clear;
addpath ..
addpath /home/alex/Documents/MATLAB/Examples/R2019b/robotics/CheckForEnvironmentalCollisionsWithManipulatorsExample
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       Load Kuka iiwa model
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
robot = importrobot("iiwa7.urdf");
robot.DataFormat = 'column';

robotCollisionModel = exampleHelperManipCollisionsFromVisuals(robot);
buildEnvironment;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%           Set problem start and target configurations
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
t0 = 0;         % Initial Time
tF = 5.0;       % Final Time
x0 = [pi/4;pi/8;0;0;0;0;0;0;0;0;0;0;0;0];     % Initial State - Order : [q, dq]
xF = [0;pi/8;0;-pi/4;0;pi/4;0;0;0;0;0;0;0;0];     % Final State

% exampleHelperVisualizeCollisionEnvironment(worldCollisionModel);
% % show(robot, xF(1:7));
% 
% xA = [pi/2;pi/16;0;-pi/16;0;pi/16;pi/6;pi/32;0;-pi/32;0;pi/32];
% xB = [pi/6;pi/6;0;-pi/16;0;pi/16;pi/6;pi/32;0;-pi/32;0;pi/32];
% x = [x0(1:7), xA(1:7), xB(1:7)];
% show(robot, xA(1:7));
% 
% [non, cd] = kukaCheckCD(robot, robotCollisionModel, worldCollisionModel,  [x, xF(1:7)])
% 
% return;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       Set up function handles                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.func.dynamics = @(t,x,u)( kukaDynamics(x,u,robot) );

% problem.func.pathObj = @(t,x,u)( ones(size(t)) );  % Time Constraint
problem.func.pathObj = @(t,x,u)(sum(u.^2));  %Simple torque-squared

problem.func.pathCst = @(t,x,u)( kukaCheckCD(robot, robotCollisionModel, worldCollisionModel, [x(1:7, :), xF(1:7)]) );

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%               Set up bounds on time, state, and control                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
problem.bounds.initialTime.low = t0;
problem.bounds.initialTime.upp = t0;
problem.bounds.finalTime.low = tF;
problem.bounds.finalTime.upp = tF;

problem.bounds.state.low = [-170/180*pi;... % q1
                            -120/180*pi;... % q2
                            -170/180*pi;... % q3
                            -120/180*pi;... % q4
                            -170/180*pi;... % q5
                            -120/180*pi;... % q6
                            -175/180*pi;... % q7
                            -98/180*pi;...  % dq1
                            -98/180*pi;...  % dq2
                            -100/180*pi;... % dq3
                            -130/180*pi;... % dq4
                            -140/180*pi;... % dq5
                            -180/180*pi;... % dq6
                            -180/180*pi];   % dq7

problem.bounds.state.upp = [170/180*pi;... % q1
                            120/180*pi;... % q2
                            170/180*pi;... % q3
                            120/180*pi;... % q4
                            170/180*pi;... % q5
                            120/180*pi;... % q6
                            175/180*pi;... % q7
                            98/180*pi;...  % dq1
                            98/180*pi;...  % dq2
                            100/180*pi;... % dq3
                            130/180*pi;... % dq4
                            140/180*pi;... % dq5
                            180/180*pi;... % dq6
                            180/180*pi];   % dq7

problem.bounds.initialState.low = x0;
problem.bounds.initialState.upp = x0;
problem.bounds.finalState.low = xF;
problem.bounds.finalState.upp = xF;

% https://www.kuka.com/-/media/kuka-downloads/imported/9cb8e311bfd744b4b0eab25ca883f6d3/kuka_sensitiverobotics_lbriiwa_insert_en.pdf
problem.bounds.control.low = [-176; -176; -110; -110; -110; -40; -40];
problem.bounds.control.upp = [176; 176; 110; 110; 110; 40; 40];

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Options:                                      %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.options.method = 'trapezoid';
problem.options.trapezoid.nGrid = 5;

% problem.options.method = 'hermiteSimpson';
% problem.options.hermiteSimpson.nSegment = 2;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%              Create an initial guess for the trajectory                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% tA = t0 + 0.25*(tF-t0);
% tB = t0 + 0.75*(tF-t0);
% 
% % xA = x0 + 0.25*(xF-x0);
% % xB = x0 + 0.75*(xF-x0);
% 
% xA = [pi/2;pi/16;0;-pi/16;0;pi/16;pi/6;pi/32;0;-pi/32;0;pi/32;0;0];
% xB = [pi/6;pi/6;0;-pi/16;0;pi/16;pi/6;pi/32;0;-pi/32;0;pi/32;0;0];

% u0 = zeros(7,1);

% problem.guess.time = [t0, tA, tB, tF];
% problem.guess.state = [x0, xA, xB, xF];
% problem.guess.control = [u0, u0, u0, u0];

load('initial_guess_t', 't');
load('initial_guess_x', 'z');
load('initial_guess_u', 'u');

problem.guess.time = t;
problem.guess.state = z;
problem.guess.control = u;


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Solve!                                        %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
tic
soln = optimTraj(problem);
toc
% Interpolate the solution on a uniform grid for plotting and animation:
tGrid = soln(end).grid.time;
t = linspace(tGrid(1),tGrid(end),100);
z = soln(end).interp.state(t);
u = soln(end).interp.control(t);

