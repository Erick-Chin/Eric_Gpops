%---------------------------------------------------%
% Classical SUAS fixed target Problem:              %
%---------------------------------------------------%
% The problem solved here is given as follows:      %
%   Minimize t_f                                    %
% subject to the dynamic constraints                %
%    dx/dt = va*sin(theta)                          %
%    dy/dt = va*cos(theta)                          %
%    dtheta/dt = u                                  %
% and the boundary conditions                       %
%    x(0) = 1, y(0) = 0.5, theta(0) = pi/2          %
%    x(t_f) = 9, y(t_f) = 9.5, theta(t_f) = free    %
%---------------------------------------------------%

clear all; close all; clc

auxdata.Rx1 = 2.75;  % Radar x-Position (center)
auxdata.Ry1 = 3.5;  % Radar y-Position (center)
auxdata.Ra1 = 1;  % Radar semi-major axis  0.5
auxdata.Rb1 = 1;  % Radar semi-minor axis  2.2
auxdata.Rr1 = 1;  % Radar radius
auxdata.Rp1 = 2;  % Corners

auxdata.Rx2 = 5;  % Radar x-Position (center)
auxdata.Ry2 = 5;  % Radar y-Position (center)
auxdata.Ra2 = 1;  % Radar semi-major axis
auxdata.Rb2 = 1;  % Radar semi-minor axis
auxdata.Rr2 = 1;  % Radar radius
auxdata.Rp2 = 2;  % Corners

auxdata.Rx3 = 6.75;  % Radar x-Position (center)
auxdata.Ry3 = 6.5;  % Radar y-Position (center)
auxdata.Ra3 = 1;  % Radar semi-major axis
auxdata.Rb3 = 1;  % Radar semi-minor axis
auxdata.Rr3 = 1;  % Radar radius
auxdata.Rp3 = 2;  % Corners

% optimal solution: tfmax = 30, u2 = +- 2, rate = 50 snopt
% poor guess, uncomment 31 & 32, comment 33
t0 = 0; 
tfmin = 0; tfmax = 25; %thetaf = deg2rad(0); 
x0 = 1; y0 = 0.5; theta0 = pi/2;
xf = 9;
yf = 9.5;
xmin = 0; xmax = 10;
ymin = 0; ymax = 10;
thetamin = -pi; thetamax = pi;
u1min = 1; u1max = 1;
% rate = 30;%12;50
% u2min = -deg2rad(rate); u2max = deg2rad(rate);
u2min = -2; u2max = 2;

tic
%-------------------------------------------------------------------------%
%----------------------- Setup for Problem Bounds ------------------------%
%-------------------------------------------------------------------------%
bounds.phase.initialtime.lower = t0; 
bounds.phase.initialtime.upper = t0;
bounds.phase.finaltime.lower = tfmin; 
bounds.phase.finaltime.upper = tfmax;
bounds.phase.initialstate.lower = [x0,y0,theta0]; 
bounds.phase.initialstate.upper = [x0,y0,theta0]; 
bounds.phase.state.lower = [xmin,ymin,thetamin]; 
bounds.phase.state.upper = [xmax,ymax,thetamax]; 
bounds.phase.finalstate.lower = [xf,yf,thetamin]; 
bounds.phase.finalstate.upper = [xf,yf,thetamax]; 
bounds.phase.control.lower = [u1min, u2min]; 
bounds.phase.control.upper = [u1max, u2max];
bounds.phase.path.lower  = [0, 0, 0];
bounds.phase.path.upper  = [100000, 100000, 100000];
% bounds.phase.integral.lower  = 0;
% bounds.phase.integral.upper  = 1000;

%-------------------------------------------------------------------------%
%---------------------- Provide Guess of Solution ------------------------%
%-------------------------------------------------------------------------%
n = 4;  % 4 for good guess, 6 or 14 
tfmax2 = 20;
guess.phase.time    = [linspace(t0,tfmax2,n)'];
guess.phase.state   = [[x0; 2.75; 6.75; xf],[y0; 7; 3; yf], theta0*ones(n,1)];
% guess.phase.state   = [[x0; 1.5; 3.5; 4; 8; xf],[y0; 8; 8; 3; 2; yf], theta0*ones(n,1)];
% guess.phase.state   = [[x0; 1.25; 1.25; 1.5; 4.75; 4.125; 3.625; 3.625; 3.625; 4.75; 8.25; 8.5; 8.5; xf],[y0; 3.5; 8.5; 8.5; 8.5; 6.25; 6.25; 5.75; 2.25; 1.5; 1.5; 1.5; 6.5;  yf], theta0*ones(n,1)];
guess.phase.control = [1*ones(n,1),0*ones(n,1)];
guess.phase.integral = 0;
%-------------------------------------------------------------------------%
%----------Provide Mesh Refinement Method and Initial Mesh ---------------%
%-------------------------------------------------------------------------%
mesh.method       = 'hp-PattersonRao';
mesh.tolerance    = 1e-6;
mesh.maxiterations = 45;
mesh.colpointsmin = 4;
mesh.colpointsmax = 10;
setup.mesh.phase.colpoints = 4*ones(1,10);
%-------------------------------------------------------------------------%
%------------- Assemble Information into Problem Structure ---------------%        
%-------------------------------------------------------------------------%
setup.name                        = 'UAS_fixed_target-Problem';
setup.functions.continuous        = @UAS_fixed_targetContinuous;
setup.functions.endpoint          = @UAS_fixed_targetEndpoint;
setup.auxdata                     = auxdata;
setup.displaylevel                = 2;
setup.bounds                      = bounds;
setup.guess                       = guess;
setup.mesh                        = mesh; 
setup.scales.method               = 'automatic-guess';
setup.nlp.solver                  = 'ipopt';
setup.nlp.snoptoptions.tolerance  = 1e-5;
setup.derivatives.supplier        = 'sparseCD';
setup.derivatives.derivativelevel = 'second';
setup.method                      = 'RPM-Differentiation';

%-------------------------------------------------------------------------%
%------------------------- Solve Problem Using GPOP2 ---------------------%
%-------------------------------------------------------------------------%
output   = gpops2(setup);
solution = output.result.solution;
toc
