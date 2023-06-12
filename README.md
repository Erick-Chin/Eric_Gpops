# Eric_Gpops

short descriptions and comments:
**all of the auxdata.Rp objects are in the cgpopsAuxDec.hpp file and can be called using the cgpopsAuxExt.hpp files** 
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

**all of the variable creation and setup for problem bounds are found in the cgpops_main.cpp file** 
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
bounds.phase.initialstate.upper = [x0,0]; 
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




