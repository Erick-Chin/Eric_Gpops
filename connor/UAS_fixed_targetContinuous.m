%---------------------------------------------%
% BEGIN: function UAS_fixed_targetContinuous.m %
%---------------------------------------------%
function phaseout = UAS_fixed_targetContinuous(input)

Rx1                = input.auxdata.Rx1;
Ry1                = input.auxdata.Ry1;
Ra1                = input.auxdata.Ra1;
Rb1                = input.auxdata.Rb1;  
Rr1                = input.auxdata.Rr1;
Rp1                = input.auxdata.Rp1;

Rx2                = input.auxdata.Rx2;
Ry2                = input.auxdata.Ry2;
Ra2                = input.auxdata.Ra2;
Rb2                = input.auxdata.Rb2; 
Rr2                = input.auxdata.Rr2;
Rp2                = input.auxdata.Rp2;

Rx3                = input.auxdata.Rx3;
Ry3                = input.auxdata.Ry3;
Ra3                = input.auxdata.Ra3;
Rb3                = input.auxdata.Rb3; 
Rr3                = input.auxdata.Rr3;
Rp3                = input.auxdata.Rp3;

x                 = input.phase.state(:,1);
y                 = input.phase.state(:,2);
theta             = input.phase.state(:,3);
v                 = input.phase.control(:,1);
u2                = input.phase.control(:,2);
xdot              = v.*cos(theta);
ydot              = v.*sin(theta);
thetadot          = u2;
%% First constraint

% g1 = log(((x-xc1)/a1).^p1+((y-yc1)/b1).^p1-c1.^p1);
g1 = ((x-Rx1)./Ra1).^Rp1+((y-Ry1)./Rb1).^Rp1-Rr1.^Rp1;

% s = 2;
% K = 1./(1+exp(s.*g1-1));
%% Second constraint

% g2 = log(((x-xc2)/a2).^p2+((y-yc2)/b2).^p2-c2.^p2);
g2 = ((x-Rx2)./Ra2).^Rp2+((y-Ry2)./Rb2).^Rp2-Rr2.^Rp2;

%% Third Constraint

% g3 = log(((x-xc3)/a3).^p3+((y-yc3)/b3).^p3-c3.^p3);
g3 = ((x-Rx3)./Ra3).^Rp3+((y-Ry3)./Rb3).^Rp3-Rr3.^Rp3;

phaseout.dynamics = [xdot, ydot, thetadot];
phaseout.path = [g1 g2, g3];
% phaseout.integrand = K;
%---------------------------------------------%
% END: function UAS_fixed_targetContinuous.m   %
%---------------------------------------------%
