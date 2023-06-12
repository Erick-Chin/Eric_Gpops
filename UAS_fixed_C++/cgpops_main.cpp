/* %---------------------------------------------------%
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
%---------------------------------------------------% */

//#include "cgpops_main.hpp"
#include <math.h>

void cgpops_go(/*doubleMat& cgpopsResults*/)
{
    // Define Global Variables used to determine problem size
    PG      = 1;    // Number of phases in problem
    nsG     = 0;    // Number of static parameters in problem
    nbG     = 0;    // Number of event constraints in problem
    nepG    = 0;    // Number of endpoint parameters in problem

    // Allocate memory for each phase
    initGlobalVars();
 
    // Define number of components for each parameter in problem phases
    // Phase 1 parameters
    nxG[0]  = 6;    // Number of state components in phase 1
    nuG[0]  = 4;    // Number of control components in phase 1
    nqG[0]  = 1;    // Number of integral constraints in phase 1
    ncG[0]  = 2;    // Number of path constraints in phase 1
    nppG[0] = 2;    // Number of phase parameters in phase 2
        
    // Define mesh grids for each phase in problem for LGR collocation
    // Phase 1 mesh grid
    int M1 = numintervalsG;         // Number of intervals used in phase 1 mesh
    int initcolpts = initcolptsG;   // Initial number of collocation points in each
                                    // interval
    double fraction1[M1];           // Allocate memory for fraction vector for phase 1
                                    // mesh
    int colpoints1[M1];             // Allocate memory for colpoints vector for phase 1
                                    // mesh
    for(int m=0; m<M1; m++)
    {
        fraction1[m] = 1.0/((double) M1);
        colpoints1[m] = initcolpts;
    }
    setRPMDG(0,M1,fraction1,colpoints1);

    // Set information for transcribed NLP resulting from LGR collocation using defined
    // mesh grid
    setInfoNLPG();

    /*-----------------------------Provide Problem Bounds-------------------------------*/
    // Phase 1
    double t0 = 0;  // initial and final time values
    double x0 = 1,  y0  = .5, theta0 = M_PI/2;   // initial and final values for x state
    double tfmin = 0, tfmax = 25;
    double xf = 9, yf = 9.5;                 
    double xmin = 0,      xmax = 10;      // minimum and maximum x state component
    double ymin = 0,      ymax = 10;      // minimum and maximum y state component
    double thetamin = -M_PI,thetamax = M_PI;// minimum and maximum x state component
    double ulmin = 1,   ulmax = 1;   // minimum and maximum y state component

    double u2min = -2,       u2max = 2;   // minimum and maximum u1 control component

    // Throughout

    // Define Bounds of Optimal Control Problem
    // Phase 1 bounds
    /*
        nxG is the number of state components in phase 1
        nuG is the number of control components in phase 1 
        ncG is the number of integral constraints in phase 1
        nppG is the number of phase parameters in phase 1
    */
    int phase1 = 0;
    double x0l1[nxG[phase1]],   x0u1[nxG[phase1]];
    double xfl1[nxG[phase1]],   xfu1[nxG[phase1]];
    double xl1[nxG[phase1]],    xu1[nxG[phase1]];
    double ul1[nuG[phase1]],    uu1[nuG[phase1]];
    double ql1[nqG[phase1]],    qu1[nqG[phase1]];
    double cl1[ncG[phase1]],    cu1[ncG[phase1]];
    double t0l1,    t0u1;
    double tfl1,    tfu1;
    //instantiating all the different phase variables and below is setting the differnt values
    x0l1[0] = x0;
    x0l1[1] = y0;
    x0l1[2] = vx0;
    x0l1[3] = vy0;
    x0l1[4] = theta0;
    x0l1[5] = omega0;
    x0u1[0] = x0;
    x0u1[1] = y0;
    x0u1[2] = vx0;
    x0u1[3] = vy0;
    x0u1[4] = theta0;
    x0u1[5] = omega0;
    xfl1[0] = xf;
    xfl1[1] = yf;
    xfl1[2] = vxf;
    xfl1[3] = vyf;
    xfl1[4] = thetaf;
    xfl1[5] = omegaf;
    xfu1[0] = xf;
    xfu1[1] = yf;
    xfu1[2] = vxf;
    xfu1[3] = vyf;
    xfu1[4] = thetaf;
    xfu1[5] = omegaf;
    xl1[0]  = xmin;
    xl1[1]  = ymin;
    xl1[2]  = vxmin;
    xl1[3]  = vymin;
    xl1[4]  = thetamin;
    xl1[5]  = omegamin;
    xu1[0]  = xmax;
    xu1[1]  = ymax;
    xu1[2]  = vxmax;
    xu1[3]  = vymax;
    xu1[4]  = thetamax;
    xu1[5]  = omegamax;
    ul1[0]  = u1min;
    ul1[1]  = u2min;
    ul1[2]  = u3min;
    ul1[3]  = u4min;
    uu1[0]  = u1max;
    uu1[1]  = u2max;
    uu1[2]  = u3max;
    uu1[3]  = u4max;
    t0l1    = t0;
    t0u1    = t0;
    tfl1    = tf;
    tfu1    = tf;
    ql1[0]  = q1min;
    qu1[0]  = q1max;
    cl1[0]  = c1min;
    cl1[1]  = c2min;
    cu1[0]  = c1max;
    cu1[1]  = c2max;

    // Set parameterized constructor for NLP Phase Bounds Class
    setNLPPBG(phase1,x0l1,x0u1,xfl1,xfu1,xl1,xu1,ul1,uu1,ql1,qu1,cl1,cu1,t0l1,t0u1,tfl1,
              tfu1);

    // Whole problem bounds
    double sl[nsG], su[nsG];
    double bl[nbG], bu[nbG];

    // Set parameterized constructor for NLP Whole Bounds Class
    setNLPWBG(sl,su,bl,bu);

    // Provide initial guess for NLP Solver
    // Phase 1 guess
    double x0g1[nxG[phase1]],   xfg1[nxG[phase1]];
    double u0g1[nuG[phase1]],   ufg1[nuG[phase1]];
    double qg1[nqG[phase1]];
    double t0g1,    tfg1;
    x0g1[0] = x0;
    x0g1[1] = y0;
    x0g1[2] = vx0;
    x0g1[3] = vy0;
    x0g1[4] = theta0;
    x0g1[5] = omega0;
    xfg1[0] = xf;
    xfg1[1] = yf;
    xfg1[2] = vxf;
    xfg1[3] = vyf;
    xfg1[4] = thetaf;
    xfg1[5] = omegaf;
    //u0g1[0] #ifndef __CGPOPS_AUX_DEC_HPP__
    //setGlobularData();    //Set any global tabular data used in this problem (page 23 of the reference guide)
#define __CGPOPS_AUX_DEC_HPP__= 0;
    ufg1[3] = 0;
    t0g1    = t0;
    tfg1    = tf;

    // Set parameterized constructor for NLP Phase Guess Class
    setNLPPGG(phase1,x0g1,xfg1,u0g1,ufg1,qg1,t0g1,tfg1);

    // Whole problem guess
    double sg[nsG];

    // Set parameterized constructor for NLP Phase Guess Class
    setNLPWGG(sg);
    
    // Indicate if warm start using previous solution is desired
//    strFileBinG = "cgpopsIPOPTSolutionHD.bin";
//    warmStartBinFlagG = 1;
    
    // Set ocp functions
    objEqG = new MinControl;
    pppEqVecG = {{new Thrust1, new Thrust2}};
    odeEqVecG = {{new XDot, new YDot, new VxDot, new VyDot, new ThetaDot, new OmegaDot}};
    pthEqVecG = {{new ControlConstraint1, new ControlConstraint2}};
    igdEqVecG = {{new ControlCost}};
    
    // Make call to CGOPS handler for IPOPT using user provided output settings
    CGPOPS_IPOPT_caller(cgpopsResults);
    
}

