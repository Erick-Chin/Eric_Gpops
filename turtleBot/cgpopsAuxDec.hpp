// Copyright (c) Yunus M. Agamawi and Anil Vithala Rao.  All Rights Reserved
//
// cgpopsAuxDec.hpp
// CGPOPS NLP Variables
// First declaration or global variables used in CGPOPS for NLP
//

#ifndef __CGPOPS_AUX_DEC_HPP__
#define __CGPOPS_AUX_DEC_HPP__
#include <math.h>


    double Rx1 = 2.75;        //Radar x-Position (center)
    double Ry1 = 3.5;         //Radar y-Position (center)
    double Ra1 = 1;           //Radar semi-major axis
    double Rb1 = 1;           //Radar semi-minor axis
    double Rr1 = 1;           //Radar radius
    double Rp1 = 2;   
    
    double Rx2 = 5;        //Radar x-Position (center)
    double Ry2 = 5;         //Radar y-Position (center)
    double Ra2 = 1;           //Radar semi-major axis
    double Rb2 = 1;           //Radar semi-minor axis
    double Rr2 = 1;           //Radar radius
    double Rp2 = 2;   

    double Rx3 = 6.75;        //Radar x-Position (center)
    double Ry3 = 6.5;         //Radar y-Position (center)
    double Ra3 = 1;           //Radar semi-major axis
    double Rb3 = 1;           //Radar semi-minor axis
    double Rr3 = 1;           //Radar radius
    double Rp3 = 2;   

    x = input phase state at index[0];
    y = input phase state at index[1];
    theta = input phase state at index[2];
    v = input phase control at index[0];
    u2 = input phase control at index[1];
    xdot = v*cos(theta);
    ydot = v*sin(theta);
    thetadot = u2;

    for(int i = 0; i<length; i++){
        x = phaseState[i][0];
    }
    //double g1 = ((x-Rx1)/Ra1)^Rp1+((y-Ry1)/Rb1)^Rp1-Rr1^Rp1;
    double g2 = ((x-Rx2)/Ra2)^Rp2+((y-Ry2)/Rb2)^Rp2-Rr2^Rp2;
    double g3 = ((x-Rx3)/Ra3)^Rp3+((y-Ry3)/Rb3)^Rp3-Rr3^Rp3;

    phaseout dynamics[] = xdot, ydog, thetadot;

    phase path[] = g1, g2, g3;

    public double column(double array[][],int i){
            
        }


#endif









