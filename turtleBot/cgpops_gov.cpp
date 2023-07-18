
// Copyright (c) Yunus M. Agamawi and Anil Vithala Rao.  All Rights Reserved
//
// cgpops_gov.cpp
// CGPOPS Tool Box
// Define governing equations of continouos time optimal control problem here
// Moon lander problem


#include "cgpops_gov.hpp"

#define x1      x[0]
#define y       x[1]
#define theta   x[2]

#define v       u[0]
#define u2      u[1]
#define u_pen   q[0][0]

template <class T> void finalTime::eq_def(T& lhs, T** x0, T** xf, T** q, T** t0, T** tf,
                                           T* s, T* e)
{ 
  u_pen;

}

template <class T> void xDot::eq_def(T& lhs, T* x, T* u, T& t, T* s, T* p)
{ 
    lhs = v*cos(theta); 
}

template <class T> void yDot::eq_def(T& lhs, T* x, T* u, T& t, T* s, T* p)
{
    lhs = v*sin(theta);
}
template <class T> void thetaDot::eq_def(T& lhs, T* x, T* u, T& t, T* s, T* p)
{
    lhs = u2;
}

template <class T> void g1::eq_def(T& lhs, T* x, T* u, T& t, T* s, T* p)
{
    lhs = (pow((x1-Rx1)/Ra1,Rp1)+pow((y-Ry1)/Rb1,Rp1)-pow(Rr1,Rp1));  
}

template <class T> void g2::eq_def(T& lhs, T* x, T* u, T& t, T* s, T* p)
{
    lhs = (pow((x1-Rx2)/Ra2,Rp2)+pow((y-Ry2)/Rb2,Rp2)-pow(Rr2,Rp2));
}

template <class T> void g3::eq_def(T& lhs, T* x, T* u, T& t, T* s, T* p)
{
    lhs = (pow((x1-Rx3)/Ra3,Rp3)+pow((y-Ry3)/Rb3,Rp3)-pow(Rr3,Rp3));
}


void finalTime::eval_eq(double& lhs, double** x0, double** xf, double** q, double** t0,
                         double** tf, double* s, double* e)
{eq_def(lhs,x0,xf,q,t0,tf,s,e);}
void finalTime::eval_eq(HyperDual& lhs, HyperDual** x0, HyperDual** xf, HyperDual** q,
                         HyperDual** t0, HyperDual** tf, HyperDual* s, HyperDual* e)
{eq_def(lhs,x0,xf,q,t0,tf,s,e);}
void finalTime::eval_eq(Bicomplex& lhs, Bicomplex** x0, Bicomplex** xf, Bicomplex** q,
Bicomplex** t0, Bicomplex** tf, Bicomplex* s, Bicomplex* e)
{eq_def(lhs,x0,xf,q,t0,tf,s,e);}


void xDot::eval_eq(double& lhs, double* x, double* u, double& t, double* s, double* p)
{eq_def(lhs,x,u,t,s,p);}
void xDot::eval_eq(HyperDual& lhs, HyperDual* x, HyperDual* u, HyperDual& t, HyperDual* s,
                   HyperDual* p)
{eq_def(lhs,x,u,t,s,p);}
void xDot::eval_eq(Bicomplex& lhs, Bicomplex* x, Bicomplex* u, Bicomplex& t, Bicomplex* s,
                   Bicomplex* p)
{eq_def(lhs,x,u,t,s,p);}

void yDot::eval_eq(double& lhs, double* x, double* u, double& t, double* s, double* p)
{eq_def(lhs,x,u,t,s,p);}
void yDot::eval_eq(HyperDual& lhs, HyperDual* x, HyperDual* u, HyperDual& t, HyperDual* s,
                   HyperDual* p)
{eq_def(lhs,x,u,t,s,p);}
void yDot::eval_eq(Bicomplex& lhs, Bicomplex* x, Bicomplex* u, Bicomplex& t, Bicomplex* s,
                   Bicomplex* p)
{eq_def(lhs,x,u,t,s,p);}
void thetaDot::eval_eq(double& lhs, double* x, double* u, double& t, double* s,
                          double* p)
{eq_def(lhs,x,u,t,s,p);}
void thetaDot::eval_eq(HyperDual& lhs, HyperDual* x, HyperDual* u, HyperDual& t,
                          HyperDual* s, HyperDual* p)
{eq_def(lhs,x,u,t,s,p);}
void thetaDot::eval_eq(Bicomplex& lhs, Bicomplex* x, Bicomplex* u, Bicomplex& t,
                          Bicomplex* s, Bicomplex* p)
{eq_def(lhs,x,u,t,s,p);}


void g1::eval_eq(double& lhs, double* x, double* u, double& t, double* s,
                          double* p)
{eq_def(lhs,x,u,t,s,p);}
void g1::eval_eq(HyperDual& lhs, HyperDual* x, HyperDual* u, HyperDual& t,
                          HyperDual* s, HyperDual* p)
{eq_def(lhs,x,u,t,s,p);}
void g1::eval_eq(Bicomplex& lhs, Bicomplex* x, Bicomplex* u, Bicomplex& t,
                          Bicomplex* s, Bicomplex* p)
{eq_def(lhs,x,u,t,s,p);}


void g2::eval_eq(double& lhs, double* x, double* u, double& t, double* s,
                          double* p)
{eq_def(lhs,x,u,t,s,p);}
void g2::eval_eq(HyperDual& lhs, HyperDual* x, HyperDual* u, HyperDual& t,
                          HyperDual* s, HyperDual* p)
{eq_def(lhs,x,u,t,s,p);}
void g2::eval_eq(Bicomplex& lhs, Bicomplex* x, Bicomplex* u, Bicomplex& t,
                          Bicomplex* s, Bicomplex* p)
{eq_def(lhs,x,u,t,s,p);}


void g3::eval_eq(double& lhs, double* x, double* u, double& t, double* s,
                          double* p)
{eq_def(lhs,x,u,t,s,p);}
void g3::eval_eq(HyperDual& lhs, HyperDual* x, HyperDual* u, HyperDual& t,
                          HyperDual* s, HyperDual* p)
{eq_def(lhs,x,u,t,s,p);}
void g3::eval_eq(Bicomplex& lhs, Bicomplex* x, Bicomplex* u, Bicomplex& t,
                          Bicomplex* s, Bicomplex* p)
{eq_def(lhs,x,u,t,s,p);}

