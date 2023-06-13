// Copyright (c) Yunus M. Agamawi and Anil Vithala Rao.  All Rights Reserved
//
// cgpops_gov.cpp
// CGPOPS Tool Box
// Define governing equations of continouos time optimal control problem here



#include "cgpops_gov.hpp"

#define thrust1 p[0]
#define thrust2 p[1]
#define x_s     x[0]
#define y       x[1]
#define vx      x[2]
#define vy      x[3]
#define theta   x[4]
#define omega   x[5]
#define u1      u[0]
#define u2      u[1]
#define u3      u[2]
#define u4      u[3]
//#define u_pen   gammaG*q[0][0]
#define u_pen   lhs=0;for(int pn=0;pn<PG;pn++)lhs=lhs+gammaG*q[pn][0];

template <class T> void MinControl::eq_def(T& lhs, T** x0, T** xf, T** q, T** t0, T** tf,
                                           T* s, T* e)
{
//    lhs = u_pen;
    u_pen;
}
template <class T> void Thrust1::eq_def(T& lhs, T* x, T* u, T& t, T* s, T* p)
{
    lhs = u1-u2;
}
template <class T> void Thrust2::eq_def(T& lhs, T* x, T* u, T& t, T* s, T* p)
{
    lhs = u3-u4;
}
template <class T> void XDot::eq_def(T& lhs, T* x, T* u, T& t, T* s, T* p)
{
    lhs = vx;
}
template <class T> void YDot::eq_def(T& lhs, T* x, T* u, T& t, T* s, T* p)
{
    lhs = vy;
}
template <class T> void VxDot::eq_def(T& lhs, T* x, T* u, T& t, T* s, T* p)
{
    lhs = (thrust1+thrust2)*cos(theta);
}
template <class T> void VyDot::eq_def(T& lhs, T* x, T* u, T& t, T* s, T* p)
{
    lhs = (thrust1+thrust2)*sin(theta);
}
template <class T> void ThetaDot::eq_def(T& lhs, T* x, T* u, T& t, T* s, T* p)
{
    lhs = omega;
}
template <class T> void OmegaDot::eq_def(T& lhs, T* x, T* u, T& t, T* s, T* p)
{
    lhs = alphaG*thrust1-betaG*thrust2;
}
template <class T> void ControlConstraint1::eq_def(T& lhs, T* x, T* u, T& t, T* s, T* p)
{
    lhs = u1+u2;
}
template <class T> void ControlConstraint2::eq_def(T& lhs, T* x, T* u, T& t, T* s, T* p)
{
    lhs = u3+u4;
}
template <class T> void ControlCost::eq_def(T& lhs, T* x, T* u, T& t, T* s, T* p)
{
    lhs = u1+u2+u3+u4;
}

void MinControl::eval_eq(double& lhs, double** x0, double** xf, double** q, double** t0,
                         double** tf, double* s, double* e)
{eq_def(lhs,x0,xf,q,t0,tf,s,e);}
void MinControl::eval_eq(HyperDual& lhs, HyperDual** x0, HyperDual** xf, HyperDual** q,
                         HyperDual** t0, HyperDual** tf, HyperDual* s, HyperDual* e)
{eq_def(lhs,x0,xf,q,t0,tf,s,e);}
void MinControl::eval_eq(Bicomplex& lhs, Bicomplex** x0, Bicomplex** xf, Bicomplex** q,
                         Bicomplex** t0, Bicomplex** tf, Bicomplex* s, Bicomplex* e)
{eq_def(lhs,x0,xf,q,t0,tf,s,e);}
void Thrust1::eval_eq(double& lhs, double* x, double* u, double& t, double* s, double* p)
{eq_def(lhs,x,u,t,s,p);}
void Thrust1::eval_eq(HyperDual& lhs, HyperDual* x, HyperDual* u, HyperDual& t,
                      HyperDual* s, HyperDual* p)
{eq_def(lhs,x,u,t,s,p);}
void Thrust1::eval_eq(Bicomplex& lhs, Bicomplex* x, Bicomplex* u, Bicomplex& t,
                      Bicomplex* s, Bicomplex* p)
{eq_def(lhs,x,u,t,s,p);}
void Thrust2::eval_eq(double& lhs, double* x, double* u, double& t, double* s, double* p)
{eq_def(lhs,x,u,t,s,p);}
void Thrust2::eval_eq(HyperDual& lhs, HyperDual* x, HyperDual* u, HyperDual& t,
                      HyperDual* s, HyperDual* p)
{eq_def(lhs,x,u,t,s,p);}
void Thrust2::eval_eq(Bicomplex& lhs, Bicomplex* x, Bicomplex* u, Bicomplex& t,
                      Bicomplex* s, Bicomplex* p)
{eq_def(lhs,x,u,t,s,p);}
void XDot::eval_eq(double& lhs, double* x, double* u, double& t, double* s, double* p)
{eq_def(lhs,x,u,t,s,p);}
void XDot::eval_eq(HyperDual& lhs, HyperDual* x, HyperDual* u, HyperDual& t, HyperDual* s,
                   HyperDual* p)
{eq_def(lhs,x,u,t,s,p);}
void XDot::eval_eq(Bicomplex& lhs, Bicomplex* x, Bicomplex* u, Bicomplex& t, Bicomplex* s,
                   Bicomplex* p)
{eq_def(lhs,x,u,t,s,p);}
void YDot::eval_eq(double& lhs, double* x, double* u, double& t, double* s, double* p)
{eq_def(lhs,x,u,t,s,p);}
void YDot::eval_eq(HyperDual& lhs, HyperDual* x, HyperDual* u, HyperDual& t, HyperDual* s,
                   HyperDual* p)
{eq_def(lhs,x,u,t,s,p);}
void YDot::eval_eq(Bicomplex& lhs, Bicomplex* x, Bicomplex* u, Bicomplex& t, Bicomplex* s,
                   Bicomplex* p)
{eq_def(lhs,x,u,t,s,p);}
void VxDot::eval_eq(double& lhs, double* x, double* u, double& t, double* s, double* p)
{eq_def(lhs,x,u,t,s,p);}
void VxDot::eval_eq(HyperDual& lhs, HyperDual* x, HyperDual* u, HyperDual& t,
                    HyperDual* s, HyperDual* p)
{eq_def(lhs,x,u,t,s,p);}
void VxDot::eval_eq(Bicomplex& lhs, Bicomplex* x, Bicomplex* u, Bicomplex& t,
                    Bicomplex* s, Bicomplex* p)
{eq_def(lhs,x,u,t,s,p);}
void VyDot::eval_eq(double& lhs, double* x, double* u, double& t, double* s, double* p)
{eq_def(lhs,x,u,t,s,p);}
void VyDot::eval_eq(HyperDual& lhs, HyperDual* x, HyperDual* u, HyperDual& t,
                    HyperDual* s, HyperDual* p)
{eq_def(lhs,x,u,t,s,p);}
void VyDot::eval_eq(Bicomplex& lhs, Bicomplex* x, Bicomplex* u, Bicomplex& t,
                    Bicomplex* s, Bicomplex* p)
{eq_def(lhs,x,u,t,s,p);}
void ThetaDot::eval_eq(double& lhs, double* x, double* u, double& t, double* s, double* p)
{eq_def(lhs,x,u,t,s,p);}
void ThetaDot::eval_eq(HyperDual& lhs, HyperDual* x, HyperDual* u, HyperDual& t,
                       HyperDual* s, HyperDual* p)
{eq_def(lhs,x,u,t,s,p);}
void ThetaDot::eval_eq(Bicomplex& lhs, Bicomplex* x, Bicomplex* u, Bicomplex& t,
                       Bicomplex* s, Bicomplex* p)
{eq_def(lhs,x,u,t,s,p);}
void OmegaDot::eval_eq(double& lhs, double* x, double* u, double& t, double* s, double* p)
{eq_def(lhs,x,u,t,s,p);}
void OmegaDot::eval_eq(HyperDual& lhs, HyperDual* x, HyperDual* u, HyperDual& t,
                       HyperDual* s, HyperDual* p)
{eq_def(lhs,x,u,t,s,p);}
void OmegaDot::eval_eq(Bicomplex& lhs, Bicomplex* x, Bicomplex* u, Bicomplex& t,
                       Bicomplex* s, Bicomplex* p)
{eq_def(lhs,x,u,t,s,p);}
void ControlConstraint1::eval_eq(double& lhs, double* x, double* u, double& t, double* s,
                                 double* p)
{eq_def(lhs,x,u,t,s,p);}
void ControlConstraint1::eval_eq(HyperDual& lhs, HyperDual* x, HyperDual* u,
                                 HyperDual& t, HyperDual* s, HyperDual* p)
{eq_def(lhs,x,u,t,s,p);}
void ControlConstraint1::eval_eq(Bicomplex& lhs, Bicomplex* x, Bicomplex* u,
                                 Bicomplex& t, Bicomplex* s, Bicomplex* p)
{eq_def(lhs,x,u,t,s,p);}
void ControlConstraint2::eval_eq(double& lhs, double* x, double* u, double& t, double* s,
                                 double* p)
{eq_def(lhs,x,u,t,s,p);}
void ControlConstraint2::eval_eq(HyperDual& lhs, HyperDual* x, HyperDual* u,
                                 HyperDual& t, HyperDual* s, HyperDual* p)
{eq_def(lhs,x,u,t,s,p);}
void ControlConstraint2::eval_eq(Bicomplex& lhs, Bicomplex* x, Bicomplex* u,
                                 Bicomplex& t, Bicomplex* s, Bicomplex* p)
{eq_def(lhs,x,u,t,s,p);}
void ControlCost::eval_eq(double& lhs, double* x, double* u, double& t, double* s,
                          double* p)
{eq_def(lhs,x,u,t,s,p);}
void ControlCost::eval_eq(HyperDual& lhs, HyperDual* x, HyperDual* u, HyperDual& t,
                          HyperDual* s, HyperDual* p)
{eq_def(lhs,x,u,t,s,p);}
void ControlCost::eval_eq(Bicomplex& lhs, Bicomplex* x, Bicomplex* u, Bicomplex& t,
                          Bicomplex* s, Bicomplex* p)
{eq_def(lhs,x,u,t,s,p);}




