
#include "cgpops_gov.hpp"
#include <math.h>


#define x   nxG[0]
#define y   nuG[1]
#define theta   nqG[2]
#define v  n[3]
#define u2 n[4]


#define final_time lhs=0;for(int pn=0;pn<PG;pn++)lhs=lhs+q[pn][0];lhs=lhs+xf[PG-1][0]+xf[PG-1][1]+xf[PG-1][2];
/*
    int lhs = 0;
    for(int pn=0; pn<PG; pn++){
        lhs = lhs+q[pn][0];
        lhs = lhs+xf[PG-1][1]+xf[PG-1][2];
    }
*/

template <class T> void finalTime::eq_def(T& lhs, T** x0, T** xf, T** q, T** t0, T** tf,
                                           T* s, T* e)
{
    final_time;
}

template <class T> void xdot::eq_def(T& lhs, T* x, T* u, T& t, T* s, T* p)
{
    lhs = v * cos(theta);
}

template <class T> void ydot::eq_def(T& lhs, T* x, T* u, T& t, T* s, T* p)
{
    lhs = v * sin(theta);
}
template <class T> void thetadot::eq_def(T& lhs, T* x, T* u, T& t, T* s, T* p)
{
    lhs = u2;
}
template <class T> void g1::eq_def(T& lhs, T* x, T* u, T& t, T* s, T* p)
{
    lhs = pow(((x-Rx1)/Ra1),Rp1)+(((y-Ry1)/Rb1),Rp1)-pow(Rr1,Rp1);
}

template <class T> void g2::eq_def(T& lhs, T* x, T* u, T& t, T* s, T* p)
{ 
    lhs = pow(((x-Rx2)/Ra2),Rp2)+(((y-Ry2)/Rb2),Rp2)-pow(Rr2,Rp2);
}
template <class T> void g3::eq_def(T& lhs, T* x, T* u, T& t, T* s, T* p)
{
    lhs = pow(((x-Rx3)/Ra3),Rp1)+(((y-Ry3)/Rb3),Rp3)-pow(Rr3,Rp3);
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
{eq_def(lhs,x,u,t,s,p);}


void xdot::eval_eq(double& lhs, double* x, double* u, double& t, double* s, double* p)
{eq_def(lhs,x,u,t,s,p);}
void xdot::eval_eq(HyperDual& lhs, HyperDual* x, HyperDual* u, HyperDual& t, HyperDual* s,
                   HyperDual* p)
{eq_def(lhs,x,u,t,s,p);}
void xdot::eval_eq(Bicomplex& lhs, Bicomplex* x, Bicomplex* u, Bicomplex& t, Bicomplex* s,
                   Bicomplex* p)
{eq_def(lhs,x,u,t,s,p);}
void ydot::eval_eq(double& lhs, double* x, double* u, double& t, double* s, double* p)
{eq_def(lhs,x,u,t,s,p);}
void ydot::eval_eq(HyperDual& lhs, HyperDual* x, HyperDual* u, HyperDual& t,
                    HyperDual* s, HyperDual* p)
{eq_def(lhs,x,u,t,s,p);}
void ydot::eval_eq(Bicomplex& lhs, Bicomplex* x, Bicomplex* u, Bicomplex& t,
                    Bicomplex* s, Bicomplex* p)
{eq_def(lhs,x,u,t,s,p);}


void thetadot::eval_eq(double& lhs, double* x, double* u, double& t, double* s, double* p)
{eq_def(lhs,x,u,t,s,p);}
void thetadot::eval_eq(HyperDual& lhs, HyperDual* x, HyperDual* u, HyperDual& t,
                    HyperDual* s, HyperDual* p)
{eq_def(lhs,x,u,t,s,p);}
void thetadot::eval_eq(Bicomplex& lhs, Bicomplex* x, Bicomplex* u, Bicomplex& t,
                    Bicomplex* s, Bicomplex* p)
{eq_def(lhs,x,u,t,s,p);}


void g1::eval_eq(double& lhs, double* x, double* u, double& t, double* s, double* p)
{eq_def(lhs,x,u,t,s,p);}
void g1::eval_eq(HyperDual& lhs, HyperDual* x, HyperDual* u, HyperDual& t,
                    HyperDual* s, HyperDual* p)
{eq_def(lhs,x,u,t,s,p);}
void g1::eval_eq(Bicomplex& lhs, Bicomplex* x, Bicomplex* u, Bicomplex& t,
                    Bicomplex* s, Bicomplex* p)
{eq_def(lhs,x,u,t,s,p);}

void g2::eval_eq(double& lhs, double* x, double* u, double& t, double* s, double* p)
{eq_def(lhs,x,u,t,s,p);}
void g2::eval_eq(HyperDual& lhs, HyperDual* x, HyperDual* u, HyperDual& t,
                    HyperDual* s, HyperDual* p)
{eq_def(lhs,x,u,t,s,p);}
void g2::eval_eq(Bicomplex& lhs, Bicomplex* x, Bicomplex* u, Bicomplex& t,
                    Bicomplex* s, Bicomplex* p)
{eq_def(lhs,x,u,t,s,p);}


void g3::eval_eq(double& lhs, double* x, double* u, double& t, double* s, double* p)
{eq_def(lhs,x,u,t,s,p);}
void g3::eval_eq(HyperDual& lhs, HyperDual* x, HyperDual* u, HyperDual& t,
                    HyperDual* s, HyperDual* p)
{eq_def(lhs,x,u,t,s,p);}
void g3::eval_eq(Bicomplex& lhs, Bicomplex* x, Bicomplex* u, Bicomplex& t,
                    Bicomplex* s, Bicomplex* p)
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


