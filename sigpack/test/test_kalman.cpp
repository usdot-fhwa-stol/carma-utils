#include "sigpack.h"

using namespace std;
using namespace sp;
int main()
{
    // Number of samples
    arma::uword Nsamp = 120;
    arma::uword N = 6;  // Nr of states [X,Y,Vx,Vy,Ax,Ay]
    arma::uword M = 2;  // Nr of measurements
    arma::uword L = 0;  // Nr of inputs

//    // Instatiate a Extended Kalman Filter
//    EKF kalman(N,M,L);
    // Instatiate an Unscented Kalman Filter
    UKF kalman(N,M,L);

    // Initialisation and setup of system
    double P0 = 50;
    double Q0 = 0.001;
    double R0 = 8;

    // Meas interval
    double dT = 0.1;

    // [X,Y,Vx,Vy,Ax,Ay] 2D position, velocity and acceleration as states
    arma::mat x ={0, 5, 10, 50, -0.5, -9.8 };
    arma::inplace_trans(x);
    arma::mat x_init(x*0.5);  // Init state as 50% of actual value
    kalman.set_state_vec(x_init);

    // Set transition function
    arma::mat A =
    {
        {1, 0, dT,  0, dT*dT/2,       0},
        {0, 1,  0, dT,       0, dT*dT/2},
        {0, 0,  1,  0,      dT,       0},
        {0, 0,  0,  1,       0,      dT},
        {0, 0,  0,  0,       1,       0},
        {0, 0,  0,  0,       0,       1}
    };
    kalman.set_trans_mat(A);
//    fcn_t f0 = FCN_XUW{ return x(0)+x(2)*dT+x(4)*dT*dT/2; };
//    fcn_t f1 = FCN_XUW{ return x(1)+x(3)*dT+x(5)*dT*dT/2; };
//    fcn_t f2 = FCN_XUW{ return x(2)+x(4)*dT; };
//    fcn_t f3 = FCN_XUW{ return x(3)+x(5)*dT; };
//    fcn_t f4 = FCN_XUW{ return x(4); };
//    fcn_t f5 = FCN_XUW{ return x(5); };
//    fcn_v  f = {f0,f1,f2,f3,f4,f5};
//    kalman.set_trans_fcn(f);

    // Set process noise
    arma::mat Q = Q0*arma::diagmat(arma::vec("1 1 0.1 0.1 0.01 0.01"));
    kalman.set_proc_noise(Q);

    // Nonlinear measurement function
    fcn_t h0 = FCN_XUW{ return sqrt(x(0)*x(0)+x(1)*x(1)); };
	fcn_t h1 = FCN_XUW{ return atan2(x(1),x(0)); };
    fcn_v  h = {h0,h1};
    kalman.set_meas_fcn(h);

    // Nonlinear measurement jacobian - analytical
//	fcn_t h_zero   = FCN_XUW{ return 0; };
//	fcn_t h00_j    = FCN_XUW{ return x(0)/sqrt(x(0)*x(0)+x(1)*x(1)); };
//	fcn_t h01_j    = FCN_XUW{ return x(1)/sqrt(x(0)*x(0)+x(1)*x(1)); };
//	fcn_t h10_j    = FCN_XUW{ return -x(1)/(x(0)*x(0)+x(1)*x(1)); };
//	fcn_t h11_j    = FCN_XUW{ return x(0)/(x(0)*x(0)+x(1)*x(1)); };
//  fcn_m h_jac =
//  {
//        {h00_j,h01_j,h_zero,h_zero,h_zero,h_zero},
//        {h10_j,h11_j,h_zero,h_zero,h_zero,h_zero}
//  };
//  kalman.set_meas_jac(h_jac);

    // Error cov
    arma::mat P = P0*arma::eye<arma::mat>(N,N);
    kalman.set_err_cov(P);

    // Meas noise  (for distance and angle)
    arma::mat R =
    {
        {1, 0     },
        {0, 0.005 }
    };
    R*=R0;
    kalman.set_meas_noise(R);

   // Create simulation data
    arma::mat  z(M, Nsamp, arma::fill::zeros);
    arma::mat z0(M, Nsamp, arma::fill::zeros);
    for(arma::uword n=0; n< Nsamp; n++)
    {
       arma::mat v0(N,1,arma::fill::zeros);

       // Update state
       x = A*x + Q*arma::randn(N,1);

       // Update measurement
       z0(0,n) =x(0);
       z0(1,n) =x(1);

       // Add meas noise
       z.col(n) = eval_fcn(h,z0.col(n)) + 0.5*R*arma::randn(M,1);
    }

    arma::mat xhat_log(N,Nsamp);
    arma::mat e_log(M,Nsamp);
    arma::cube P_log(N,N,Nsamp);
    arma::mat xs_log(M,Nsamp);
    arma::cube Ps_log(N,N,Nsamp);
    arma::cube K_log(N,M,Nsamp);

    // Kalman filter loop
    for(arma::uword n=0; n<Nsamp; n++)
    {
        // Update
        kalman.update(z.col(n));
        xhat_log.col(n)= kalman.get_state_vec();

        // Predict
        kalman.predict();

        e_log.col(n)   = kalman.get_err();
        P_log.slice(n) = kalman.get_err_cov();
        K_log.slice(n) = kalman.get_kalman_gain();
    }

    // RTS smoother
    kalman.rts_smooth(xhat_log,P_log,xs_log,Ps_log);

    // Display result
    gplot gp0;
    gp0.window("Plot", 10, 10, 500, 500);
    gp0.set_term("qt");
    gp0.plot_add( z0.row(0), z0.row(1),"True Y","lines dashtype 2 linecolor \"black\"");
    gp0.plot_add( arma::mat(z.row(0)%cos(z.row(1))), arma::mat(z.row(0)%sin(z.row(1))),"Meas Y","points");
    gp0.plot_add( xhat_log.row(0), xhat_log.row(1),"Kalman x hat");
    gp0.plot_add(xs_log.row(0),xs_log.row(1),"RTS smooth");
    gp0.plot_show();

    gplot gp1;
    gp1.window("Plot", 800, 10, 500, 500);
    gp1.set_term("qt");
    gp1.plot_add( xhat_log.row(4),"Acc x");
    gp1.plot_add( xhat_log.row(5),"Acc y");
    gp1.plot_show();


    gplot gp2;
    gp2.window("Plot", 800, 10, 500, 500);
    gp2.set_term("qt");

    arma::mat ppp = P_log.tube(0,0);
    gp2.plot_add(  ppp ,"P  x");
    ppp = P_log.tube(1,1);
    gp2.plot_add(  ppp ,"P  y");
    ppp = P_log.tube(2,2);
    gp2.plot_add(  ppp ,"P Vx");
    ppp = P_log.tube(3,3);
    gp2.plot_add(  ppp ,"P Vy");
    ppp = P_log.tube(4,4);
    gp2.plot_add(  ppp ,"P Ax");
    ppp = P_log.tube(5,5);
    gp2.plot_add(  ppp ,"P Ay");
//    arma::mat ppp = K_log.tube(0,0);
//    gp2.plot_add(  ppp ,"K 00");
//    ppp = K_log.tube(0,1);
//    gp2.plot_add(  ppp ,"K 01");
//
//    ppp = K_log.tube(1,0);
//    gp2.plot_add(  ppp ,"K 10");
//    ppp = K_log.tube(1,1);
//    gp2.plot_add(  ppp ,"K 11");
//
//    ppp = K_log.tube(2,0);
//    gp2.plot_add(  ppp ,"K 20");
//    ppp = K_log.tube(2,1);
//    gp2.plot_add(  ppp ,"K 21");
//
//    ppp = K_log.tube(3,0);
//    gp2.plot_add(  ppp ,"K 30");
//    ppp = K_log.tube(3,1);
//    gp2.plot_add(  ppp ,"K 31");
//
//    ppp = K_log.tube(4,0);
//    gp2.plot_add(  ppp ,"K 40");
//    ppp = K_log.tube(4,1);
//    gp2.plot_add(  ppp ,"K 41");
//
//    ppp = K_log.tube(5,0);
//    gp2.plot_add(  ppp ,"K 50");
//    ppp = K_log.tube(5,1);
//    gp2.plot_add(  ppp ,"K 51");

    gp2.plot_show();

    return 1;
}
