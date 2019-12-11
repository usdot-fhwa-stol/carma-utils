#include "sigpack.h"

using namespace std;
using namespace sp;

int main()
{
    // Number of samples
    arma::uword Nsamp = 120;
    arma::uword N = 6;  // Nr of states
    arma::uword M = 2;  // Nr of measurements
    arma::uword L = 1;  // Nr of inputs

    // Instatiate a Kalman Filter
    KF kalman(N,M,L);

    // Initialisation and setup of system
    double P0 = 100;
    double Q0 = 0.00003;
    double R0 = 25;

    // Meas interval
    double dT = 0.1;

    arma::mat x ={0, 10, 1, 50, -0.08, -9 };
    kalman.set_state_vec(0.9*x.t());
    // [X,Y,Vx,Vy,Ax,Ay] use position, velocity and acceleration as states
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

    arma::mat H =
    {
        {1, 0, 0, 0, 0, 0 },
        {0, 1, 0, 0, 0, 0}
    };
    kalman.set_meas_mat(H);

    arma::mat P = P0*arma::eye(N,N);
    kalman.set_err_cov(P);
    arma::mat Q = arma::zeros(N,N);
    Q(N-2,N-2) = 1;
    Q(N-1,N-1) = 1;
    Q = Q0*Q;
    kalman.set_proc_noise(Q);

    arma::mat R = R0*arma::eye(M,M);
    kalman.set_meas_noise(R);


    // Create simulation data
    arma::mat  z(M,Nsamp,arma::fill::zeros);
    arma::mat z0(M,Nsamp,arma::fill::zeros);

    arma::mat xx(N,1,arma::fill::zeros);
    xx = x.t();
    for(arma::uword n=1; n< Nsamp; n++)
    {
       xx = A*xx+0.1*Q*arma::randn(N,1);
       z0.col(n)    = H*xx;//
    }

    z.row(0) = z0.row(0)+ 0.001*R0*arma::randn(1,Nsamp);
    z.row(1) = z0.row(1)+ 0.8*R0*arma::randn(1,Nsamp);

    arma::mat x_log(N,Nsamp);
    arma::mat e_log(M,Nsamp);
    arma::cube P_log(N,N,Nsamp);
    arma::mat xs_log(M,Nsamp);
    arma::cube Ps_log(N,N,Nsamp);


    // Kalman filter loop
    for(arma::uword n=0; n< Nsamp; n++)
    {
        kalman.predict();
        kalman.update(z.col(n));

        x_log.col(n) = kalman.get_state_vec();
        e_log.col(n) = kalman.get_err();
        P_log.slice(n) = kalman.get_err_cov();
    }

    // RTS smoother
    kalman.rts_smooth(x_log,P_log,xs_log,Ps_log);

    // Display result
    gplot gp0;
    gp0.window("Plot", 10, 10, 500, 500);
    gp0.set_term("qt");
    gp0.plot_add(    z0.row(0),    z0.row(1),"True Y","lines dashtype 2 linecolor \"black\"");
    gp0.plot_add(     z.row(0),     z.row(1),"Meas Y","points");
    gp0.plot_add( x_log.row(0), x_log.row(1),"Kalman");
    gp0.plot_add(xs_log.row(0),xs_log.row(1),"RTS smooth");
    gp0.plot_show();


    return 1;
}
