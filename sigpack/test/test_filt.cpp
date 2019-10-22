#include "sigpack.h"
using namespace std;
using namespace arma;
using namespace sp;

int main()
{
    vec b;
	int N = 15;
	vec X(N,fill::zeros);  // Input sig
	vec Y(N,fill::zeros);  // Output sig

	Delay<double> del;

	// Create a FIR filter
	FIR_filt<double,double,double> fir_filt;
	b = fir1(7,0.35);
	fir_filt.set_coeffs(b);
	X[0] = 1;  // Impulse

	del.set_delay(3);
	// Filter - sample loop
	for(int n=0;n<N;n++)
	{
		Y[n] = fir_filt(X[n]); 
	}

	std::cout << "Filter coeffs: \n" << b.t() << std::endl;
	std::cout << "Impulse response sample:\n" << Y.t() << std::endl;
	Y = fir_filt.filter(X);
    std::cout << "Impulse response vector:\n" << Y.t() << std::endl;


    IIR_filt<double,double,double> iir_filt;
	vec a;
	b << 0.25 << 0.5 << 0.25 << endr;
	a << 1.0 << -0.9 << endr;

	iir_filt.set_coeffs(b,a);
	Y = iir_filt.filter(X);
	std::cout << "IIR theor. impulse response: \n   0.2500   0.7250   0.9025   0.8123   0.7310   0.6579   0.5921   0.5329   0.4796   0.4317   0.3885 ..." << std::endl;
	std::cout << "IIR output: \n" << Y.t() << std::endl;
	std::cout << "Delayed (3) output: \n"  << del.delay(Y).t() << std::endl;

    gplot gp;
    gp.window(0,"Filter plots",   10,10,700,700);

    vec Mg(1024);
    vec Ph(1024);
    
    a << 1 << endr;
    b = fir1(44,0.25);
    Mg = 20*log10(freqz(b,a,1024));
    Ph = phasez(b,a,1024);

    // Plot
    gp.figure(0);
    gp.grid_on();
    gp.send2gp("set multiplot layout 2, 1");
    gp.send2gp("set xtics (\"0\" 1,\"0.5\" 512,\"1\" 1024)");  // Direct call to gnuplot pipe
    gp.ylabel("Magnitude [dB]");gp.xlabel("Frequency [f/Fs]");
    gp.plot_add(Mg,"Filter");
    gp.plot_show();

    gp.send2gp("set xtics (\"0\" 1,\"0.5\" 512,\"1\" 1024)");  // Direct call to gnuplot pipe
    gp.ylabel("Phase [rad]");gp.xlabel("Frequency [f/Fs]");
    gp.plot_add(Ph,"Filter");
    gp.plot_show();
    gp.send2gp("unset multiplot");

    return 1;
}


