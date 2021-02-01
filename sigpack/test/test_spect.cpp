#include "sigpack.h"
using namespace arma;
using namespace sp;

int main()
{
    int N=25000,NFFT=512;
    vec x(N),f(N),t(N),n(N),S(NFFT);
    cx_vec cx(N);
    
    gplot gp0,gp1,gp2;
    gp0.window("Spectrum",   10,10,500,400);
    gp1.window("Time",       550,10,900,400);
    gp2.window("Spectrogram",10,450,1440,500); 
  
    // Create two tone sig
    t = linspace(0,N-1,N);
    complex<double> j(0,1);
    cx = exp(j*(datum::pi*0.1)*t)+0.8*exp(j*(datum::pi*0.2)*t)+0.01*n.randn();

    // Calc spectrum
    S = 10*log10(pwelch(cx,NFFT,NFFT/2));

    gp0.send2gp("set xtics (\"-Fs/2\" 1,\"0\" 256,\"Fs/2\" 512)");  // Direct call to gnuplot pipe
    gp0.ylabel("Power");gp0.xlabel("Frequency");
    vec SS = fftshift(S);
    gp0.plot_add(SS,"TestSig"); 
    gp0.plot_show(); 

    gp1.ylabel("Amplitude");gp1.xlabel("Time");
    vec tt = t.head(200);
    vec xx = real(cx.head(200));
    gp1.set_parula_line();
    gp1.plot_add(tt,xx,"TestSig");
    gp1.plot_show(); 

    // Create sig - chirp plus noise
    f = 0.01+0.45*t/double(N);
    x = sin((datum::pi*f)%t)+0.01*n.randn();

    // Calc spectrogram
    mat P = 10*log10(abs(specgram(x,NFFT,NFFT/4)));

    // Plot
    gp2.ylabel("Frequency");gp2.xlabel("Time");
    gp2.set_jet_palette();
    gp2.image(P);
    
    return 0;
}
    
