#include "sigpack.h"

using namespace std;
using namespace arma;
using namespace sp;

int main()
{
    int N = 512;
    
		
    gplot gp0;
    gp0.window("Plot1", 10,  10,  500, 520);

    //========================================
    gp0.title("Hamming");
    gp0.xlim(0,511);
    gp0.ylim(-0.2,1);    
    gp0.plot_add(hamming(N),"Hamming");
    gp0.plot_show();
    cin.ignore();  

    //========================================
    gp0.title("Hann");
    gp0.plot_add(hann(N),"Hann");
    gp0.plot_show();
    cin.ignore();  
   
    //========================================
    gp0.title("Blackman");
    gp0.plot_add(blackman(N),"Blackman");
    gp0.plot_show();
    cin.ignore();  

    //========================================
    gp0.title("Blackman-Harris");
    gp0.plot_add(blackmanharris(N),"BH-4");
    gp0.plot_show();
    cin.ignore();  

    //========================================
    gp0.title("Flat top Win");
    gp0.plot_add(flattopwin(N),"Flattop");
    gp0.plot_show();
    cin.ignore();  

    //========================================
    gp0.title("Hanning");
    gp0.plot_add(hanning(N),"Hanning");
    gp0.plot_show();
    cin.ignore();  

    //========================================
    gp0.title("Kaiser, beta=5");
    gp0.plot_add(kaiser(N,5),"Kaiser");
    gp0.plot_show();
    cin.ignore();  

    //========================================
    gp0.title("Triangle");
    gp0.plot_add(triang(N),"Triangle");
    gp0.plot_show();
    cin.ignore();  

    gp0.close_window();

    return 1;
}

