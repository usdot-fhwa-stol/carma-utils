#include "sigpack.h"
using namespace arma;
using namespace sp;

int main()
{   
	int N = 45191;  // Large prime number
	FFTW ss(N,FFTW_ESTIMATE);
	vec x(N);
	cx_vec Sxx(N);

	vec P(N);
	clock_t tic;
	for (int n = 1; n < 5; n++)
	{
		x.randn();
		tic = clock();
		Sxx = ss.fft(x);
		cout << "FFTW Loop[" << n << "]. Time = " << (clock() - tic) / double(CLOCKS_PER_SEC) << endl;
		x.randn();
		tic = clock();
		Sxx = fft(x);
		cout << " FFT Loop[" << n << "]. Time = " << (clock() - tic) / double(CLOCKS_PER_SEC) << endl;
	}
    return 0;
}
