#include <sigpack.h>
using namespace arma;
using namespace sp;

int main()
{
    int r=480, c=640;
	mat x(r,c);
	cx_mat X(r,c);
	clock_t tic;
	FFTW X_2d(r,c, FFTW_MEASURE);

	x.randn();

	#if 1

//	  X_2d.export_wisdom_fft("wisd_fft.txt");
//	  X_2d.import_wisdom_file("wisd_fft.txt");

	  std::string str_fft = "\
      (fftw-3.3.5 fftw_wisdom #x4be12fff #x7b2df9b2 #xa5975329 #x385b0041 \
      (fftw_codelet_hc2cf_32 0 #x11048 #x11048 #x0 #x882273a3 #x56a533d6 #xc6c3347e #xd190e241) \
      (fftw_dft_vrank_geq1_register 0 #x10048 #x10048 #x0 #x099817e7 #xc596b28c #xe506412c #x581cf2d0) \
      (fftw_rdft2_rank_geq2_register 0 #x11048 #x11048 #x0 #xed40bad3 #x93c23437 #xaf3711ed #x603ee510) \
      (fftw_rdft2_vrank_geq1_register 0 #x11048 #x11048 #x0 #x1a6357c6 #x413f903b #x74fe70f1 #xfccb5c54) \
      (fftw_rdft_rank0_register 3 #x11048 #x11048 #x0 #x02d4eca5 #x884a04c6 #x3f0ad214 #xda717200) \
      (fftw_dft_buffered_register 0 #x11048 #x11048 #x0 #x6196ea82 #x099f9b85 #x08198834 #xe7593275) \
      (fftw_codelet_t1_10 0 #x10048 #x10048 #x0 #xba1708ce #x154e0e87 #x86aebd39 #xcb9e43fe) \
      (fftw_rdft_vrank_geq1_register 1 #x11048 #x11048 #x0 #x081f12db #xc5b1275f #x5907e52d #x646a8914) \
      (fftw_dft_r2hc_register 0 #x11048 #x11048 #x0 #x514942d9 #xe0534dce #x7d1dcd63 #xde881c5a) \
      (fftw_codelet_n1_64 0 #x10048 #x10048 #x0 #x9dcd7b03 #xdf2d4251 #x0230d342 #xde1fe96d) \
      (fftw_dft_r2hc_register 0 #x11048 #x11048 #x0 #x43917e3e #x5c1ab09e #xdc26854f #x352833a8) \
      (fftw_codelet_r2cf_15 0 #x11048 #x11048 #x0 #x251274f4 #x219f0dbb #x0c38f4e4 #xf19e1c79) \
      (fftw_dft_nop_register 0 #x11048 #x11048 #x0 #xc1cc28d6 #x5c67e01b #x280816eb #x7ee0ce02) \
      (fftw_codelet_r2cf_32 2 #x11048 #x11048 #x0 #xc267a0b3 #xc28de71d #x550f0573 #x959c40e7) \
      (fftw_codelet_n1_64 0 #x10048 #x10048 #x0 #x101f223c #xcff4353b #xd4a553bb #xe3ff9319) \
      (fftw_dft_buffered_register 0 #x11048 #x11048 #x0 #x9973ff81 #x0b0fdaf1 #xfd5496b5 #xfe9c4fba) \
      (fftw_codelet_t1_10 0 #x10048 #x10048 #x0 #x47334bf5 #x898f072c #x86c99502 #xe82acf7f) \
      (fftw_rdft_rank0_register 2 #x11048 #x11048 #x0 #x5c2cc86a #x3d86b0c3 #xe3aeadc0 #xdc7e28da) \
      (fftw_rdft2_nop_register 0 #x11048 #x11048 #x0 #xf80d9535 #x1b15b669 #x8ee1f97f #x42fa82cd))";

	  X_2d.import_wisdom_string(str_fft);

	  tic = clock();
	  X = fft2(x);
	  cout << "FFT Time using arma::fft2()= " << (clock() - tic) / double(CLOCKS_PER_SEC) << endl;
	  tic = clock();
	  X = X_2d.fft2(x);
	  cout << "FFTW Time using Wisdom= " << (clock() - tic) / double(CLOCKS_PER_SEC) << endl;

	#else
	  tic = clock();
	  X = fft2(x);
	  cout << "FFT Time using arma::fft2()= " << (clock() - tic) / double(CLOCKS_PER_SEC) << endl;
	  tic = clock();
	  X = X_2d.fft2(x);
	  cout << "FFTW Time without Wisdom= " << (clock() - tic) / double(CLOCKS_PER_SEC) << endl;
	#endif

	return 0;
}

