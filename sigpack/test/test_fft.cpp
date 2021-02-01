#include <sigpack.h>
using namespace arma;
using namespace sp;

int main()
{
	unsigned int N = 45191;
	vec a(N);
	cx_vec cx1(N);
	clock_t tic;
	FFTW A(N);
//	FFTW A(N,FFTW_PATIENT);
//	A.export_wisdom_fft("wisd_fft.txt");
//	A.import_wisdom_file("wisd_fft.txt");
	std::string str_fft = "\
	(fftw-3.3.5 fftw_wisdom #x4be12fff #x7b2df9b2 #xa5975329 #x385b0041 \
    (fftw_codelet_r2cf_15 0 #x10048 #x10048 #x0 #x3eea495f #xf8b6fa08 #xcbca0fcf #x35e42336) \
    (fftw_rdft_vrank_geq1_register 0 #x10048 #x10048 #x0 #x227134d0 #xa844b68a #x2ec28df7 #xdef0d956) \
    (fftw_codelet_hf_6 0 #x10048 #x10048 #x0 #xcb8b4355 #xdc488294 #x06231846 #x058ff89b) \
    (fftw_dht_rader_register 1 #x1040 #x11048 #x0 #x195ac4d0 #xa2a7c6db #xd6bd7b6f #x54472891) \
    (fftw_codelet_r2cfII_6 0 #x10048 #x10048 #x0 #xa9653873 #x07d08e07 #x1924bdd8 #xdbcc61d4) \
    (fftw_codelet_r2cf_16 0 #x10048 #x10048 #x0 #x5c3a53c9 #x36cc7c82 #x81de7975 #xa2cdbc7c) \
    (fftw_rdft_rank0_register 0 #x11048 #x11048 #x0 #x5a4086db #x9d962620 #xcd2e506a #xe2fb515a) \
    (fftw_codelet_r2cfII_16 0 #x10048 #x10048 #x0 #xe5cd5c92 #x4bf7fb06 #x2cf38cc0 #x991d9a75) \
    (fftw_codelet_r2cf_6 0 #x10048 #x10048 #x0 #x3fe43bff #xa14d2000 #xce933746 #x28825b6e) \
    (fftw_rdft_dht_register 0 #x1040 #x11048 #x0 #x1fa04525 #x5b6aea21 #x16f739bc #x732a39ea) \
    (fftw_codelet_hf_16 0 #x10048 #x10048 #x0 #x3673f1df #xaf302d9c #xed683a6b #xe265494f) \
    (fftw_codelet_r2cf_64 0 #x10048 #x10048 #x0 #x462dc8f6 #x0561dee9 #x70661096 #xfc9fae16) \
    (fftw_rdft_vrank_geq1_register 0 #x10048 #x10048 #x0 #x7270e575 #x368eadc4 #xe64d3144 #x405826cf) \
    (fftw_rdft_nop_register 0 #x11048 #x11048 #x0 #x2678942a #xeb486c20 #xe13bf19b #xfa66a230) \
    (fftw_rdft2_rdft_register 0 #x1040 #x11048 #x0 #xee349610 #x1f1d1cdb #xe2741ba7 #xb1984e38) \
    (fftw_rdft_buffered_register 0 #x1048 #x11048 #x0 #x95be10f1 #x49bd643a #xd93386dc #x009da5ac) \
    (fftw_rdft2_nop_register 0 #x11048 #x11048 #x0 #xf7ca14a7 #x48dda91d #x81f40951 #x53ac82f3) \
    (fftw_codelet_r2cfII_15 0 #x10048 #x10048 #x0 #xcc1b42a9 #x47229488 #x3d52ae94 #x4e4599a7) \
    (fftw_codelet_hf_15 0 #x10048 #x10048 #x0 #x1680efe3 #xf1b3a636 #xfc919cb1 #xb33ac249) \
    )";
	A.import_wisdom_string(str_fft);
	a.randn();
	tic = clock();
	cout << "Calculating, be patient - it can take minutes ..." << endl;
	cx1 = fft(a);
	cout << "ARMA FFT Time = " << (clock() - tic) / double(CLOCKS_PER_SEC) << endl;

	tic = clock();
	cx1 = A.fft(a);
	cout << "SP FFTW Time first pass= " << (clock() - tic) / double(CLOCKS_PER_SEC) << endl;

	tic = clock();
	cx1 = A.fft(a);
	cout << "SP FFTW Time = " << (clock() - tic) / double(CLOCKS_PER_SEC) << endl;
		
		
    PNM ppm;       // Portable Pixel Map
    
    unsigned int R=120,C=160;
    mat x(R,C);
    cx_mat X(R, C);
    mat mag(R, C);
    FFTW ss(R, C, FFTW_ESTIMATE);

    gplot gp0,gp1;
    gp0.window("Image", 10, 10, 2*C+80, 2*R+50);
    gp1.window("FFT Blue channel", 640, 10, 2*C+80+50, 2*R+50 );

    // Generate test image
    cube x3(R,C,3,fill::randu);
    x3*=100;
    x3.slice(0).submat(span(20,51),span(20,51))    = 250*ones(32,32); // Red box
    x3.slice(1).submat(span(80,111),span(70,101))  = 250*ones(32,32); // Green box
    x3.slice(2).submat(span(20,51),span(110,141))  = 250*ones(32,32); // Blue box

    // Write to file
    ppm.write("test.ppm",ppm.PPM_B, x3,"Test picture");

    // Do FFT of blue channel
    x   = x3.slice(2);
    X   = ss.fft2(x);
    mag = 20*log10(abs(fftshift(X)));

    // Plot
    gp0.image(x3);
    gp1.send2gp("set palette grey");
    gp1.image(mag);
	return 0;
}

