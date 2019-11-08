// Verified using Matlab 2014b and Microsoft Visual Studio 2010
// Armadillo version 4.320.2

#include "armaMex.hpp"
#include "sigpack.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    // Check input parameters
    int M = mxGetM(prhs[0]);
    int N = mxGetN(prhs[0]);
    if(nrhs != 1)               mexErrMsgTxt("One input required.");
    if(nlhs != 1)               mexErrMsgTxt("One output required.");     
    if( M == 1 &&  N >= 1 )     mexErrMsgTxt("Column vector required.");
    if( mxIsComplex(prhs[0]))   mexErrMsgTxt("Real input data required.");
    
    // Convert to Armadillo
    vec x = conv_to<vec>::from(armaGetPr(prhs[0],true));
    int Nfft = 512;
    vec y(x.size());
    vec Pyy(Nfft);
    
    // Do your stuff here ...
    sp::FIR_filt<double,double,double> fir_filt;
    int K = 17;
    vec b(K);
    
    // Create a FIR filter
    b = sp::fir1(K,0.25);
    fir_filt.set_coeffs(b);
    
    // Filter the signal
    y = fir_filt.filter(x);
    
    // Calculate spectrum
    Pyy = sp::pwelch(y,Nfft,Nfft/2);
    
    // Convert back to Matlab
    plhs[0] = armaCreateMxMatrix(Pyy.size(), 1, mxDOUBLE_CLASS, mxREAL); 
    armaSetPr(plhs[0], conv_to<mat>::from(Pyy));
    return;
}
