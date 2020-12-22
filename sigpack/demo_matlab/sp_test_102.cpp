
/*------------------------------------------------------------------------------
* Copyright (C) 2019-2021 LEIDOS.
*
* Licensed under the Apache License, Version 2.0 (the "License"); you may not
* use this file except in compliance with the License. You may obtain a copy of
* the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
* WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
* License for the specific language governing permissions and limitations under
* the License.

------------------------------------------------------------------------------*/

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
    
    // Do your stuff here ...
    cx_mat Pyy = sp::specgram_cx(x,Nfft,Nfft-10);
    
    // Convert back to Matlab
    plhs[0] = armaCreateMxMatrix(Pyy.n_rows, Pyy.n_cols, mxDOUBLE_CLASS, mxCOMPLEX);
    armaSetCx(plhs[0], Pyy);   
    return;
}
