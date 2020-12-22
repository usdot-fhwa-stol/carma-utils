
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

#include "sigpack.h"
using namespace arma;
using namespace sp;

int main()
{
    PNM ppm;       // Portable Pixel Map
    
    int R=120,C=160;
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


