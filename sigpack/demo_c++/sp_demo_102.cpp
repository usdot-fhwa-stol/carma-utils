
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
    int N=25000,NFFT=512;
    vec x(N),f(N),t(N),n(N),S(NFFT);

#if defined(WIN32)
    gplot gp;
    gp.window(0,"Spectrum",   10,10,500,400);
    gp.window(1,"Time",       550,10,900,400);
    gp.window(2,"Spectrogram",10,450,1440,500);
#elif defined(unix)
    gplot gp0,gp1,gp2;
    gp0.window("Spectrum",   10,10,500,400);
    gp1.window("Time",       550,10,900,400);
    gp2.window("Spectrogram",10,450,1440,500);
#endif

    // Create sig - chirp plus noise
    t = linspace(0,N-1,N);
    f = 0.01+0.45*t/double(N);
    x = sin((datum::pi*f)%t)+0.01*n.randn();

    // Calc spectrum
    S = 10*log10(pwelch(x,NFFT,NFFT/2));

    // Calc spectrogram
    mat P = 10*log10(abs(specgram(x,NFFT,NFFT/4)));

    x.resize(2000);
    t.resize(2000);
    
    // Plot
#if defined(WIN32)
    gp.figure(0);
    gp.send2gp("set xtics (\"-Fs/2\" 1,\"0\" 256,\"Fs/2\" 512)");  // Direct call to gnuplot pipe
    gp.ylabel("Power");gp.xlabel("Frequency");
    gp.plot_add(S,"TestSig");
    gp.plot_show();

    gp.figure(1);
    gp.ylabel("Amplitude");gp.xlabel("Time");
    gp.plot_add(t,x,"TestSig");
    gp.plot_show();

    gp.figure(2);
    gp.ylabel("Frequency");gp.xlabel("Time");
    gp.image(P);
#elif defined(unix)
    gp0.send2gp("set xtics (\"-Fs/2\" 1,\"0\" 256,\"Fs/2\" 512)");  // Direct call to gnuplot pipe
    gp0.ylabel("Power");gp0.xlabel("Frequency");
    gp0.plot_add(S,"TestSig");
    gp0.plot_show();

    gp1.ylabel("Amplitude");gp1.xlabel("Time");
    gp1.plot_add(t,x,"TestSig");
    gp1.plot_show();

    gp2.ylabel("Frequency");gp2.xlabel("Time");
    gp2.image(P);
#endif

    return 0;
}
