
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
    gplot gp;
    gp.window(0,"Filter plots",   10,10,700,700);

    vec Mg(1024);
    vec Ph(1024);
    vec a,b;
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

    return 0;
}
