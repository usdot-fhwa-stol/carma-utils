# Sigpack

This is a copy of the C++ SigPack header-only library.
The library was obtained through the sourceforge distribution found here https://sourceforge.net/projects/sigpack/
The library is licensed under the Mozilla Public License 2.0 (MPL 2.0) a copy of which is included in the LICENSE file in this folder.
The exception to the MPL 2.0 license is the findFFTW folder which is licensed under BSD 3-Clause License and contains it's own LICENSE file.
The findFFTW folder comes from https://github.com/egpbos/findFFTW and is used in the CMakeList.txt file.
To simplify using the sigpack library in the CARMA system a CMakeList.txt file and package.xml file were added to this folder. In addition the README.txt file was modified to include this leading paragraph and changed to a markdown (.md) file.
At the moment the gnuplot dependency has not been called out in the package.xml as it is not needed for current use cases in CARMA.  

## SigPack Description

=======
SigPack is a C++ signal processing library using the Armadillo library as a base. The API will be 
familiar for those who has used IT++ and Octave/Matlab. The intention is to keep it small and only 
implement the fundamental signal processing algorithms.

## Release notes

=============

## Version     Notes

--------------------------------------------------------------
1.2.7       Bug fix in sp_version(), made it inline.
1.2.6       Improved plot performance for image(), mesh() and surf(). Added fast_plot().
            Removed need for -Dunix flag.
1.2.5       Removed warnings in Visual Studio. Bug fix in PNM class write_header().
            Added flush_buf()/draw_now() in gplot
1.2.4       Updated resampling class, added goertzel and timevec functions.
1.2.3       Updated FIR design functions, support for highpass, bandpass and bandstop
1.2.2       Extended Kalman filter and Unscented Kalman filter class
1.2.1       Updated for Gnuplot 5.0
1.1.2       Kalman tracking/predictor, RTS smoother, new colormaps
1.1.1       Cleanup, added Kalman and Newton adaptive filters
1.0.8       Added adaptive filters - LMS, N-LMS and RLS. New line plot function of matrix data.  
1.0.7       Added support for 2D-FFTW and some image IO functions (read/write of .pbm, .pgm and .ppm) 
1.0.6       Added support for importing/exporting Wisdom plans in FFTW 
1.0.5       Added support for save plot to file in gplot module
1.0.4       Added FFTW class. Updated comments for Doxygen, added unwrap and update_coeffs functions
1.0.3       Added parser class, freqz/phasez functions, error handler.
1.0.2       New file structure, added gnuplot, angle and spectrum functions
1.0.1       Initial commit

## Known Issues

============
Requires Gnuplot > 5.0
The Gnuplot functions worked well under Windows and RedHat distributions, however in Ubuntu you might
have to change the gplot.h file as:
        gnucmd = popen("gnuplot -persist &> /dev/null","w");
to
        gnucmd = popen("gnuplot -persist","w");
Also you need to use the gnuplot-x11 version:
         sudo apt-get install gnuplot-x11

