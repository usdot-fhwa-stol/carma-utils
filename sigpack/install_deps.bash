#!/bin/bash

# Install Armadillo
curl -L  http://sourceforge.net/projects/arma/files/armadillo-9.800.1.tar.xz > armadillo-9.800.1.tar.xz

tar -xvf armadillo-9.800.1.tar.xz
cd armadillo-9.800.1
./configure
make
sudo make install
cd ../
rm -R armadillo-9.800.1 armadillo-9.800.1.tar.xz

# Install FFTW
sudo apt install libfftw3-dev

# Install GNU Plot
sudo apt install gnuplot-qt
