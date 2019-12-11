CXX = g++
CXX_FLAGS = -Wall -ggdb -std=c++11 -DHAVE_FFTW -I../sigpack -larmadillo -lfftw3

%: %.cpp
	@echo '+++++++++++++++++++++++++++++++++++++++++++++'
	@echo 'Compiling $<, making target ./$@ ...'
	@echo '+++++++++++++++++++++++++++++++++++++++++++++'
	$(CXX) $< $(CXX_FLAGS) -o $@
	
	 	
	
	
