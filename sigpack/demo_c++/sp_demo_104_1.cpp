
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
	int N = 45191;  // Large prime number
	FFTW ss(N,FFTW_ESTIMATE);
	vec x(N);
	cx_vec Sxx(N);

	vec P(N);
	clock_t tic;
	for (int n = 1; n < 5; n++)
	{
		x.randn();
		tic = clock();
		Sxx = ss.fft(x);
		cout << "FFTW Loop[" << n << "]. Time = " << (clock() - tic) / double(CLOCKS_PER_SEC) << endl;
		x.randn();
		tic = clock();
		Sxx = fft(x);
		cout << " FFT Loop[" << n << "]. Time = " << (clock() - tic) / double(CLOCKS_PER_SEC) << endl;
	}
    return 0;
}
