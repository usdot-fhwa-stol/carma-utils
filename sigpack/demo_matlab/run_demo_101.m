%% Compile the demo as a mex file
if ispc()
    mex -IC:\Users\xxxxx\armadillo-4.320.2\include ...
        -IC:\Users\xxxxx\armadillo-4.320.2\mex_interface ...
        -IC:\Users\xxxxx\Desktop\SigPack\SigPack_work\repo ... % Sigpack directory
        -LC:\Users\xxxxx\armadillo-4.320.2\examples\lib_win64 ...
        -lblas_win64_MT.lib -llapack_win64_MT.lib ...
        -DWIN32 ...
        sp_test_101.cpp
else
    mex -I/home/xxxxx/proj/armadillo-4.320.2/include ...
        -I/home/xxxxx/proj/armadillo-4.320.2/mex_interface ...
        -I/home/xxxxx/proj/sigpack/source ... % Sigpack directory
        -lblas -llapack ...
        sp_test_101.cpp    
end    

%% Demo 
% Generate random signal
X = 0.5+randn(10000,1);

% Run the demo using X
Y = sp_test_101(X);

% Do the same thing using matlab
Y2 = filter(fir1(17,0.25),1,X);

%% Plot spectrum
figure
P2 = pwelch(Y2,512,256,'twosided','power');
plot(10*log10(abs(Y)),'b-o')
% plot(Y,'b-o')
hold on
plot(10*log10((P2)),'r')
legend('SigPack','Matlab')
zoom on;
