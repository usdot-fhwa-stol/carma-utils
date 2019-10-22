%% Compile the demo as a mex file
if ispc()
    mex -IC:\Users\xxxxx\armadillo-4.320.2\include ...
        -IC:\Users\xxxxx\armadillo-4.320.2\mex_interface ...
        -IC:\Users\xxxxx\Desktop\SigPack\SigPack_work\repo ... % Sigpack directory
        -LC:\Users\xxxxx\armadillo-4.320.2\examples\lib_win64 ...
        -lblas_win64_MT.lib -llapack_win64_MT.lib ...
        -DWIN32 ...
        sp_test_102.cpp
else
    mex -I/home/xxxxx/proj/armadillo-4.320.2/include ...
        -I/home/xxxxx/proj/armadillo-4.320.2/mex_interface ...
        -I/home/xxxxx/proj/sigpack/source ... % Sigpack directory
        -lblas -llapack ...
        sp_test_102.cpp    
end    


%% Demo 
load train.mat
Y = sp_test_102(y);
X = Y(1:size(Y,1)/2,:);      % Cut out positive frequencies

%% Plot spectrogram
figure
subplot(211)
imagesc(20*log10(abs(X)))    % Plot half spectrum
set(gca,'Ydir','normal')     % Restore to normal Yaxis direction
title('Power spectrogram')
subplot(212)
imagesc(angle(X))            % Plot half spectrum
set(gca,'Ydir','normal')     % Restore to normal Yaxis direction
colormap(jet)                % Set colormap
title('Phase spectrogram')
