function [Y_pred,var_pred,diff,binned_data] = FixedRankFilter2D(nmea_file,mcpc_file,startTimesMat,r,numBins,resolution, training1, training2, training1_var, training2_var)
    
    %nmea_file='data/coordinates3.txt'; mcpc_file='data/MCPC_180502_142439.txt'; numBins=[40 50]; r=16; resolution=2;
    %startTimesMat = ['02-May-2018 14:28:09';'02-May-2018 14:35:09';'02-May-2018 14:42:09';'02-May-2018 14:49:09';'02-May-2018 14:56:09'];
    %Load training1 and training2 that is saved from Python script
    %[Y_pred,var_pred,diff,binned_data] = FixedRankFilter2D(nmea_file,mcpc_file,startTimesMat,r,numBins,resolution, training1, training2, training1_var, training2_var);
    
    %Get arrays for x coordinates, y coordinates, and their respective
    %times from nmea data, and get average concentration and their
    %respective times from mcpc data.
    [xcoord, ycoord, time_nmea, aveconc, time_mcpc] = data_reader2D(nmea_file, mcpc_file);
    
    %raw_binned_data = cell array of all position/time/conc points in their
    %respective spatio-temporal bins
    %binned_data = median concentration values for each spatio-temporal bin
    %central_coord = coordinates for the center of each bin
    %combined_data = array of nmea coordinates matched with respective mcpc
    %values
    [~, binned_data, central_coord, combined_data] = binMaker2D(xcoord, ycoord, time_nmea, aveconc, time_mcpc, numBins, startTimesMat);
    
    %if training sets are run through simple KF, replace them with
    %respective binned data
    if exist('training1', 'var')
        training1 = reshape(training1, 1, []);
        training2 = reshape(training2, 1, []);
        binned_data(1, :) = training1;
        binned_data(2, :) = training2;
    end
    %log transform binned_data
    binned_data = log(binned_data);
    %transpose binned_data so time is 3rd dimension
    binned_data = binned_data';
    training1 = training1';
    training2 = training2';
        
    %detrend the first two runs (the training sets). Detrend using all data
    %because we assume that all data will be availble to us for the
    %training runs. 
    allTrends = zeros(size(binned_data,1),size(binned_data,2));
    detrended_data = zeros(size(binned_data, 1), size(binned_data, 2));
    for i=1:2
        [allTrends(:, i), detrended_data(:, i)]=detrend2D(central_coord, binned_data(:, i));
    end

    [S, eta, resolution] = createBasisFunctions2D(reshape(central_coord, numBins(1), [], 2), r, resolution, detrended_data(:, 2));
    
%     training1 = detrended_data(:, 1);
%     training2 = detrended_data(:, 2);
    %remove training sets from binned data
    binned_data = binned_data(:, 3:end);
    
    N = numBins(1) * numBins(2);
    T = size(binned_data, 2);
    
    %qr decomposition of S.
    [Q, R] = qr(S, 0);
    
    %Set signal to noise ratio
    SNR = 100;
    
    %set value of d, the threshold value for the positive definite process.
    dVal = .5;
    
    %change r to the number of basis functions for THIS resolution,
    %instead of the number of basis functions for the 1ST resolution.
    r = sum(arrayfun(@(a) r * 4^(a-1), 1:resolution));
    
    %Compute sigma_1, sigma_2, sigmaLag_2 from training sets. Because the
    %training sets are detrended, the variance can be computed by
    %multiplying it by its transpose and dividing by N.
%     if exist('training1_var', 'var')
%         sigma_1 = diag(reshape(training1_var, 1, [])); 
%         sigma_2 = diag(reshape(training2_var, 1, [])); 
%     else
%         sigma_1 = (training1*training1')/N; 
%         sigma_2 = (training2*training2')/N;
%     end
%     sigmaLag_2 = ((training1*training2')/N)';
    
    t1 = training1 - mean(training1);
    t2 = training2 - mean(training2);
    sigma_1 = (t1 * t1')/N;
    sigma_2 = (t2 * t2')/N;
    sigmaLag_2 = (t1 * t2')/N;
    if exist('training1_var', 'var')
        training1_var = reshape(training1_var, N, 1);
        training2_var = reshape(training2_var, N, 1);
        for i=1:N
           sigma_1(i,i) = training1_var(i);
           sigma_2(i,i) = training2_var(i);
        end
    end
    
    
    %Now that we have sigma_1, we can calculate Kprelim, sigma2_xi and
    %sigma2_eps
    Kprelim = R \ Q' * sigma_1 * Q * (inv(R))';
    sigma2_xi = (sum(diag(S*Kprelim*S'))) / N;
    sigma2_eps = (sum(diag(S*Kprelim*S')) + (N*sigma2_xi)) / (N*SNR);
    
    %Create an identity matrix of size N to multiply by sigma2_xi and
    %sigma2_eps. Then the two can be added to create the diagonal matrix D.
    D = (sigma2_xi * eye(N)) + (sigma2_eps * eye(N));
    Dii = sigma2_xi+sigma2_eps;
    
    %The following process is used to calculate sigma_1_PD
    A = D^(-1/2) * sigma_1 * D^(-1/2) - eye(N);
    %eigendecomposition of sigma_1
    [eigvec, eigval] = eig(A); 
    % Get the eigenvalues in a vector "d"
    d = diag(eigval);
    % Set negative values to small, nonzero value
    d(d <= dVal) = dVal;
    % New PD diagonal matrix 
    eigval_PD = diag(d);
    % reconstruct the new A matrix (A_PD), now positive definite
    A_PD = eigvec * eigval_PD * eigvec';
    % Recalculate sigma_1 so it's positive definite
    sigma_1_PD = (D^(1/2)) * A_PD * (D^(1/2)) + D;
    
    %repeat previous for sigma_2
    A = D^(-1/2) * sigma_2 * D^(-1/2) - eye(N);
    [eigvec, eigval] = eig(A); 
    d = diag(eigval);
    d(d <= dVal) = dVal;
    eigval_PD = diag(d);
    A_PD = eigvec * eigval_PD * eigvec';
    sigma_2_PD = (D^(1/2)) * A_PD * (D^(1/2)) + D;
    
    %calculate K_1, K_2, L_2, H_2, U_2
    K_1 = R \ Q'*(sigma_1_PD-D)*Q*(inv(R))';
    K_2 = R \ Q'*(sigma_2_PD-D)*Q*(inv(R))';
    L_2 = R \ Q'*(sigmaLag_2')*Q*(inv(R))';
    H_2 = L_2 / K_1;
    U_2 = K_2 - (H_2 * L_2);
    
    %Create matrices to keep track of H, U, L, K, sigma for each time
    H_t = zeros([r, r, T+2]);
    U_t = zeros([r, r, T+2]);
    L_t = zeros([r, r, T+2]);
    K_t = zeros([r, r, T+2]);
    sigma_t = zeros([N, N, T+2]);
    sigmaLag_t = zeros([N, N, T+2]);
    
    %insert values we know into the previous matrices
    H_t(:,:,2) = H_2;
    U_t(:,:,2) = U_2;
    L_t(:,:,2) = L_2;
    K_t(:,:,1) = K_1;
    K_t(:,:,2) = K_2;
    sigma_t(:,:,1) = sigma_1_PD;
    sigma_t(:,:,2) = sigma_2_PD;
    sigmaLag_t(:,:,2) = sigmaLag_2;
    
    %Create empty matrices for all the variables we need to keep track of
    %during Kalman Filtering.
    eta_t_t = zeros([r T]);
    eta_t_tp = zeros([r T]);
    P_t_t = zeros([r r T]);
    P_t_tp =zeros([r r T]);
    Y_pred = zeros([N T]);
    var_pred = zeros([N T]);
  
    for t=1:T
        %detrend data for this test
        [allTrends(:, t+2), detrended_data(:, t+2)]=detrend2D(central_coord, binned_data(:, t));
        [St, M, Z_obs, n] = findHoles(S, detrended_data(:, t+2), binned_data(:,t));
        if t==1
            eta_t_tp(:,t) = H_2 * eta;
            P_t_tp(:,:,t) = H_2 * K_2 * H_2' + U_2; %P_0_0 = K_2
        else
            eta_t_tp(:,t) = H_t(:,:,t+1) * eta_t_t(:,t-1);
            P_t_tp(:,:,t) = H_t(:,:,t+1) * P_t_t(:,:,t-1) * H_t(:,:,t+1)' + U_t(:,:,t+1);
        end
        G = P_t_tp(:,:,t)*St'*((1/Dii).*eye(n)-(Dii^(-2)).*St*inv(inv(P_t_tp(:,:,t))+(1/Dii).*St'*St)*St');
        eta_t_t(:,t) = eta_t_tp(:,t) + G * (Z_obs - St * eta_t_tp(:,t));
        P_t_t(:,:,t) = P_t_tp(:,:,t) - G * St * P_t_tp(:,:,t);
        Y_pred(:,t) = S * eta_t_t(:,t) + sigma2_xi .* M *...
            ((1/Dii).*eye(n)-(Dii^(-2)).*St * inv(inv(P_t_tp(:,:,t))+(1/Dii).*St'*St)*St')*...
            (Z_obs- St * eta_t_tp(:,t)); 
        var_pred(:,t)=reshape(diag(S*P_t_tp(:,:,t)*S'+sigma2_xi-(sigma2_xi.*M)*...
                ((1/Dii).*eye(n)-(Dii^(-2)).*St*inv(inv(P_t_tp(:,:,t))+(1/Dii).*St'*St)*St')*(sigma2_xi.*M')...
                -2*S*P_t_tp(:,:,t)*St'*((1/Dii).*eye(n)-(Dii^(-2)).*St*inv(inv(P_t_tp(:,:,t))+(1/Dii).*St'*St)*St')*...
                (sigma2_xi.*M')),N,1);     
        
        Y_pred(:,t) = Y_pred(:,t) + allTrends(:, t+2);
        
        sigma = ((Y_pred(:,t)-mean(Y_pred(:,t)))*(Y_pred(:,t)-mean(Y_pred(:,t)))')/N;
        %sigma = (Y_pred(:,t) * Y_pred(:,t)') / N;        
        %make sigma_t positive definite and add to sigma_t
        A = D^(-1/2) * sigma * D^(-1/2) - eye(N);
        [eigvec, eigval] = eig(A); 
        d = diag(eigval);
        d(d <= dVal) = dVal;
        eigval_PD = diag(d);
        A_PD = eigvec * eigval_PD * eigvec';
        sigma_t(:,:,t+2) = (D^(1/2)) *A_PD * (D^(1/2)) + D;
  
        %calculate sigmaLag from sigma_t and sigma_t-1
        if t==1
            sigmaLag_t(:,:,t+2) = (t2*(Y_pred(:,t)-mean(Y_pred(:,t)))')/N;
        else
            sigmaLag_t(:,:,t+2) = ((Y_pred(:,t-1)-mean(Y_pred(:,t-1)))*(Y_pred(:,t)-mean(Y_pred(:,t)))')/N;
        end
        %sigmaLag_t(:,:,t+2) = (sigma_t(:,:,t+1) * sigma_t(:,:,t+2)') / N;
        %Calculate new values for K,L,H,U,
        K_t(:,:,t+2) = R \ Q' * (sigma_t(:,:,t+2) - D) * Q * (inv(R))';
        L_t(:,:,t+2) = R \ Q' * (sigmaLag_t(:,:,t+2)') * Q * (inv(R))';
        H_t(:,:,t+2) = L_t(:,:,t+2) / K_t(:,:,t+1);
        U_t(:,:,t+2) = K_t(:,:,t+2) - (H_t(:,:,t+2) * L_t(:,:,t+2));
        
        %add the trend back to the data
        %Y_pred(:,t) = Y_pred(:,t) + allTrends(:, t+2);
    end
    
    %reverse log transformation
    Y_pred = exp(Y_pred);
    binned_data = exp(binned_data);
    diff = Y_pred - binned_data;
    
    Y_2d = reshape(Y_pred, numBins(1), numBins(2), []);
    df = reshape(diff, numBins(1), numBins(2), []);
    bd = reshape(binned_data, numBins(1), numBins(2), []);
    var = reshape(var_pred, numBins(1), numBins(2), []);
    cc = reshape(central_coord, numBins(1), numBins(2), size(central_coord, 2));
    fp = '/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/plots/2D/180621/test_map.png';
    xSize = central_coord(end, 1) - central_coord(1, 1);
    ySize = central_coord(end, 2) - central_coord(1, 2);
    img = imread(fp);
    img = imresize(img, [xSize, ySize]);
    startIndices = getStartIndices(combined_data, startTimesMat);
    
    %save('/Users/jaredbrauner/Documents/data/Pollution_Monitoring/PollutionMapping/FRF/dat.mat', 'central_coord', 'Y_pred', 'var_pred')
    
    graph2D(Y_2d, df, bd, var, cc, fp, xSize, ySize, img, central_coord, combined_data, startIndices);
    
end