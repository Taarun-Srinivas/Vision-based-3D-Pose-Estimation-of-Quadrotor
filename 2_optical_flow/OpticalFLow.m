%% PROJECT 2 VELOCITY ESTIMATION
close all;
clear all;
clc;
addpath('../data')

%Change this for both dataset 1 and dataset 4. Do not use dataset 9.
datasetNum = 1;

[sampledData, sampledVicon, sampledTime] = init(datasetNum);

PT1 = vision.PointTracker('MaxBidirectionalError',1);

%% INITIALIZE CAMERA MATRIX AND OTHER NEEDED INFORMATION
K = [311.0520        0 201.8724;
            0 311.3885 113.6210;
            0        0        1];

R = rotz(-45,'deg')*rotx(180,'deg');
T = [-0.04 0 -0.03]';



for n = 2:length(sampledData)
    
    H = [];
    VD = [];
    %% Initalize Loop load images
    prev_frame_points = detectFASTFeatures(sampledData(n-1).img);
    initialize(PT1, prev_frame_points.Location, sampledData(n-1).img);
    [curr_frame_points,~] = step(PT1,sampledData(n).img);
    release(PT1);

    prev = prev_frame_points.Location;
    curr = curr_frame_points;
    prev(:,3) = 1;
    curr(:,3) = 1;
    dt = sampledData(n).t - sampledData(n-1).t;
    

    for i = 1 : length(curr) 
        prev(i,:) = transpose(inv(K)*transpose(prev(i,:)));
        curr(i,:) = transpose(inv(K)*transpose(curr(i,:)));
    end

    [pos, ori, R_c2w] = estimatePose(sampledData,n);
    a = R_c2w(:,3);
    b = [0 0 -1]';
    Z = pos(3)/ abs(dot(a,b));

    vel_x = (curr(:,1) - prev(:,1)) / dt;
    vel_y = (curr(:,2) - prev(:,2)) / dt;
    
    for j = 1:length(curr(:,1))
        p = [curr(j,1) curr(j,2)]';

        A = [-1  0 p(1);
              0 -1 p(2)];
        
        B = [ p(1)*p(2)  -(1+p(1)^2)  p(2);
             (1+p(2)^2) -p(1)*p(2) -p(1)];
        
        h = [A/Z B];

        H = vertcat(H,h);
        vd = [vel_x(j) vel_y(j)]';
        VD = vertcat(VD,vd);
    end
    
    pseudo_H = inv(transpose(H)*H)*transpose(H);

    adj1 = [transpose(R_c2w)       zeros(3,3);
                  zeros(3,3) transpose(R_c2w)];

    s = [    0  -T(3)  T(2);
          T(3)      0 -T(1);
         -T(2)   T(1)    0];
    
    adj2 = [    eye(3) -s;
            zeros(3,3)   eye(3)];

    final_adj = adj2*adj1;

    estimatedV(:,n) = final_adj*pseudo_H*VD;
end



plotData(estimatedV, sampledData, sampledVicon, sampledTime, datasetNum)