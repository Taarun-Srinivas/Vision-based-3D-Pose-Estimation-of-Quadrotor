function [position, orientation] = estimatePose(data, t)
%% CHANGE THE NAME OF THE FUNCTION TO estimatePose
% Please note that the coordinates for each corner of each AprilTag are
% defined in the world frame, as per the information provided in the
% handout. Ideally a call to the function getCorner with ids of all the
% detected AprilTags should be made. This function should return the X and
% Y coordinate of each corner, or each corner and the centre, of all the
% detected AprilTags in the image. You can implement that anyway you want
% as long as the correct output is received. A call to that function
% should made from this function.
    %% Input Parameter Defination
    % data = the entire data loaded in the current dataset
    % t = index of the current data in the dataset
    
    %% Output Parameter Defination
    
    % position = translation vector representing the position of the
    % drone(body) in the world frame in the current time, in the order ZYX
    
    % orientation = euler angles representing the orientation of the
    % drone(body) in the world frame in the current time, in the order ZYX
    
    tags = data(t).id;
    length(tags);

    p0 = data(t).p0;
    p1 = data(t).p1;
    p2 = data(t).p2;
    p3 = data(t).p3;
    p4 = data(t).p4;

    A = zeros(length(tags)*8,9);
    k = 1;

    K = [311.0520         0 201.8724;
                 0 311.3885 113.6210;
                 0        0        1];

    for i = 1:length(tags)
        res = getCorner(tags(i));
        p0x = p0(1,i);
        p0y = p0(2,i);
        p1x = p1(1,i);
        p1y = p1(2,i); 
        p2x = p2(1,i);
        p2y = p2(2,i);
        p3x = p3(1,i);
        p3y = p3(2,i);
        p4x = p4(1,i);
        p4y = p4(2,i);

        wx1 = res(1,2);
        wy1 = res(2,2);
        wx2 = res(1,3);
        wy2 = res(2,3);
        wx3 = res(1,4);
        wy3 = res(2,4);
        wx4 = res(1,5);
        wy4 = res(2,5);

        a = [wx1 wy1 1   0   0 0 -p1x*wx1 -p1x*wy1 -p1x;
               0   0 0 wx1 wy1 1 -p1y*wx1 -p1y*wy1 -p1y;
             wx2 wy2 1   0   0 0 -p2x*wx2 -p2x*wy2 -p2x;
               0   0 0 wx2 wy2 1 -p2y*wx2 -p2y*wy2 -p2y;
             wx3 wy3 1   0   0 0 -p3x*wx3 -p3x*wy3 -p3x;
               0   0 0 wx3 wy3 1 -p3y*wx3 -p3y*wy3 -p3y;
             wx4 wy4 1   0   0 0 -p4x*wx4 -p4x*wy4 -p4x;
               0   0 0 wx4 wy4 1 -p4y*wx4 -p4y*wy4 -p4y];

        

        A(k:i*8,:) = a;
        k = (i*8)+1;
    end

    A;
    size(A);

    [~, ~, V] = svd(A);
%     V = V';
    H = V(:,end)*sign(V(9,9));
    H = reshape(H,3,3);
    H = H';

   
    r = inv(K) * H;

    T =  r(:,3)/norm(r(:,1));
    R1 = r(:,1)/norm(r(:,1));
    R2 = r(:,2)/norm(r(:,2));

    norm_r = [R1 R2 cross(R1,R2)];

    [um, ~, vm] = svd(norm_r);

    S = [ 1 0                     0;
          0 1                     0;
          0 0 det(um*transpose(vm))];

    final_R = um * S * transpose(vm);
%     orientation = rotm2eul(final_R,'ZYX');
%     position = T;

    homo1 = [final_R T;
               0 0 0 1];

    rot_trans = rotz(-45,'deg') * rotx(180,'deg');
    homo2 = [rot_trans transpose([-0.04, 0.0, -0.03]);
                 0 0 0 1];

    final_H = inv(homo1)* homo2;

    orientation = [rotm2eul(final_H(1:3,1:3),'ZYX')]';
    position = final_H(1:3,4);
        
    end








        

    
