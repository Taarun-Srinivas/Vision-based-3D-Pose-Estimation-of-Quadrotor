function res = getCorner(id)
%% CHANGE THE NAME OF THE FUNCTION TO getCorner
    %% Input Parameter Description
    % id = List of all the AprilTag ids detected in the current image(data)
    
    %% Output Parameter Description
    % res = List of the coordinates of the 4 corners (or 4 corners and the
    % centre) of each detected AprilTag in the image in a systematic method

    tl = 0.152;
s = 0.152;


    val = id;
    col = floor(val/12); 
    row = mod(val,12); 
    

    %Top X
    p4x = row*tl + row*s;
    p3x = row*tl + row*s;

    %Bottom X
    p1x = (row+1)*tl + row*s;
    p2x = (row+1)*tl + row*s;

    %Left Y
    p4y = col*tl + col*s;
    p1y = col*tl + col*s;

    %Right Y
    p3y = (col+1)*tl + col*s;
    p2y = (col+1)*tl + col*s;


    if (col == 3 || col == 4 || col == 5)
        %Left Y
        p4y = col*tl + col*s + (0.178-0.152);
        p1y = col*tl + col*s + (0.178-0.152);
    
        %Right Y
        p3y = (col+1)*tl + col*s + (0.178-0.152);
        p2y = (col+1)*tl + col*s + (0.178-0.152);

    elseif (col == 6 || col == 7 || col == 8)
        %Left Y
        p4y = col*tl + col*s + 2*(0.178-0.152);
        p1y = col*tl + col*s + 2*(0.178-0.152);
    
        %Right Y
        p3y = (col+1)*tl + col*s + 2*(0.178-0.152);
        p2y = (col+1)*tl + col*s + 2*(0.178-0.152);
    end

    p0x = (p1x + p3x)/2;
    p0y = (p1y + p3y)/2;

    p0 = [p0x p0y]';
    p1 = [p1x p1y]';
    p2 = [p2x p2y]';
    p3 = [p3x p3y]';
    p4 = [p4x p4y]';

    res = [p0 p1 p2 p3 p4];

end