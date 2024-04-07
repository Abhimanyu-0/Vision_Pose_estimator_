%% PROJECT 2 VELOCITY ESTIMATION
%close all;
%clear all;
%clc;
addpath('../data')

%Change this for both dataset 1 and dataset 4. Do not use dataset 9.
datasetNum = 4;

[sampledData, sampledVicon, sampledTime] = init(datasetNum);

% Initialize important data

use_RANSAC = 0; % Change the variable to use Ransac

K = [311.0520        0        201.8724;
 
         0         311.3885    113.6210;
 
         0            0           1   ]; % Camera Calibration Matrix

 t = zeros(length(sampledData),1);
    for n = 1:length(sampledData)
         t(n) = sampledData(n).t;
    end
    t = sgolayfilt(t,1,101);   % filtering the time

for n = 2:length(sampledData)
    %% Initalize Loop load images
    current_Img = sampledData(n-1).img; % obtaining image for current frame
    next_Img = sampledData(n).img;   % obtaining image for next frame

    %% Detect good points
    goodPoints_set = detectMinEigenFeatures(current_Img); % keypoints detected
    goodPoints_set = goodPoints_set.selectStrongest(250);  % selected strongest 250
    %% Initalize the tracker to the last frame.
    pointTracker = vision.PointTracker('MaxBidirectionalError',1); % tracker initialized
    
    %% Find the location of the next points;
    initialize(pointTracker,goodPoints_set.Location,current_Img);

    [Points,validity] = pointTracker(next_Img); %Tracker moved to next image
    usable_points = [];
    next_image_points = [];

    for a = 1: length(goodPoints_set)
        matrix1 = inv(K)*[goodPoints_set.Location(a,1);goodPoints_set.Location(a,2); 1];
        usable_points = [usable_points; matrix1(1,1) matrix1(2,1)];

        matrix2 = inv(K)*[Points(a,1);Points(a,2); 1];
        next_image_points = [next_image_points; matrix2(1,1) matrix2(2,1)];
    end 


    %% Calculate velocity
    % Use a for loop
    v = [];
    % Computing optical flow velocity
    for len = 1: length(usable_points)
    
     V = [(next_image_points(len,1)-usable_points(len,1))/(t(n) - t(n-1)), (next_image_points(len,2)-usable_points(len,2))/(t(n) - t(n-1))];
     
     v = [v;V(1,1); V(1,2)]; 

    end
    
    %% Calculate Height
    [position, orientation, R_c2w] = estimatePose(sampledData, n-1);

    T_camera_to_robot = [0.7071  -0.7071  0 0.04;
           -0.7071 -0.7071  0 0;
              0       0     -1 -0.03;
              0       0      0 1];    
 
    rot = eul2rotm(orientation); % rotation of world in camera frame
   
    z = [];

    for c=1:length(usable_points)

        Z = position(3)/(dot([usable_points(c,1);usable_points(c,2);1],-1*R_c2w(3,:)));

        z=[z;Z]; % Depth 
    end   

    %% RANSAC    
    % Write your own RANSAC implementation in the file velocityRANSAC
    if use_RANSAC ==0 
        op =[];
        for i =1:length(usable_points)
             H_ = [-1/z(i) 0 usable_points(i,1)/z(i)  usable_points(i,1)*usable_points(i,2) -(1+usable_points(i,1)^2) usable_points(i,2);
                0 -1/z(i) usable_points(i,2)/z(i) (1+usable_points(i,2)^2) -usable_points(i,1)*usable_points(i,2) -usable_points(i,1)];
                
             op = [op;H_];
        end 

        V_without_ransac = pinv(op)*v; 
        Vel = V_without_ransac;

    end 

    if use_RANSAC ==1
        [Vel] = velocityRANSAC(v,usable_points,z,R_c2w,0.5);
    end 
    %% Thereshold outputs into a range.
    % Not necessary

    %%
    
    
    %% Fix the linear velocity
    % Change the frame of the computed velocity to world frame
    skew_matrix = [0 , -T_camera_to_robot(3,4), T_camera_to_robot(2,4);
                   T_camera_to_robot(3,4), 0 , -T_camera_to_robot(1,4);
                   -T_camera_to_robot(2,4),0 , T_camera_to_robot(2,4) ];
    Vel =[rot zeros(3);zeros(3) rot]*[T_camera_to_robot(1:3,1:3) -T_camera_to_robot(1:3,1:3)*skew_matrix
        ;zeros(3) T_camera_to_robot(1:3,1:3)]*Vel;
    %% ADD SOME LOW PASS FILTER CODE
    % Not neceessary but recommended 
    %estimatedV(:,n) = Vel;
    
    %% STORE THE COMPUTED VELOCITY IN THE VARIABLE estimatedV AS BELOW
    estimatedV(:,n) =Vel; % Feel free to change the variable Vel to anything that you used.
    % Structure of the Vector Vel should be as follows:
    % Vel(1) = Linear Velocity in X
    % Vel(2) = Linear Velocity in Y
    % Vel(3) = Linear Velocity in Z
    % Vel(4) = Angular Velocity in X
    % Vel(5) = Angular Velocity in Y
    % Vel(6) = Angular Velocity in Z
end

% Different cases for applying Savitzky-Golay filtering

if(use_RANSAC ==1 && datasetNum == 1)
estimatedV(1,:) = sgolayfilt(double(estimatedV(1,:)), 1, 15);
estimatedV(2,:) = sgolayfilt(double(estimatedV(2,:)), 1, 17);
estimatedV(3,:) = sgolayfilt(double(estimatedV(3,:)), 1, 13);
estimatedV(4,:) = sgolayfilt(double(estimatedV(4,:)), 1, 3);
estimatedV(5,:) = sgolayfilt(double(estimatedV(5,:)), 1, 3);
estimatedV(6,:) = sgolayfilt(double(estimatedV(6,:)), 1, 3);



elseif(use_RANSAC ==1 && datasetNum == 4)
estimatedV(1,:) = sgolayfilt(double(estimatedV(1,:)), 1, 19);
estimatedV(2,:) = sgolayfilt(double(estimatedV(2,:)), 1, 15);
estimatedV(3,:) = sgolayfilt(double(estimatedV(3,:)), 1, 13);
estimatedV(4,:) = sgolayfilt(double(estimatedV(4,:)), 1, 3);
estimatedV(5,:) = sgolayfilt(double(estimatedV(5,:)), 1, 3);
estimatedV(6,:) = sgolayfilt(double(estimatedV(6,:)), 1, 3);


elseif(use_RANSAC ==0 && datasetNum ==1)
estimatedV(1,:) = sgolayfilt(double(estimatedV(1,:)), 1, 19);
estimatedV(2,:) = sgolayfilt(double(estimatedV(2,:)), 1, 57);
estimatedV(3,:) = sgolayfilt(double(estimatedV(3,:)), 1, 13);
estimatedV(4,:) = sgolayfilt(double(estimatedV(4,:)), 1, 3);
estimatedV(5,:) = sgolayfilt(double(estimatedV(5,:)), 1, 3);
estimatedV(6,:) = sgolayfilt(double(estimatedV(6,:)), 1, 3);

elseif(use_RANSAC ==0 && datasetNum ==4)
estimatedV(1,:) = sgolayfilt(double(estimatedV(1,:)), 1, 19);
estimatedV(2,:) = sgolayfilt(double(estimatedV(2,:)), 1, 15);
estimatedV(3,:) = sgolayfilt(double(estimatedV(3,:)), 1, 13);
estimatedV(4,:) = sgolayfilt(double(estimatedV(4,:)), 1, 3);
estimatedV(5,:) = sgolayfilt(double(estimatedV(5,:)), 1, 3);
estimatedV(6,:) = sgolayfilt(double(estimatedV(6,:)), 1, 3);


end 

plotData(estimatedV, sampledData, sampledVicon, sampledTime, datasetNum)

