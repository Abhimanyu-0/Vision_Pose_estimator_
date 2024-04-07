function [Vel] = velocityRANSAC(optV,optPos,Z,R_c2w,e)
     
    %% CHANGE THE NAME OF THE FUNCTION TO velocityRANSAC
    %% Input Parameter Description
    % optV = The optical Flow
    % optPos = Position of the features in the camera frame 
    % Z = Height of the drone
    % R_c2w = Rotation defining camera to world frame
    %e = 0.8; %RANSAC hyper parameter
     
     psuccess = 0.99; % probability of success

     M = 3; % number of points

     k = log(1-psuccess)/log(1-(e^M)); % number of iterations required
     Max_inliers = 0; % initializing maximum numers of inliers

     for i = 1: k
     
     rand_init_position = randperm(length(optPos),3); % Random values of position
     p1 = optPos(rand_init_position(1,1),:);
     p2 = optPos(rand_init_position(1,2),:);
     p3 = optPos(rand_init_position(1,3),:);
     
     
     H1 = [-1/Z(rand_init_position(1,1)) 0 p1(1,1)/Z(rand_init_position(1,1))  p1(1,1)*p1(1,2) -(1+p1(1,1)^2) p1(1,2);
            0 -1/Z(rand_init_position(1,1)) p1(1,2)/Z(rand_init_position(1,1)) (1+p1(1,2)^2) -p1(1,1)*p1(1,2) -p1(1,1)];

     H2 = [-1/Z(rand_init_position(1,2)) 0 p2(1,1)/Z(rand_init_position(1,2))  p2(1,1)*p2(1,2) -(1+p2(1,1)^2) p2(1,2);
           0 -1/Z(rand_init_position(1,2)) p2(1,2)/Z(rand_init_position(1,2)) (1+p2(1,2)^2) -p2(1,1)*p2(1,2) -p2(1,1)];

     H3 = [-1/Z(rand_init_position(1,3)) 0 p3(1,1)/Z(rand_init_position(1,3))  p3(1,1)*p3(1,2) -(1+p3(1,1)^2) p3(1,2);
           0 -1/Z(rand_init_position(1,3)) p3(1,2)/Z(rand_init_position(1,3)) (1+p3(1,2)^2) -p3(1,1)*p3(1,2) -p3(1,1)];

     H = [H1; H2; H3]; 

     uv = [optV(2*rand_init_position(1,1) - 1); optV(2*rand_init_position(1,1)); optV(2*rand_init_position(1,2) - 1); optV(2*rand_init_position(1,2)); optV(2*rand_init_position(1,3) - 1); optV(2*rand_init_position(1,3))]; % optical flow velocity

     V = pinv(H)*uv; % VW for this minimal set

     inliers = 0;

     for a = 1: length(optPos)

         Hi = [-1/Z(a) 0 optPos(a,1)/Z(a)  optPos(a,1)*optPos(a,2) -(1+optPos(a,1)^2) optPos(a,2);
              0 -1/Z(a) optPos(a,2)/Z(a) (1+optPos(a,2)^2) -optPos(a,1)*optPos(a,2) -optPos(a,1)];
         
         Pi = [optV(2*a - 1); optV(2*a)];

         beta = (norm(Hi*V -Pi))^2;

         if(beta<= 0.005) 

            inliers = inliers +1;

         end

     end
      
     if(inliers >= Max_inliers)

         Max_inliers = inliers;
         Vel = V;
     end
     end

    %% Output Parameter Description
    % Vel = Linear velocity and angualr velocity vector
    
end