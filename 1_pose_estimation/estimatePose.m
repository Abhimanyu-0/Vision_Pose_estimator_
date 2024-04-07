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
  
    id = data(t).id;
    
    % This function gives me the Corners of all ids detected in image data
    % in world frame
    res = getCorner(id,data,t);
    

    % Will calculate find A now 
    A = [];
%%
    for i = 1: numel(data(t).id)
        adder = [
             res(1,i), res(2,i), 1 , 0 , 0 , 0 , -data(t).p0(1,i)*res(1,i) , -data(t).p0(1,i)*res(2,i) , -data(t).p0(1,i) ;
             0 , 0 , 0 , res(1,i) , res(2,i),1, -data(t).p0(2,i)*res(1,i), -data(t).p0(2,i)*res(2,i), -data(t).p0(2,i);
             res(3,i), res(4,i), 1, 0, 0, 0, -data(t).p1(1,i)*res(3,i), -data(t).p1(1,i)*res(4,i), -data(t).p1(1,i);
             0 , 0 , 0 , res(3,i) , res(4,i) , 1 , -data(t).p1(2,i)*res(3,i) , -data(t).p1(2,i)*res(4,i) , -data(t).p1(2,i);
             res(5,i), res(6,i), 1, 0, 0, 0, -data(t).p2(1,i)*res(5,i), -data(t).p2(1,i)*res(6,i), -data(t).p2(1,i);
             0 , 0 , 0 , res(5,i) , res(6,i) , 1 , -data(t).p2(2,i)*res(5,i) , -data(t).p2(2,i)*res(6,i) , -data(t).p2(2,i);
             res(7,i), res(8,i), 1, 0, 0, 0, -data(t).p3(1,i)*res(7,i), -data(t).p3(1,i)*res(8,i), -data(t).p3(1,i);
             0 , 0 , 0 , res(7,i) , res(8,i) , 1 , -data(t).p3(2,i)*res(7,i) , -data(t).p3(2,i)*res(8,i) , -data(t).p3(2,i);
             res(9,i), res(10,i), 1, 0, 0, 0, -data(t).p4(1,i)*res(9,i), -data(t).p4(1,i)*res(10,i), -data(t).p4(1,i);
             0 , 0 , 0 , res(9,i) , res(10,i), 1 , -data(t).p4(2,i)*res(9,i) , -data(t).p4(2,i)*res(10,i), -data(t).p4(2,i)];

        A = [A;adder];
    end 
    % using SVD
    [U_c,S_c,V_c] = svd(A);   

     h = reshape(V_c(:,9),[3,3]);
  
     h = transpose(h);
     h = h*sign(V_c(9,9));

     % Intrinsic matrix 
     k = [311.0520,    0,    201.8724;
         0,       311.3885, 113.6210;
         0,           0,    1       ];
    
    
     R_T = (k)\h;
    
    % Now taking svd to get rotation matrix 
      inter_R = [R_T(1:3,1) R_T(1:3,2) cross(R_T(:,1),R_T(:,2))];
    
      [U_r, S_r, V_r] = svd(inter_R);
      R = U_r * [1 0 0; 0 1 0; 0 0 det(U_r*V_r')] * V_r'; % Rotation Matrix

      T = R_T(:,3)/(norm(R_T(:,1)));
      
      Transformation = [0.707, -0.707, 0, 0.04;
                        -0.707, -0.707, 0, 0.0;
                         0, 0, -1, -0.03;
                         0,0,0,1];
      Rot = [R; 0 0 0];
      Tr = [T; 1];
      pose_world = inv((Transformation)*[Rot, Tr]); 

        
        
    %% Output Parameter Defination
    position  = pose_world(1:3,4);
    % position = translation vector representing the position of the
    % drone(body) in the world frame in the current time, in the order ZYX
    
    
    orientation = rotm2eul(pose_world(1:3,1:3), "ZYX");
    % orientation = euler angles representing the orientation of the
    % drone(body) in the world frame in the current time, in the order ZYX
end