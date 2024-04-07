function res = getCorner(id,data,t)
%% CHANGE THE NAME OF THE FUNCTION TO getCorner
    %% Input Parameter Description
    % id = List of all the AprilTag ids detected in the current image(data)
  
    tag_size = 0.152;
    tag_spacing = 0.152;
    special_spacing = 0.178;
    world_tag_corners = zeros(10,numel(length(data(t).id)));
    for i = 1: length(id)
        p = data(t).id(1,i);
        row = mod(p,12) + 1;
        col = ceil(p/11);
    
        if p<=11
            x = 0 + 2*0.152*mod(p,12); 
            y = 0;
        elseif p>=12 && p<=23
            y = 2*0.152;
            x = 2*0.152*mod(p,12);
        elseif p>=24 && p<=35
            y = 4*0.152;
            x = 2*0.152*mod(p,12);
        elseif p>=36 && p<=47
            y = 5*0.152 + special_spacing;
            x = 2*0.152*mod(p,12);
        elseif p>=48 && p<=59
            y = 5*0.152 + special_spacing + 2*0.152;
            x = 2*0.152*mod(p,12);
        elseif p>=60 && p<=71
            y = 5*0.152 + special_spacing + 2*0.152 + 2*0.152;
            x = 2*0.152*mod(p,12);
        elseif p>=72 && p<=83
            y = 5*0.152 + special_spacing + 2*0.152 + 2*0.152 + 0.152 + special_spacing;
            x = 2*0.152*mod(p,12);
        elseif p>=84 && p<=95
            y = 5*0.152 + special_spacing + 2*0.152 + 2*0.152 + 0.152 + special_spacing + 2*0.152;
            x = 2*0.152*mod(p,12);
        elseif p>=96 && p <=107
            y = 5*0.152 + special_spacing + 2*0.152 + 2*0.152 + 0.152 + special_spacing + 2*0.152 + 2*0.152;
            x = 2*0.152*mod(p,12);
        end 

        world_tag_corners(:,i) = [x+(tag_size/2);y+(tag_size/2);x+tag_size;y;x+tag_size;y+tag_size;x;y+tag_size;x;y];
             
    end 
        res = world_tag_corners;
    %% Output Parameter Description
    % res = List of the coordinates of the 4 corners (or 4 corners and the
    % centre) of each detected AprilTag in the image in a systematic method
end