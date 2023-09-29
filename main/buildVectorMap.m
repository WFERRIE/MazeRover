function [VectorMap] = buildVectorMap(VectorMap, version)
    goal = 8;
    up = 2;
    down = 3;
    left = 4;
    right = 5;
    if version == 1 %for loading zone
        VectorMap(1:24,1:24) = goal;
        VectorMap(25:48,1:12) = up;
        VectorMap(1:12,25:48) = left;
        VectorMap(13:24,37:48) = up;
        VectorMap(13:24,49:96) = left;
        VectorMap(37:48,13:72) = left;
        VectorMap(25:36,25:36) = down;
        VectorMap(1:12,61:72) = down;
        VectorMap(25:48,85:96) = up;
        VectorMap(1:12,85:96) = down;
        VectorMap(25:48,61:72) = up;
        VectorMap(13:16, 25:28) = 0;

    elseif version == 2 %block in bottom left
        VectorMap(1:24,1:12) = down;
        VectorMap(1:24,13:24) = left;
        VectorMap(24:48,1:12) = down;
        VectorMap(1:12,25:48) = left;
        VectorMap(13:24,37:60) = right;
        VectorMap(13:24,49:72) = down;
        VectorMap(13:24,73:96) = left;
        VectorMap(37:48,25:36) = up;
        VectorMap(37:48, 1:24) = right;
        VectorMap(37:48,37:72) = left;
        VectorMap(25:36,25:36) = goal;
        VectorMap(1:12,61:72) = down;
        VectorMap(25:48,85:96) = up;
        VectorMap(1:12,84:96) = down;
        VectorMap(25:36,61:72) = down;
        VectorMap(13:24, 49:60) = right;
    
    elseif version == 3 %block in top left
        VectorMap(1:12,1:48) = right;
        VectorMap(13:24,1:24) = up;
        VectorMap(25:36,1:12) = up;
        VectorMap(37:48, 1:60) = right;
        VectorMap(25:36, 25:36) = down;
        VectorMap(1:12, 25:36) = right;
        VectorMap(1:12, 37:48) = down;
        VectorMap(13:24,37:60) = right;
        VectorMap(13:48,61:72) = up;
        VectorMap(13:24,73:96) = left;
        VectorMap(1:12,85:96) = down;
        VectorMap(1:12, 61:72) = goal;
        VectorMap(25:48, 85:96) = up;
        
    elseif version == 4 %block in top right
        VectorMap(1:12,1:48) = right;
        VectorMap(13:24,1:24) = up;
        VectorMap(25:36,1:12) = up;
        VectorMap(37:48, 1:60) = right;
        VectorMap(25:36, 25:36) = down;
        VectorMap(1:12, 25:36) = right;
        VectorMap(1:12, 37:48) = down;
        VectorMap(13:24,37:84) = right;
        VectorMap(25:48,61:72) = up;
        VectorMap(1:12,85:96) = goal;
        VectorMap(1:12, 61:72) = down;
        VectorMap(13:48, 85:96) = up;
        
    elseif version == 5 %block in bottom right
        VectorMap(1:12,1:48) = right;
        VectorMap(13:24,1:24) = up;
        VectorMap(25:36,1:12) = up;
        VectorMap(37:48,1:60) = right;
        VectorMap(25:36,25:36) = down;
        VectorMap(1:12,37:48) = down;
        VectorMap(13:24,37:84) = right;
        VectorMap(25:48,61:72) = up;
        VectorMap(1:36,85:96) = down;
        VectorMap(1:12,61:72) = down;
        VectorMap(37:48,85:96) = goal;   
    end
end