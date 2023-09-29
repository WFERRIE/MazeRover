function desiredHeading = getDesiredHeading(VectorMap, xcoord, ycoord, prevDesHeading)
%     [~,column] = max(max(p));
%     [~,row] = max(p(:,column));    
    
    
    dir = VectorMap(ycoord, xcoord);
    switch(dir)
        case 2
            desiredHeading = 90;
        case 3
            desiredHeading = 270;
        case 4
            desiredHeading = 180;
        case 5
            desiredHeading = 0;
        case 8
            desiredHeading = 263;
        otherwise
            disp('getDesiredHeading thinks its in a wall')
            desiredHeading = prevDesHeading;
    end

end