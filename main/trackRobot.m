function [world, xcoord, ycoord] = trackRobot(world, xcoord, ycoord, heading, speed)

    xmove = round(speed * cosd(heading));
    ymove = round(speed * sind(heading));
    world(ycoord, xcoord) = 0;
    if (ycoord - ymove) <= 0 || (ycoord + ymove) > 96 || (ycoord + ymove) <= 0 || (ycoord - ymove) > 96
        ymove = 0;
    end
    
    if (xcoord - xmove) <= 0 || (xcoord + xmove) > 96 || (xcoord + xmove) <= 0 || (xcoord - xmove) > 96
        xmove = 0;
    end
    world(ycoord - ymove, xcoord + xmove) = 1;
    ycoord = ycoord - ymove;
    xcoord = xcoord + xmove;
        
    
end