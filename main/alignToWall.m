function aligned = alignToWall(u, s_cmd, s_rply)
    d =  7.8; %cm; distance between the two sensors 

    %for simmer:
    d = 3;

%% find which sensor is closest to wall
    sensor = find(u==min(u));
    rotate = 0;
    if (length(sensor)) > 1
        sensor = sensor(1);
    end
    
%I think before we rotate, we may want to back up? but not sure how to do
%this well 
    if sensor == 1 || sensor == 3
        rotate = 90;
        %bluetooth
%     arduinoCmd('rot', rotate)

       %simmer
        cmdstring = [strcat('r1-',num2str(rotate)) newline]; 
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    end
    
    %get new ultrasound reading: 
    %simmer:
    cmdstring = ['ua' newline];
    u = tcpclient_write(cmdstring, s_cmd, s_rply);

    %bluetooth
%     u = arduinoCmd('rUS');

    goodSensor = 0; %set flag
    while goodSensor == 0
        %% Now that robot is facing closest wall, use sensors 2and 3, 5 and 6 to calculate the angle to the wall
        %sensor 2
        x1(1) = 0;
        y1(1) = u(2);
        
        %sensor 3
        x1(2) = d; %distance in cm between sensor 2 and 3
        y1(2) = u(5);
    
        %sensor 5
        x2(1) = 0;
        y2(1) = u(4);
    
        %sensor 6
        x2(2) = d;
        y2(2) = u(6);
    
        %% Make line of best fit to calculate angle between wall and sensors 2+3, then 5+6
        if y1(1)-y1(2) < 5 %choose threshold     
%             slope = polyfit([x1(1) x1(2)], [y1(1) y1(2)], 1);
%             angle = round(atand(slope(1)));
            a1 = round(atand((y1(2)-y1(1))/(x1(2)-x1(1))));
            if y2(1)-y2(1) < 5
%                 slope = polyfit([x2(1) x2(2)], [y2(1) y2(2)], 1);
%                 angle = (angle + round(atand(slope(1))))/2; %takes average if both side readings are accurate
                a2 = round(atand((y2(2)-y2(1))/(x2(2)-x2(1))));
                angle = round((abs(a1)+abs(a2))/2);
            end
            goodSensor = 1;
        elseif y2(1)-y2(1) < 5
%             slope = polyfit([x2(1) x2(2)], [y2(1) y2(2)], 1);
%             angle = round(atand(slope(1))); 
            angle = round(atand((y2(2)-y2(1))/(x2(2)-x2(1))));
            goodSensor = 1;
        else %rotate to get new measurements
            %bluetooth
        %     arduinoCmd('rot', 5)
        
            %simmer
            cmdstring = [strcat('r1-',num2str(5)) newline]; 
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        end
    end
        
    %% Rotate back to original position and then move the additional angle
    if a1<0
        angle = -angle;
    end

    rotate = angle - rotate;

    if rotate ~= 0 || angle ~= 0
        %bluetooth
    %     arduinoCmd('rot', rotate)
    
        %simmer
        cmdstring = [strcat('r1-',num2str(rotate)) newline]; 
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    end
    
%Original code for front sensors:


%     %% rotate so that sensors 1, 5 and 6 face the wall
%     if sensor == 2
%         rotate = -90;
%     elseif sensor == 3
%         rotate = 180;
%     elseif sensor == 4
%         rotate = 90;
%     elseif sensor == 5
%         rotate = -30;
%     elseif sensor == 6
%         rotate = 30;
%     end
%     
%     %bluetooth
% %     arduinoCmd('rot', rotate)
% 
%     %simmer
%     cmdstring = [strcat('r1-',num2str(rotate)) newline]; 
%     reply = tcpclient_write(cmdstring, s_cmd, s_rply);
%     
%     %% Now that robot is facing closest wall, use sensors 1, 5, and 6 to calculate the angle to the wall
%     % a represents the normal distance from wall to sensor 1, using sensor
%     % 5; b represents the sensor reading for 1, and c represents the normal
%     % distance from wall to sensor 1, using sensor 6
% 
%     d = 1.183; %normal distance in y direction in cm from sensor 5 to 1, 6 to 1
%     p = 6.245; % normal distance in x direction in cm from sensor 1 to 5, 1 to 6
%     
%     ay = u(5)*cosd(30) - d;
%     by = u(1);
%     cy = u(6)*cosd(30) - d;
% 
%     ax = 0;
%     bx = u(5)*sind(30)+6.2452;
%     cx = bx + 6.2452 + u(6)*sind(30);
%     
%     %% Make line of best fit to calculate angle between wall and sensor 1
%     slope = polyfit([ax bx cx], [ay by cy], 1);
%     angle = atand(slope(1));
%     angle = round(angle); %makes the angle a multiple of 1; the lowest rotation arduino takes
%     
%     %% Rotate back to original position and then move the additional angle
%     rotate = -angle - rotate;
% 
%     %bluetooth
% %     arduinoCmd('rot', rotate)
% 
%     %simmer
%     cmdstring = [strcat('r1-',num2str(rotate)) newline]; 
%     reply = tcpclient_write(cmdstring, s_cmd, s_rply);

end
