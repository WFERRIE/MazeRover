clear
close all
clc

startHeading = 0; % -1 * starting orientation. Robot will rotate this
% amount when it starts so it will start at a 0 degree heading.
DROPOFFZONELOCATION = 5; % set this for which dropoff zone the robot will
% go to. Zones are described in buildVectorMap.m
heading = 0; % desired starting heading

% Initialize tcp server to read and respond to algorithm commands
[s_cmd, s_rply] = tcp_setup(); %initiate command stuff
sim = 1; % set to 0 if running on the robot, 1 if doing a sim
if sim
    fopen(s_cmd);
    speed = 2; % speed is 2 for simulation
else
    a = bluetooth('98D331F5AFF0'); %initiate bluetooth connection
    speed = 3; % speed is 3 for real life
end

% Robot Sensor Measurements
u = [0,0,0,0,0,0,0];  % Ultrasonic measurements

rotStuck = 90; % amount the robot will rotate when it can't go forward
rotReallyStuck = 90; % additional rotation robot will do when stuck
stuckCount = 0; % initialize counter for checking if robot is stuck
stuckThresh = 5; % number of rotations before robot decides it is stuck
stepcount = 0;
localized = 0; % flag for if localized
locProbThreshUpper = 0.05; % criteria for localization to converge
locProbThreshLower = 0.02; 
locCountThresh = 4;
prevDesHeading = heading; % keep track of a previous heading
lastRot = 0; 
lastMovement = 0;
schmoovin = 0; % flag for driving to loading zone
partyTime = 0; % flag for being done
wallAlignmentFlag = 10;


 %% setup stuff
dim1 = 32; dim2 = 16;
locationindex = reshape(1:dim1*dim2,dim1,dim2)'; 
worldCoord = zeros(48, 96);
VectorMap = zeros(48, 96);
n = numel(locationindex);
rand('twister',5489);
bw = reshape(randi([0 1],n,1),dim2,dim1); %0 = black, 1 = white

%make blocks
M = zeros(size(bw));
Blocks = [2, 3; 3, 2; 4, 3; 5, 1; 5, 3; 7, 1; 7, 3; 7, 4;];
for xx = 1:size(Blocks,1),
	x = Blocks(xx,1); y = Blocks(xx,2);
	M(1+(y-1)*4:(y-1)*4+4, 1+(x-1)*4:(x-1)*4+4) = 1;
    VectorMap(1+(y-1)*4:(y-1)*4+4, 1+(x-1)*4:(x-1)*4+4) = 1;
end
% at this point M is a map of the maze with 0's for driveable spots
% and 1's for the walls
M = [ones(dim2,1) M ones(dim2,1)];
M = [ones(1, dim1+2); M; ones(1, dim1+2)];
%the two lines above add a border of 1's to the maze

VectorMap = buildVectorMap(VectorMap, 1); % vector map for driving to LZ

% generate ultrasonic world
ultra = zeros(size(bw)); % empty ultrasonic world
for sec_row = 1:4:dim2
% this for loop goes through M in 4x4 'cells' and assigns ultra
% the reading of what type of cell it is (1 to 5) based on the key
% in the lecture slides
    for sec_col = 1:4:dim1,
        segRow = M(sec_row+2, sec_col:sec_col+5);
        segCol = M(sec_row:sec_row+5, sec_col+2);
        val = sum(segRow)+sum(segCol);
        if val == 2 && sum(segRow)~=1,
            val = 5;
        end
        ultra(sec_row:sec_row+3, sec_col:sec_col+3) = val;
    end
end
% create mask for blocks
M = abs(M-1);
M = M(2:end-1, 2:end-1);


% initialize probability to 0.002 for each minicell
p = ones(dim2,dim1)*(1/n); 

%% initial rotation
if sim
    cmdstring = [strcat('r1-',num2str(heading - startHeading)) newline];  % Build command string to rotate bot
    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
else
    arduinoCmd(a, 'rot', (startHeading)) % rotates to start at a 0 deg heading
end

%% Localization
while ~localized
    pause
    if sim
        cmdstring = ['ua' newline];
        u = tcpclient_write(cmdstring, s_cmd, s_rply);
    else
        u = arduinoCmd(a, 'rUS', 0) % take sensor reading
    end

    %     sensor update
    m_u = getLocalizationCell(u); % determine what type of cell we're in
    
    p = sense_u(ultra, M, p, m_u); % updates our probabilities

    imagesc(p); % display probability map
    title(['step: ' num2str(stepcount)]);
    
    if  (u(1) > 10)
        % If the way ahead is clear, go forward
        if sim
            cmdstring = [strcat('d1-',num2str(speed)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        else
            arduinoCmd(a, 'for', speed)
        end
        disp('move forward')
        [p, heading] = schmoove(u, p, M, heading, speed); 
        p = sense_u(ultra, M, p, m_u);
        stuckCount = 0; % reset the stuck count
        
    elseif (u(2) > u(1)) && (u(2) > 8) && (u(5) > 10)
        % if way ahead isn't clear and there's room on the left side,
        % rotate left
        if sim
            cmdstring = [strcat('r1-',num2str(rotStuck)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        else
            arduinoCmd(a, 'rot', rotStuck)
        end
        disp('rotate left')
        heading = heading + rotStuck; % update heading
        stuckCount = stuckCount + 1; % increment the stuck count
        
    else
        % if we get to this point it means there's not room in front and no
        % room on the left, so there must be room on the right to rotate
        if sim
            cmdstring = [strcat('r1-',num2str(rotStuck * (-1))) newline];  % Build command string to rotate bot
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        else
            arduinoCmd(a, 'rot', (rotStuck * -1))
        end
        disp("rotate right") 
        heading = heading - rotStuck;
        stuckCount = stuckCount + 1;
        if stuckCount > stuckThresh,
            % if stuck count exceeds our threshold, robot will do an
            % additional rotation. This stops the robot from being stuck in
            % a loop of rotating back and forth and making no progress.
            if sim
                cmdstring = [strcat('r1-',num2str(rotReallyStuck * (-1))) newline];  % Build command string to rotate bot
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            else
                arduinoCmd(a, 'rot', (rotStuck * (-1)))
                heading = heading - rotStuck;
            end
        end
    end
    
    if abs(heading) >= 360
        % if the heading has overflown, just reset it so it stays within
        % 360 degrees
        heading = mod(heading, 360);
    end
    
    % checkLocalized determines if our probability map meets the criteria
    % set during setup and if we can decide we are localized or not. If so,
    % we set localized = 1 and proceed onwards.
    localized = checkLocalized(p, locProbThreshUpper, locProbThreshLower, locCountThresh);
    
    stepcount = stepcount+1;
end

if ~sim
    % turn on the red LED to show we have localized.
    arduinoCmd(a, 'red', 1) %turn on red LED to confirm localized
end


% get x and y coordinates of the maximum probability location. We will keep
% tracking our position based on these coordinates.

[value,xcoord] = max(max(p));
[~,ycoord] = max(p(:,xcoord));

% scale the coordinates up to a 48x96 matrix since p was 16x32 (just means
% our tracking will be higher resolution and more accurate)
ycoord = ycoord * 3;
xcoord = xcoord * 3;

% do a check to make sure our scaled coordinates are within the bounds of
% the maze. If they're not, just set them to the boundary.
if ycoord > 48
    ycoord = 48;
end
if xcoord > 96
    xcoord = 96;
end
if ycoord < 1
    ycoord = 1;
end
if xcoord < 1
    xcoord = 1;
end

worldCoord(ycoord, xcoord) = 1;
desiredHeading = getDesiredHeading(VectorMap, xcoord, ycoord, prevDesHeading);

% reorient is a flag that will have the robot reorient itself to the
% desired direction based on the VectorMap. This happens every 4 and 8 steps, alternating.
reorient = 4;

if sim
    cmdstring = [strcat('r1-',num2str(desiredHeading - heading)) newline];  % Build command string to rotate bot
    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
else
    % rotate to align with the VectorMap
    arduinoCmd(a, 'rot', (desiredHeading - heading));
%     heading = alignToWall(u, s_cmd, s_rply, heading, sim);
end


heading = desiredHeading; % update our heading

%% drive to loading zone code
disp('Localization Complete, heading to loading zone')
while localized && ~schmoovin
    % schmoovin is our flag that becomes true once we are in the LZ
    if sim
        cmdstring = ['ua' newline];
        u = tcpclient_write(cmdstring, s_cmd, s_rply);
    else
        u = arduinoCmd(a, 'rUS', 0)
    end
    
    imagesc(worldCoord); % visualize where the robot thinks it is
    title(['step: ' num2str(stepcount)]);
    
    prevDesHeading = desiredHeading;
    % keep track of the previous desired heading. sometimes the robot will
    % think it is in a wall, so if that is the case, getDesiredHeading will
    % just return the prevDesHeading
    desiredHeading = getDesiredHeading(VectorMap, xcoord, ycoord, prevDesHeading);
    if desiredHeading == 263 % this means we are in the loading zone
        disp('loading zone reached')
        if ~sim
            arduinoCmd(a, 'yel', 1)
        end
        
        schmoovin = 1; % set schmoovin to 1 to break out of this while loop
    end

   if mod(lastMovement, reorient) == 0 && desiredHeading ~= 263
%        disp('reorienting')
       if sim
           cmdstring = [strcat('r1-',num2str(desiredHeading - heading)) newline];  % Build command string to rotate bot
           reply = tcpclient_write(cmdstring, s_cmd, s_rply);
       else
           arduinoCmd(a, 'rot', (desiredHeading - heading))
           if u(2) < 10 && u(4) < 10
               if u(2) < 5
                   arduinoCmd(a, 'rot', -5);
                   heading = heading -5;
               elseif u(4) < 5
                   arduinoCmd(a, 'rot', 5);
                   heading = heading + 5;
               end
           end
%            heading = alignToWall(u, s_cmd, s_rply, heading, sim);
       end
       lastMovement = lastMovement + 1;
       heading = desiredHeading;
       if reorient == 8
           reorient = 4;
       else
           reorient = 8;
       end
        
   elseif  (u(1) > 10) % && (u(5) > 2.5) && (u(6) > 2.5)
        % If the way ahead is clear, go forward
        if sim
            cmdstring = [strcat('d1-',num2str(speed)) newline];             % Build command string to move bot
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        else
            arduinoCmd(a, 'for', speed)
        end
%         disp("move forward")
%        [p, heading] = schmoove(u, p, M, heading, speed); 
%        p = sense_u(ultra, M, p, m_u);
        [worldCoord, xcoord, ycoord] = trackRobot(worldCoord, xcoord, ycoord, heading, speed);
        stuckCount = 0;
        lastMovement = lastMovement + 1;
        
    elseif (u(2) > u(4))
       %if more room on left than right and room forward, rotate left
       if sim
           cmdstring = [strcat('r1-',num2str(rotStuck)) newline];  % Build command string to rotate bot
           reply = tcpclient_write(cmdstring, s_cmd, s_rply);
       else
           arduinoCmd(a, 'rot', rotStuck)
       end
       heading = heading + rotStuck;
       stuckCount = stuckCount + 1;
       lastMovement = lastMovement + 1;
        
    elseif (u(2) > u(1)) && (u(2) > 8) % && (u(5) > 10)
        % If not, pick a random direction that is clear and go that way
        if sim
            cmdstring = [strcat('r1-',num2str(rotStuck)) newline];  % Build command string to rotate bot
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        else
            arduinoCmd(a, 'rot', rotStuck)
        end
        heading = heading + rotStuck;
        stuckCount = stuckCount + 1;
        lastMovement = lastMovement + 1;
        
    else
        % If not, pick a random direction that is clear and go that way
        if sim
            cmdstring = [strcat('r1-',num2str(rotStuck * (-1))) newline];  % Build command string to rotate bot
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        else
            arduinoCmd(a, 'rot', (rotStuck * (-1)))
        end
        
%         disp("rotate right") 
        heading = heading - rotStuck;
        stuckCount = stuckCount + 1;
        lastMovement = lastMovement + 1;
        if stuckCount > stuckThresh,
            if sim
                cmdstring = [strcat('r1-',num2str(rotReallyStuck * (-1))) newline];  % Build command string to rotate bot
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            else
                arduinoCmd(a, 'rot', (rotReallyStuck * (-1)))
            end
            stuckCount = 0;
            heading = heading - rotReallyStuck;
%             disp("REALLY stuck: rotate right twice")
        end
   end
    if abs(heading) >= 360
        heading = mod(heading, 360);
    end
    
%     heading
    stepcount = stepcount + 1;
    
end

clear VectorMap
VectorMap = zeros(48, 96);
VectorMap = buildVectorMap(VectorMap, DROPOFFZONELOCATION);

disp('in loading zone, attempting to pick up block')


%% block detection code


blockFound = 0;
flag1 = 0;


% arduinoCmd(a, 'for', speed)
% [worldCoord, xcoord, ycoord] = trackRobot(worldCoord, xcoord, ycoord, heading, speed);
arduinoCmd(a, 'oGr', 0);
arduinoCmd(a, 'rot', -20);
heading = heading - 20;
while ~blockFound
    clear bottomValues
    clear frontU
    clear delta
    arduinoCmd(a, 'rot', 20);
    heading = heading + 20;
    bottomValues = [];
    frontU = [];
    delta = [];
    kernel = [];
    prev7 = 0;
    prev1 = 0;
    k = 1;
    %% large adjustments
    while flag1 == 0 && k <= 180/5
        u = arduinoCmd(a, 'rUS', 0)
%         while abs(prev1 - u(1)) > 30
%             u = arduinoCmd(a, 'rUS', 0);
%             prev1 = u(1);
%         end
        if u(7) < 10
            disp('we gettin close')
            if u(7) < 6
                blockFound = 1;
                break
            end
        end
        if k ~= 1   
            if ((prev7 - u(7)) > 5 && u(7) < 30) || ((u(1)-u(7))>10)
                if (u(1)-u(7)) >= 10
                    disp('da block may be found')
                    counter = 1;
                    arduinoCmd(a, 'rot', -5);
                    heading = heading - 5;
                    
                    u = arduinoCmd(a, 'rUS', 0)
                    %find "middle" of block then move forward 
                    if (u(1)-u(7)) >= 10
                        while (u(7)-prev7) <= 10
                            arduinoCmd(a, 'rot', -5);
                            heading = heading - 5;
                            counter = counter + 1;
                            prev7 = u(7);
                            u = arduinoCmd(a, 'rUS', 0)
                        end
                        arduinoCmd(a, 'rot', round((counter)/2)*5);
                        heading = heading + (round((counter) / 2) * 5);
                        flag1 = 1;
                        if u(7) < 20
                            arduinoCmd(a, '1cm', u(7)-6)
                            [worldCoord, xcoord, ycoord] = trackRobot(worldCoord, xcoord, ycoord, heading, 1);
                        else
                            arduinoCmd(a, 'for', speed)
                            [worldCoord, xcoord, ycoord] = trackRobot(worldCoord, xcoord, ycoord, heading, speed);
                        end
                    end
                end
            end
        end

        prev7 = u(7);
        prev1 = u(1);
        k = k+1;
        arduinoCmd(a, 'rot', -5);
        heading = heading - 5;

%         bottomValues = [bottomValues, u(7)];
%         frontU = [frontU, u(1)];
%         delta = [delta, frontU(k) - bottomValues(k)];
    end
    pause(0.2)
    if u(7) > 10
        arduinoCmd(a, 'rot', 10)
        heading = heading + 10;
        pause(0.2)
        flag2 = 0;
    else
        flag2 = 1;
    end
    
    k = 0;
    blah = 1;
    j = 1;
    
    %% small adjustments
    while flag2 == 0 && k <= 20
        u = arduinoCmd(a, 'rUS', 0)
%         while abs(prev1 - u(1)) > 30
%             u = arduinoCmd(a, 'rUS', 0);
%             prev1 = u(1);
%         end
        while u(7) < 15
            disp('we gettin close')
            if u(7) <= 6
                blockFound = 1;
                flag2 =1;
                arduinoCmd(a, '1cm', 3);
                [worldCoord, xcoord, ycoord] = trackRobot(worldCoord, xcoord, ycoord, heading, 1);
                break
            else 
                arduinoCmd(a, '1cm', 1);
                [worldCoord, xcoord, ycoord] = trackRobot(worldCoord, xcoord, ycoord, heading, 1);
                prev7 = u(7);
                u = arduinoCmd(a, 'rUS', 0)
                
                if u(7)>prev7
                    blah = blah * -1;
                    blah_rot = 5*j;
                    j = j+1
                    arduinoCmd(a, 'rot', blah_rot*blah*j);
                    heading = heading + blah_rot*blah*j;
                    prev7 = u(7);
%                     u = arduinoCmd(a, 'rUS', 0)
% %                     while pre
%                         arduinoCmd(a, 'rot', -5);
%                         prev7 = u(7);
%                         u = arduinoCmd(a, 'rUS', 0)
%                     end
                end

%                 while abs(prev1 - u(1)) > 2
%                     u = arduinoCmd(a, 'rUS', 0);
%                     prev1 = u(1);
%                 end

            end
        end
        arduinoCmd(a, '1cm', 1);
        [worldCoord, xcoord, ycoord] = trackRobot(worldCoord, xcoord, ycoord, heading, 1);
        u = arduinoCmd(a, 'rUS', 0)
        
        if k ~= 1 && ~blockFound
%             if (prev7 - u(7)) > 5 && u(7) < 30
            if u(1)-u(7) >= 10
                disp('da block may be found')
%                 if u(7) > 10
%                     counter = 1;
%                     arduinoCmd(a, 'rot', -5);
%                     u = arduinoCmd(a, 'rUS', 0)
%                     %find "middle" of block then move forward 
%                     if u(1)-u(7) >= 10
%                         while (u(7)-prev7) <= 10
%                             arduinoCmd(a, 'rot', -5);
%                             counter = counter + 1;
%                             prev7 = u(7);
%                             u = arduinoCmd(a, 'rUS', 0)
%                         end
%                         arduinoCmd(a, 'rot', round((counter+1)/2)*5);
%                         distance = round((u(7) - 6)/3);
%                         arduinoCmd(a, '1cm', distance)
%                         flag2 =1;
%                     end
%                 end
            else
                arduinoCmd(a, '1cm', 1);
                [worldCoord, xcoord, ycoord] = trackRobot(worldCoord, xcoord, ycoord, heading, 1);
                u = arduinoCmd(a, 'rUS', 0)
            end
        end

        prev7 = u(7);
        prev1 = u(1);
        k = k+1;
%         arduinoCmd(a, 'rot', -2);

%         bottomValues = [bottomValues, u(7)];
%         frontU = [frontU, u(1)];
%         delta = [delta, frontU(k) - bottomValues(k)];
    end
    flag2 =1;
    break
end
arduinoCmd(a, 'cGr', 0);

disp('got the block (i think)')
%% head to dropoff zone code

while localized && schmoovin && ~partyTime
    if sim
        cmdstring = ['ua' newline];
        u = tcpclient_write(cmdstring, s_cmd, s_rply);
    else
        u = arduinoCmd(a, 'rUS', 0)
    end
    prevDesHeading = desiredHeading;
    desiredHeading = getDesiredHeading(VectorMap, xcoord, ycoord, prevDesHeading);
    if desiredHeading == 263
        if ~sim
            arduinoCmd(a, 'red', 0)
            arduinoCmd(a, 'yel', 0)
        end
        partyTime = 1;
    end
%     imagesc(p);
    imagesc(worldCoord);
%     desiredHeading
%     heading
    title(['step: ' num2str(stepcount)]);   
%     pause;
   if mod(lastMovement, reorient) == 0 && desiredHeading ~= 263;
%        disp('reorienting')
        if sim
            cmdstring = [strcat('r1-',num2str(desiredHeading - heading)) newline];  % Build command string to rotate bot
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        else
            arduinoCmd(a, 'rot', (desiredHeading - heading))
            if u(2) < 10 && u(4) < 10
               if u(2) < 5
                   arduinoCmd(a, 'rot', -5);
                   heading = heading -5;
               elseif u(4) < 5
                   arduinoCmd(a, 'rot', 5);
                   heading = heading + 5;
               end
           end
%             heading = alignToWall(u, s_cmd, s_rply, heading, sim);
        end
        lastMovement = lastMovement + 1;
        heading = desiredHeading;
        if reorient == 8
            reorient = 4;
        else
           reorient = 8;
       end
        
   elseif  (u(1) > 10) % && (u(5) > 2.5) && (u(6) > 2.5)
        % If the way ahead is clear, go forward
        if sim
            cmdstring = [strcat('d1-',num2str(speed)) newline];             % Build command string to move bot
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        else
            arduinoCmd(a, 'for', speed)
        end
        disp("move forward")
%        [p, heading] = schmoove(u, p, M, heading, speed); 
%        p = sense_u(ultra, M, p, m_u);
        [worldCoord, xcoord, ycoord] = trackRobot(worldCoord, xcoord, ycoord, heading, speed);
        stuckCount = 0;
        lastMovement = lastMovement + 1;
        
    elseif (u(2) > u(4)) && (u(5) > u(6)) && (u(2) > 4) && (u(5) > 2.5)
       %if more room on left than right and room forward, rotate left
       if sim
           cmdstring = [strcat('r1-',num2str(rotStuck)) newline];  % Build command string to rotate bot
           reply = tcpclient_write(cmdstring, s_cmd, s_rply);
       else
           arduinoCmd(a, 'rot', rotStuck)
       end
       heading = heading + rotStuck;
       stuckCount = stuckCount + 1;
       lastMovement = lastMovement + 1;
        
    elseif (u(2) > u(1)) % && (u(2) > 4) && (u(5) > 2.5)
        % If not, pick a random direction that is clear and go that way
        if sim
            cmdstring = [strcat('r1-',num2str(rotStuck)) newline];  % Build command string to rotate bot
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        else
            arduinoCmd(a, 'rot', rotStuck)
        end
        heading = heading + rotStuck;
        stuckCount = stuckCount + 1;
        lastMovement = lastMovement + 1;
        
    else
        % If not, pick a random direction that is clear and go that way
        if sim
            cmdstring = [strcat('r1-',num2str(rotStuck * (-1))) newline];  % Build command string to rotate bot
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        else
            arduinoCmd(a, 'rot', (rotStuck * (-1)))
        end
        disp("rotate right") 
        heading = heading - rotStuck;
        stuckCount = stuckCount + 1;
        lastMovement = lastMovement + 1;
        if stuckCount > stuckThresh,
            if sim
                cmdstring = [strcat('r1-',num2str(rotReallyStuck * (-1))) newline];  % Build command string to rotate bot
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            else
                arduinoCmd(a, 'rot', (rotReallyStuck * (-1)))
            end
            disp("REALLY stuck: rotate right twice")
            stuckCount = 0;
            heading = heading - rotReallyStuck;
        end
   end
    if abs(heading) >= 360
        heading = mod(heading, 360);
    end
    
%     heading
    stepcount = stepcount + 1;
    
end

disp('party time!')
arduinoCmd(a, 'oGr', 0);
arduinoCmd(a, 'bac', 3);