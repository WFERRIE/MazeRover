function x = arduinoCmd(a, command, value)
%% Initialize com port
%     a = bluetooth('98D331F5AFF0');
    pause(0.2)
    %% rotate
    if command == 'rot'
        if value == 90
            write(a, 'R', "char")
        elseif value == -90
            write(a, 'r', "char")
        elseif value == 30
            write(a, 'T', "char")
        elseif value == -30
            write(a, 't', "char")
        elseif value == 5
            write(a,'Q', "char")
        elseif value == -5
            write(a,'q', "char")
        elseif value == 1
            write(a,'O', "char")
        elseif value == -1
            write(a,'0', "char")
        else
            if rem(value, 30) == 0
                repeat = abs(value/30); 
                if value>0
                    for i = 1:repeat
                        write(a,'T', "char")
                        pause(0.2)
                    end
                else
                    for i = 1:repeat
                        write(a,'t', "char")
                        pause(0.2)
                    end
                end

            elseif rem(value, 5) == 0
                repeat = abs(value/5); 
                if value>0
                    for i = 1:repeat
                        write(a,'Q', "char")
                        pause(0.2)
                    end
                else
                    repeat = abs(repeat);
                    for i = 1:repeat
                        write(a,'q', "char")
                        pause(0.2)
                    end
                end
            else
                repeat = abs(value); %smallest step size is 1 degree
                if value>0
                    for i = 1:repeat
                        write(a,'O', "char")
                        pause(0.2)
                    end
                else
                    for i = 1:repeat
                        write(a,'o', "char")
                        pause(0.2)
                    end
                end
            end
        end
%         pause(0.2)

    %% move forward
    elseif command =='for'
        repeat = abs(value/3); %smallest step size is 3 in
            for i = 1:repeat
                write(a,'f', "char")
                pause(0.2)
            end
    %% move backward
    elseif command =='bac'
        repeat = abs(value/3); %smallest step size is 3 in
            for i = 1:repeat
                write(a,'b', "char")
                pause(0.2)
            end  
    %% read ultrasound sensor 
    elseif command == 'rUS'
        write(a,'s', "char")
        pause(0.2)
        x = read(a, 7);
        prev7 = x(7);
        prev1 = x(1);
        write(a,'s', "char")
        pause(0.2)
        x = read(a, 7);
        
        while (abs(prev1 - x(1)) > 5) || (abs(prev7-x(7))>5)
            prev7 = x(7);
            prev1 = x(1);
            write(a,'s', "char")
            pause(0.2)
            x = read(a, 7);
        end
    
    %% red led
    elseif command == 'red'
        if value == 0
            write(a,'4', "char")
        elseif value == 1
            write(a, '3', "char")
        pause(0.2)
        end
    %% yellow led
    elseif command == 'yel'
        if value == 0
            write(a,'2', "char")
        elseif value == 1
            write(a, '1', "char")
        pause(0.2)
        end
        
    %% gripper    
    elseif command == 'oGr'
        write(a, 'g', "char")
    elseif command == 'cGr'
        write(a, 'G', "char")
    pause(0.2)

    %% move 1 cm
    elseif command == '1cm'
        if value == 1
            write(a,'c', "char")
            pause(0.2)
        elseif value == -1
            write(a,'C', "char")
            pause(0.2)
        else
            if value > 0
                for i = 1:value
                    write(a,'c', "char")
                    pause(0.2)
                end
            elseif value < 0
                for i = 1:abs(value)
                    write(a,'C', "char")
                    pause(0.2)
                end
            end
        end
    
end

% Here’s a list of all commands we currently have:
% ‘f’ forward movement
% ‘b’ backward movement
% ‘s’ reads all sensors (I can tell you which # corresponds to which sensor once we install)
% ‘1’ turns a yellow LED on (it should turn on when localization is complete)
% ‘2’ turns the LED off.
% ‘3’ turns a green LED on (we can save this for when the rover gets to the loading zone in the next milestone)
% ‘4’ turns it off 
% 'r' rotate 5
% 'R' rotate 90
% 't' rotate 30
% 'T' rotate - 30