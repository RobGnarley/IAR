ideal_distance = 300;
K = 0.001;
speed = 10;
vleft = speed;
turn_speed = 5;
while true
    %pause(0.1);
    percepts = readIR(s);
    front_average = (percepts(4) + percepts(3))/2;
    left_average = (percepts(1) + percepts(2) + percepts(8))/3;
    right_average = (percepts(5) + percepts(6) + percepts(7))/3;
    
    if front_average > 100
        if left_average < right_average % turn left
            while percepts(5) > 100
                turn(s, -turn_speed, turn_speed)
                percepts = readIR(s);

            end
    else %turn right
            while percepts(2) > 100
                turn(s, turn_speed, -turn_speed)
                percepts = readIR(s);
            end
        end
    else
        vleft = speed;
        go(s, speed);  
    end
end