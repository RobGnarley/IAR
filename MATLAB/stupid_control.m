% Initialise global variables
ideal_distance = 300;
speed = 10;
vleft = speed;
turn_speed = 5;
error = 0;

% Main control loop 
while true
    %pause(0.1);

    percepts = readIR(s);
    % Check if the percept input is in the correct form.
    % Should reduce the amount of crashes due to corrupt serial communication
    if size(percepts, 2) == 8 
    front_average = (percepts(4) + percepts(3))/2;
    left_average = (percepts(1) + percepts(2) + percepts(8))/3;
    right_average = (percepts(5) + percepts(6) + percepts(7))/3;
    
    % Object detection layer - layer 1
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
      % Wall following layer - layer 2
       if percepts(6) > 100 % If a wall is detected on the right
          
          old_vleft = vleft;
          old_error = error;
          error = ideal_distance - percepts(6);
          if error > 50
              delta_v = int8(error/50);
              delta_v = min(delta_v, 3);
              turn(s, speed + delta_v, speed);
          else if error < -50
                  delta_v = int8(error/50);
                  delta_v = max(delta_v, -3);
                  turn(s, speed + delta_v, speed)
              else
                  go(s, speed)
              end
          end
       else if percepts(1) > 100 % If a wall is detected on the left
          old_vleft = vleft;
          old_error = error;
          error = ideal_distance - percepts(1);
          if error > 50
              delta_v = int8(error/50);
              delta_v = min(delta_v, 3);
              turn(s, speed - delta_v, speed)
          else if error < -50
                  delta_v = int8(error/50);
                  delta_v = max(delta_v, -3);
                  turn(s, speed - delta_v, speed)
              else
                  go(s, speed)
              end
          end
           else % If no walls nor objects found - go straight forwards
            vleft = speed;
            go(s, speed);
           end
       end
    end
    end
end