ideal_distance = 300;
K = 0.001;
speed = 10;
vleft = speed;
turn_speed = 5;
error = 0;
while true
    %pause(0.1);
    percepts = readIR(s);
    if size(percepts, 2) == 8
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
       if percepts(6) > 100
          
          old_vleft = vleft;
          old_error = error;
          error = ideal_distance - percepts(6);
          delta_error = old_error - new_error;
          if delta_error > 0
              d_control = -1;
          else
              d_control = 1;
          end
          if error > 50
              delta_v = int8(error/50) + d_control;
              delta_v = min(delta_v, 3);
              turn(s, speed + delta_v, speed)
          else if error < -50
                  delta_v = int8(error/50) - d_control;
                  delta_v = max(delta_v, -3);
                  turn(s, speed + delta_v, speed)
              else
                  go(s, speed)
              end
          end
       else if percepts(1) > 100
          old_vleft = vleft;
          old_error = error;
          error = ideal_distance - percepts(6);
          delta_error = old_error - new_error;
          if delta_error > 0
              d_control = -1;
          else
              d_control = 1;
          end
          if error > 50
              delta_v = int8(error/50) + d_control;
              delta_v = min(delta_v, 3);
              turn(s, speed + delta_v, speed)
          else if error < -50
                  delta_v = int8(error/50) - d_control;
                  delta_v = max(delta_v, -3);
                  turn(s, speed + delta_v, speed)
              else
                  go(s, speed)
              end
          end
           else
            vleft = speed;
            go(s, speed);
           end
       end
    end
    end
end