s = openConnection;

% Very simple MATLAB controller for Webots
% File: control.m
% Date: 21/09/2011          
% Description: This controller will open MATLAB and let you input
%              commands from the keyboard
% Author: Simon Smith (artificialsimon@ed.ac.uk)        
% Modifications: 
% Use: command to set motor speed 1 to left motor, -1 to right motor
%       wb_differential_wheels_set_speed(1, -1)
% After sending a setting command, the controller have to resume by
% sending "return" in the keyboard input
% This will allow a sensor value update as well

TIME_STEP = 64;
N = 8;
LEFT = 0;
RIGHT = 1;
WHEEL_RADIUS = 0.008;
BODY_LENGTH = 0.053;

% Calling MATLAB desktop versionwb_differential_wheels_set_speed(1, -1);
% main loop:
% perform simulation steps of TIME_STEP milliseconds
% and leave the controll to the keyboard

ideal_distance = 300;
K = 0.001;
speed = 10;
vleft = speed;
turn_speed = 5;
error = 0;

while wb_robot_step(TIME_STEP) ~= -1

  % read all distance sensors
       for i=1:N
           sensor_values(i) = wb_distance_sensor_get_value(ps(i));
       end
  % display all distance sensors values
  
  %sensor_values

  object_ahead_left = (sensor_values(1) + sensor_values(2) + sensor_values(3)) > 10 && sensor_values(2) > 0;


  if object_ahead_left
      wb_differential_wheels_set_speed(1, -1);
  else
      [vr,error] = prop_control(old_vr, sensor_values, old_error);
      wb_differential_wheels_set_speed(vl, vr);
      old_error = error;
      old_vr = vr;
  end
  
  
  
  % ODOMETRY
  
  encoders = [0,0];
  
  encoders(1) = wb_differential_wheels_get_left_encoder();
  encoders(2) = wb_differential_wheels_get_right_encoder();
  
  encoders_step = encoders - encoders_old;
  
  arcs = (encoders_step * WHEEL_RADIUS) / 100;
  % NOTE - velocities in metres/s
  
  distance = 0.5 * (arcs(1) + arcs(2));
  total_distance = total_distance + distance;
  phi = phi + (arcs(1) - arcs(2))/BODY_LENGTH;
  
  x = x + distance * sin(phi);
  y = y + distance * cos(phi);
  
  encoders_old = encoders;
  
  if total_distance > 0.5 && abs(x) < 0.01 && abs(y) < 0.01
      disp('back home!')
      break;
  end
  
  % CAMERA
  
  rgb = wb_camera_get_image(CAMERA);
  hsv = rgb2hsv (rgb);
  for i=1:WIDTH
    for j=1:HEIGHT
      if hsv(i,j,1)>(0.15) & hsv(i,j,1)<(0.45)
        hsv(i,j,1) = 1;
      else
        hsv(i,j,1) = 0;
      end;
    end;
  end;
  
  labels = bwlabel (hsv(:,:,1));
  regions = regionprops(labels, 'Area');
  array = [regions.Area];
  
  if size(array) > 0
    disp(array(1));
    if array(1) > 250
      disp('intruder alert!')
      wb_differential_wheels_set_speed(0, 0);
      wb_robot_step(TIME_STEP);
      break;
    end;
  end;
  %imshow (hsv(:,:,1));
  
  %set_speeds(left_speed,right_speed);
  %if sensor_values(1)+sensor_values(2)+sensor_values(3)>10
  %  wb_differential_wheels_set_speed(1, -1);
  %elseif sensor_values(4)+sensor_values(5)+sensor_values(6)>10
  %  wb_differential_wheels_set_speed(-1, 1);
  %else
  %   wb_differential_wheels_set_speed(1, 1);
  %end;
  %control goes to the keyboard
  
end

keyboard;