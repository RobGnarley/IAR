counter = 1;
%s = serial('/dev/ttyS0');
%fopen(s);
vl = 5;
old_error = 0;
old_vr = 5;

while true
  counter = counter + 1;

  % read all distance sensors
  sensor_values = readIR(s);
    
  
  


  object_ahead_left = (sensor_values(1) + sensor_values(2) + sensor_values(3)) > 400 && sensor_values(2) > 100;


  if object_ahead_left
      turn(s, 1, -1);
  else
      [vr,error] = prop_control(old_vr, sensor_values, old_error);
      turn(s, vl, vr)
      old_error = error;
      old_vr = vr;
  end
end

