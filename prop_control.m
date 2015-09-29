function [new_vr, error] = prop_control(vr, sensor_values, old_error)


% WALL FOLLLOWING
%--------------------
L_IDEAL = 400;
K1 = 0.00001;
K2 = 0.04;

error = L_IDEAL - sensor_values(1);
delta_error = error - old_error;

delta_vr = K1 * error + K2 * delta_error;
new_vr = vr + delta_vr;
%--------------------