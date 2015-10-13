
%s=serial('/dev/ttyS0');
%fopen(s);

go(s,5);
sensor = readIR(s);
while sensor(3) < 200
    sensor = readIR(s);
end

stop(s);
turn(s,-3,3);
while sensor(6) < 350
    sensor = readIR(s);
end
stop(s)
sensor

%fclose(s);