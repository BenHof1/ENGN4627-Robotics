function drive_circle(pb, radius)
% DRIVE_CIRCLE send a sequence of commands to the pibot to make it drive in a circle.

% pb is the pibot instance to send commands
% radius is the radius of the circle to drive
ave =80;
bias = round((ave/100)*15/(radius*2));



%pb.setVelocity(x,2*pi*radius);
for i = 0
    pb.setVelocity([ave-bias,ave+bias], 4*pi*radius);
    %pb.setVelocity([-23 23], 1);

end
end