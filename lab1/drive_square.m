function drive_square(pb, sideLength)
% DRIVE_SQUARE send a sequence of commands to the pibot to make it drive in a square.

% pb is the pibot instance to send commands
% sideLength is the length of each side of the square to drive
for i = 0:3
    pb.setVelocity([95 95], 2*sideLength);
    pb.setVelocity([-23 23], 1);

end

end