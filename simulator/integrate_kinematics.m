function [x,y,O] = integrate_kinematics(state, dt, lin_velocity, ang_velocity)
%INTEGRATE_KINEMATICS integrate the kinematics of the robot

%   state is the current state, and has the form [x;y;theta]
%   dt is the length of time for which to integrate
%   lin_velocity is the (forward) linear velocity of the robot
%   ang_velocity is the angular velocity of the robot

%   new_state is the state after integration, also in the form [x;y;theta]
x = state(end,1)+dt*lin_velocity*cos(state(end,3));
y = state(end,2)-dt*lin_velocity*sin(state(end,3));
O = state(end,3)+dt*ang_velocity;

new_state = x,y,O;        
end