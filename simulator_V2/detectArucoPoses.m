function [marker_nums, landmark_centres, marker_corners] = detectArucoPoses(image, marker_length, cameraParams, arucoDict)
%DETECTARUCOPOSES Detecting the poses of ARUCO markers in an image.
%
%   [marker_nums, landmark_centres, marker_corners] = DETECTARUCOPOSES(image, marker_length, cameraParams)
%   detects the poses of the ARUCO markers in image, given the side length
%   of the markers.
%
%   You can load cameraParams and arucoDict using:
%   addpath("include")
%   addpath("dictionary")
%   load("arucoDict.mat")
%   load("cameraParameters.mat")
%   
%   IF DETECTION ISN'T PERFORMING AS EXPECTED START HERE
%   You can vary some detection hyperparameters and choose to plot the
%   detected markers and their poses using the properties in ArunoDetc.m

if nargin == 4
    [marker_nums, marker_centres, marker_orientations, marker_corners] = detectArucoPosesMatlab(image, marker_length, cameraParams, arucoDict);
else
    error("You must provide all 4 input arguments")
end

% Reshape the raw outputs 
marker_centres = reshape(marker_centres, size(marker_centres,1), 3);
marker_orientations = reshape(marker_orientations, size(marker_orientations,1), 3);
marker_corners = reshape(marker_corners, 4, size(marker_corners,1)/4, 2);
marker_corners = permute(marker_corners,[2,1,3]);

% Compute the centres of the landmarks, where the landmarks are assumed to
% be cubes with the same ARUCO marker on every side.
number_markers = numel(marker_nums);
landmark_centres = marker_centres;
for i = 1:number_markers
    R = rod2rotm(marker_orientations(i,:)');
    landmark_centres(i,:) = landmark_centres(i,:) - R(:,3)'*marker_length;
end

% Convert landmark centres to the robot frame
R_robot2camera = [0 0 1
                  -1 0 0
                  0 -1 0];
landmark_centres = (R_robot2camera * landmark_centres')';


end

function R = rod2rotm(r)
    % Convert rodrigues vector to SO(3) matrix
    theta = norm(r);
    if (theta < 1e-6)
        R = eye(3);
        return
    end
    r = r / theta;
    R = cos(theta)*eye(3) + (1-cos(theta))*(r*r') + sin(theta) * skew(r);
end

function [e] = skew(x)
    % Return the skew symmetric matrix of a given vector
    e = [0 -x(3) x(2)
         x(3) 0 -x(1)
         -x(2) x(1) 0];
end