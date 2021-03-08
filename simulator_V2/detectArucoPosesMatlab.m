function [marker_nums_out, marker_centers_out, marker_orientations_out, ...
    marker_corners_out] = detectArucoPosesMatlab(imgRGB, marker_length, ...
    cameraParams, arucoDict)
        
    aruco = ArucoDetc;

    imgBW = aruco.imgPrep(imgRGB);
    marker_corners = aruco.cmptMarkerProposal(imgBW);
    marker_corners = aruco.ensureCW(marker_corners);
    marker_total = size(marker_corners,2);
    marker_nums = zeros(1,marker_total);
    for marker = 1:marker_total
        [marker_nums(1,marker)] = aruco.identifyMarkerSingle(marker_corners(:,marker), rgb2gray(imgRGB), arucoDict);  
    end
    [marker_corners, marker_nums, marker_total] = aruco.filterDetectedMarkers(marker_corners, marker_nums);
    [marker_orientations, marker_centers] = aruco.estimatePoseMarkers(marker_corners, marker_length, marker_nums, cameraParams);
    aruco.plotPoses(marker_total, marker_centers, marker_orientations, marker_length, marker_nums, cameraParams);
    
    [marker_corners_out, marker_orientations_out, ...
                marker_centers_out, marker_nums_out] = aruco.parseOutput(...
                marker_corners, marker_orientations, marker_centers, ...
                marker_nums, marker_total);
end
