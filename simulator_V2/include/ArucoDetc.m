classdef ArucoDetc
   properties
      bwT = 0.8; %Adaptive threshold sensitivity
      winSize = 15; % BW window sizeS
      % marker size thresholds, w.r.t input img size
      HT = 0.25;
      LT = 0.001;
      % corner sharppening radius, pixel
      rT = 5;
      % edge smoothing threshold, degree
      aT = 10;      
      % plot marker detection and pose
      plotting = false;
   end
   
    methods
        function imgBW = imgPrep(obj, img)
          % check is rgb, grey or binary image
          if ismatrix(img) && max(img, [], 'all') == 1
              imgBW = img;
          elseif  ismatrix(img) && max(img, [], 'all') > 1
              T = adaptthresh(img, obj.bwT, 'NeighborhoodSize', obj.winSize);
              imgBW = imbinarize(img,T);    
          elseif ndims(img) == 3
              imgGrey = rgb2gray(img);
              T = adaptthresh(imgGrey, obj.bwT, 'NeighborhoodSize', obj.winSize);
              imgBW = imbinarize(imgGrey,T);
          end
        end

        function proposals = cmptMarkerProposal(obj, imgBW)
          [h, w] = size(imgBW);
          imgArea = h*w;
          invBW = imcomplement(imgBW);
          stat = regionprops(invBW, 'Area', 'Convexhull');
          if obj.plotting == true
              figure(1)
              imshow(invBW); hold on;
          end
          proposals = [];
          for cnt = 1 : numel(stat)
              area = stat(cnt).Area;
              if imgArea*obj.LT< area && area < imgArea*obj.HT
                vertices = stat(cnt).ConvexHull; % n x 2
%                 [vertices(:, 1), vertices(:, 2)] = ...
%                 poly2cw(vertices(:, 1), vertices(:, 2));
                % reorder corners to and convert to 2 x n
                vertices(end, :) = []; vertices  = vertices.';
                polyU = vertices(1, :); polyV = vertices(2, :);
                % reject polygons touching img boundaries
                if 5 < min(polyU) && max(polyU) < (w-5) && ...
                        5 < min(polyV) && max(polyV) < (h-5)
                    vertices = obj.smoothEdge(vertices, obj.aT);
                    % 3 repetitions for corner refinement
                    for i = 1:3
                        if size(vertices, 2) < 4
                            break
                        end
                        vertices = obj.shappenCorners(vertices, obj.rT);
                    end
                    if size(vertices, 2) == 4
                       proposals = [proposals, vertices(:)];
                       if obj.plotting == true
                           figure(1)                 
                           obj.plotPoly(vertices); 
                       end
                    end
                end
              end
          end
        end

        function vertices = smoothEdge(obj, vertices, T)
        % smoothEdge:  noisey vertices roughly lies on one edge will 
        % be removed 
        n = size(vertices, 2);
        valid_idx = ones(n);
        for i = 1:n
            pt1 = vertices(:, i);
            pt2 = vertices(:, obj.wrapIdx(i+1, n));
            pt3 = vertices(:, obj.wrapIdx(i+2, n));
            vec1 = pt2-pt1;
            vec2 = pt3-pt2;
            theta = acosd(dot(vec1, vec2)/(norm(vec1)*norm(vec2)));
            if theta < T
                valid_idx(obj.wrapIdx(i+1, n)) = 0;
            end
        end
        vertices(:, valid_idx == 0) = [];
        end

        function newVertices = shappenCorners(obj, vertices, radius)
          % shappenCorners: merging two close vertices that define the same
          % object corner
            newVertices = vertices;
            n = size(vertices, 2);
            for i =  1:n
                pt1 = vertices(:, i);
                pt2 = vertices(:, obj.wrapIdx(i+1, n));
                pt3 = vertices(:, obj.wrapIdx(i+2, n));
                pt4 = vertices(:, obj.wrapIdx(i+3, n));
                if norm(pt2-pt3) < radius
                    pt = obj.findIntersect(pt1, pt2, pt3, pt4);
                    if ~isempty(pt)
                        newVertices(:, obj.wrapIdx(i+1, n)) = pt;
                        newVertices(:, obj.wrapIdx(i+2, n)) = [0; 0];
                    end
                end
            end
            newVertices(:, all(~newVertices, 1)) = [];
        end

        function pt = findIntersect(obj, pt1, pt2, pt3, pt4)
          x1 = pt1(1); y1 = pt1(2); x2 = pt2(1); y2 = pt2(2);
          x3 = pt3(1); y3 = pt3(2); x4 = pt4(1); y4 = pt4(2);        
          if ((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4)) ~= 0
              px = ((x1*y2-y1*x2)*(x3-x4) - (x1-x2)*(x3*y4-y3*x4))/...
                  ((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4));
              py = ((x1*y2-y1*x2)*(y3-y4) - (y1-y2)*(x3*y4-y3*x4))/...
                  ((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4));
              pt = [px; py];
          else
              pt = [];
          end
        end

        function newIdx = wrapIdx(obj, idx, len)
          if idx > len
              newIdx = rem(idx, len);
          else
              newIdx = idx;
          end
        end

        function plotPoly(obj, corners)
          corners = [corners, corners(:, 1)];
          U = corners(1, :);
          V = corners(2, :);
          plot(U, V, 'linewidth', 2); 
        end
      
        function [corners, ids, total] = filterDetectedMarkers(obj, corners, ids)
            % corners = 8xN matrix of 2D image coordinates
            % ids = 1xN matrix of id integers

            total = length(ids);
            if isempty(corners)
                return
            end

            % mark markers that will be removed
            toRemove = false(size(corners,2),1)';
            atLeastOneRemove = false;
            % remove repeated markers with same id, if one contains the other (double border bug)
            for i=1:size(corners,2)
                for j=1:size(corners,2)
                    if i~=j
                        % check if first marker is inside second
                        inside = true;
                        marker1 = corners(:,j);
                        marker2 = corners(:,i);
                        if ~inpolygon(marker1(1:2:end),marker1(2:2:end),marker2(1:2:end),marker2(2:2:end))
                            inside = false;
                            break
                        end
                        if inside
                            if isnan(ids(i))
                                toRemove(i) = true;
                            else
                                toRemove(j) = true;
                            end
                            atLeastOneRemove = true;
                            continue
                        end
                        % check if second marker is inside first
                        inside = true;
                        marker1 = corners(:,i);
                        marker2 = corners(:,j);
                        if ~inpolygon(marker1(1:2:end),marker1(1:2:end),marker2(1:2:end),marker2(2:2:end))
                            inside = false;
                            break
                        end
                        if inside
                            if isnan(ids(j))
                                toRemove(j) = true;
                            else
                                toRemove(i) = true;
                            end
                            atLeastOneRemove = true;
                            continue
                        end  
                    end
                end
            end   
            % parse output
            if atLeastOneRemove
                filteredIds = [];
                filteredCorners = [];
                for i=1:length(toRemove)
                    if ~isnan(i)
                        if ~toRemove(i)
                            filteredCorners = [filteredCorners corners(:,i)];
                            filteredIds = [filteredIds ids(:,i)];
                        end
                    end
                end   
                corners = filteredCorners;
                ids = filteredIds;
                total = length(filteredIds);
            end
        end
    
       function [rvecs, tvecs] = estimatePoseMarkers(obj, corners, markerLength, marker_nums, cameraParams)
            % corners = 8xN matrix of 2D image coordinates
            % markerLength = float (length of a marker in meters?)

            markerObjPoints = obj.getSingleMarkerObjectPoints(markerLength);
            nMarkers = size(corners,2);
            rvecs = zeros(nMarkers,3);
            tvecs = zeros(nMarkers,3);

            for i=1:nMarkers
                if ~isnan(marker_nums(i))
                   [rvecs(i,:),tvecs(i,:)] = obj.solvePnP(markerObjPoints, corners(:,i), cameraParams);
                end
            end
            
        end

        function objPoints = getSingleMarkerObjectPoints(obj, markerLength)
            % objPoints = 4x2 (4 corner points in 3D with Z=0)

            objPoints = zeros(4,2);
            % set coordinate system in the middle of the marker, with Z pointing out
            objPoints(1,1:2) = [-markerLength/2.0, markerLength/2.0];
            objPoints(2,1:2) = -[markerLength/2.0, markerLength/2.0];
            objPoints(3,1:2) = [markerLength/2.0, -markerLength/2.0];
            objPoints(4,1:2) = -[-markerLength/2.0, -markerLength/2.0];
        end

        function [rvecs,tvecs] = solvePnP(obj, markerObjPoints, corners, cameraParams)
            imagePoints = zeros(4,2);

            imagePoints(1,1) = corners(3);
            imagePoints(2,1) = corners(5);
            imagePoints(3,1) = corners(7);
            imagePoints(4,1) = corners(1);
            imagePoints(1,2) = corners(4);
            imagePoints(2,2) = corners(6);
            imagePoints(3,2) = corners(8);
            imagePoints(4,2) = corners(2);
                       
            worldPoints = markerObjPoints;
             
            % Compute homography.
            A = cameraParams.IntrinsicMatrix';
            H = obj.computeHomography(worldPoints,imagePoints);
            H = H';
            h1 = H(:, 1);
            h2 = H(:, 2);
            h3 = H(:, 3);

            lambda = 1 / norm(A \ h1);

            % Compute rotation
            r1 = A \ (lambda * h1);
            r2 = A \ (lambda * h2);
            r3 = cross(r1, r2);
            rMat = [r1'; r2'; r3'];

            % Compute translation vector.
            tvec = (A \ (lambda * h3))';             

            rvecs = rotationMatrixToVector(rMat);
            tvecs = tvec';
        end 
        
        
        function [idx] = identifyMarkerSingle(obj, candidate, img, dict)
            assert(all(size(candidate)==[8,1]));

            % Check border bits
            [bits] = obj.extractBits(candidate, img);
            %maxErroneousBitsInBorderRate = 0.35;
            maxBorderWhitePix = 5.6;

            if obj.getBorderErrors(bits) > maxBorderWhitePix
                idx = NaN;
                return;
            end

            innerBits = bits(2:end-1, 2:end-1);
            [idx, ~] = obj.identifyMarkerBits(innerBits, dict);

        end

        function whiteCount = getBorderErrors(obj, bits)
            n = size(bits,1)-2;
            whiteCount = sum(bits(1,1:n+1)) + sum(bits(1:n+1,n+2)) + sum(bits(n+2,2:n+2)) + sum(bits(2:n+2,1));
        end
        
        function [bits] = extractBits(obj, candidate, img)
            % Finds the bits in an aruco marker given the full image and the corners of
            % the marker in that image.
            % Candidate is 8x1 double array listing corner coordinates
            % [x1;y1;x2;y2;...;x4;y4] These must be in clockwise order.
            % img is the full-size input image.
            assert(size(img,3) == 1); % img is grey
            assert(all(size(candidate)==[8,1]));

            markerSizeWithBorders = 4+2*1;
            cellSize = 4;
            resultImgSize = cellSize * markerSizeWithBorders;

            % Transform marker image
            cornersOriginal = reshape(candidate, [2,4])';
            cornersAligned = [1,1;
                              resultImgSize,1;
                              resultImgSize,resultImgSize;
                              1,resultImgSize];

            H = obj.computeHomography(cornersOriginal, cornersAligned);
            resultImg = obj.quickImWarp(img, inv(H'), resultImgSize);

            bits = zeros(markerSizeWithBorders);

            % Check if otsu can be applied
            innerRegion = resultImg(cellSize/2+1:(end-cellSize/2),cellSize/2+1:(end-cellSize/2));
            mu = mean(innerRegion,'all');
            sig = std(innerRegion,0,'all');

            if (sig < 5.0)
                if (mu > 127)
                    bits = ones(size(bits));
                else
                    bits = zeros(size(bits));
                end

                return;
            end

            % Apply otsu method
            resultImg = uint8(resultImg);
            T = graythresh(resultImg);
            resultImg = imbinarize(resultImg,T);

            % Compute bits
            cellMarginPix = floor(0.13 * cellSize);
            for y = 1:markerSizeWithBorders
                startY = (y-1)*cellSize + cellMarginPix +1;
                for x = 1:markerSizeWithBorders
                    startX = (x-1)*cellSize + cellMarginPix +1;
                    cellSquare = resultImg(startY:(startY+cellSize-2*cellMarginPix-1), startX:(startX+cellSize-2*cellMarginPix-1));
                    whitePix = sum(cellSquare,'all');
                    if whitePix > numel(cellSquare)/2
                        bits(y,x) = 1;
                    end
                end
            end
        end
        
        function T = computeHomography(obj, points1, points2)
            classToUse = class(points1);
            numPts = size(points1, 1);
            constraints = zeros(2*numPts, 5, classToUse);
            constraints(1:2:2*numPts, :) = [-points1(:, 2), points1(:, 1), ...
                zeros(numPts, 1), -ones(numPts,1), points2(:,2)];
            constraints(2:2:2*numPts, :) = [points1, ones(numPts,1), ...
                zeros(numPts, 1), -points2(:,1)];
            [~, ~, V] = svd(constraints, 0);
            h = V(:, end);
            T = coder.nullcopy(eye(3, classToUse));
            T(:, 1:2) = [h(1:3), [-h(2); h(1); h(4)]] / h(5);
            T(:, 3)   = [0; 0; 1];
        end      

        function resultImg = quickImWarp(obj, img, H, squareSize)
            % Applies a homography H to a given image and returns the result in a
            % square of given size.
            % img is the full size image
            % H is the homography matrix such that H*p_out = p_in
            % squareSize is the integer width and height of the output image.
            % This method was implemented to be simpler and quicker than MATLAB's built
            % in imwarp.
            assert(squareSize > 0);
            assert(rem(squareSize,1) == 0);

            resultImg = zeros(squareSize);

            for y = 1:squareSize
                for x = 1:squareSize
                    queryPt = H*[x;y;1];
                    queryX = min(round(queryPt(1)/queryPt(3)),size(img,2));
                    queryX = max(queryX,1);
                    queryY = min(round(queryPt(2)/queryPt(3)),size(img,1));
                    queryY = max(queryY,1);
                    resultImg(y,x) = img(queryY,queryX);
                end
            end
        end
        
        function corners = ensureCW(obj, corners)
            %ENSURECW Makes sure an array of corners is always in clockwise order
            % Corners is an 8xn double array
            if isempty(corners)
                return
            end
            assert(size(corners,1) == 8)

            n = size(corners,2);
            for i = 1:n
                c = corners(:,i);
                v1 = c(5:6) - c(1:2);
                v2 = c(7:8) - c(3:4);

                ccwCheck = v2(1)*v1(2) - v1(1)*v2(2);

                if ccwCheck > 0
                    c([3,4,7,8]) = c([7,8,3,4]);
                    corners(:,i) = c;
                end
            end
        end
        
        function [idx, rotation] = identifyMarkerBits(obj, bits, dict)
            % Given the bits of a marker and a dictionary, find the corresponding id
            % number and marker rotation.
            assert(all(size(bits)==[4,4]));

            byteList = obj.getByteList(bits);

            for rot = 1:4
                rotByte = byteList(rot,:);
                match = sum((rotByte == dict),2);
                [match,idxRaw] = max(match);

                idx = floor((idxRaw - 1) / 4);
                rotation = rem(idxRaw-1,4);
            end

            if (match < 15)
                idx = NaN;
                rotation = NaN;
                return
            end

        end

        function byteList = getByteList(obj, bits)
            nBytes = floor((numel(bits) + 8 -1) / 8);
            byteList = uint8(zeros(1, nBytes, 4));

            rot0 = logical(bits');
            rot0 = rot0(:);

            rot1 = logical(rot90(bits,1)');
            rot1 = rot1(:);

            rot2 = logical(rot90(bits,2)');
            rot2 = rot2(:);

            rot3 = logical(rot90(bits,3)');
            rot3 = rot3(:);

            byteList = [rot0'; rot1'; rot2'; rot3'];
        end
        
        function [] = plotPoses(obj, marker_total, marker_centers, ...
                marker_orientations, marker_length, marker_nums, cameraParams)
            if obj.plotting == true
                % plot marker poses
                for marker = 1:marker_total
                    if ~isnan(marker_nums(marker))
                        worldPoints = zeros(4,3);
                        R = rotationVectorToMatrix(marker_orientations(marker,:));
                        worldPoints(4-3,:) = marker_centers(marker,:);
                        worldPoints(4-2,:) = marker_centers(marker,:) + R(:,1)'*2*marker_length;
                        worldPoints(4-1,:) = marker_centers(marker,:) + R(:,2)'*2*marker_length;
                        worldPoints(4,:) = marker_centers(marker,:) + R(:,3)'*2*marker_length;
                        imagePoints = worldToImage(cameraParams,eye(3),zeros(3,1),worldPoints);
                        plot([imagePoints(1,1) imagePoints(2,1)],[imagePoints(1,2),imagePoints(2,2)] ,'Color','r');
                        plot([imagePoints(1,1) imagePoints(3,1)],[imagePoints(1,2),imagePoints(3,2)] ,'Color','g');
                        plot([imagePoints(1,1) imagePoints(4,1)],[imagePoints(1,2),imagePoints(4,2)] ,'Color','b');
                    end
                end
            end
        end
        
        function [marker_corners_out, marker_orientations_out, ...
                marker_centers_out, marker_nums_out] = parseOutput(obj,...
                marker_corners, marker_orientations, marker_centers, ...
                marker_nums, marker_total)

            % parse output so that it matches OpenCV output and NaN ideas 
            % are removed
            marker_nums_out = [];
            marker_corners_x = [];
            marker_corners_y = [];
            marker_orientations_x = [];
            marker_orientations_y = [];
            marker_orientations_z = [];
            marker_centers_x = [];
            marker_centers_y = [];
            marker_centers_z = [];
            for i=1:marker_total
                if ~isnan(marker_nums(i))
                    marker_nums_out = [marker_nums_out; marker_nums(i)];
                    marker_corners_x = [marker_corners_x; round(marker_corners(1:2:end,i))];
                    marker_corners_y = [marker_corners_y; round(marker_corners(2:2:end,i))];
                    marker_orientations_x = [marker_orientations_x; marker_orientations(i,1)];
                    marker_orientations_y = [marker_orientations_y; marker_orientations(i,2)];
                    marker_orientations_z = [marker_orientations_z; marker_orientations(i,3)];
                    marker_centers_x = [marker_centers_x; marker_centers(i,1)];
                    marker_centers_y = [marker_centers_y; marker_centers(i,2)];   
                    marker_centers_z = [marker_centers_z; marker_centers(i,3)];  
                end    
            end
            marker_corners_out(:,:,1) = marker_corners_x;
            marker_corners_out(:,:,2) = marker_corners_y;
            marker_orientations_out(:,:,1) = marker_orientations_x;
            marker_orientations_out(:,:,2) = marker_orientations_y;
            marker_orientations_out(:,:,3) = marker_orientations_z;
            marker_centers_out(:,:,1) = marker_centers_x;
            marker_centers_out(:,:,2) = marker_centers_y;
            marker_centers_out(:,:,3) = marker_centers_z;
            
            
        end
        
   end
end