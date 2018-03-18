%% Initialization
 a = arduino();
   
 tic;
 toc;
 
 thresh = 0.90;                                                           % Threshold for white detection
 vidDevice = imaq.VideoDevice('winvideo', 2, 'YUY2_640x480', ...         % Acquire input video stream
                     'ROI', [1 1 480 480], 'ReturnedColorSpace', 'rgb');
 vidInfo = imaqhwinfo(vidDevice);                                        % Acquire input video property
 hblob = vision.BlobAnalysis('AreaOutputPort', false, ...                % Set blob analysis handling
                                 'CentroidOutputPort', true, ... 
                                 'BoundingBoxOutputPort', true', ...
                                 'MinimumBlobArea', 400, ...
                                 'MaximumCount', 1);
 hshapeinsWhiteBox = vision.ShapeInserter('BorderColor', 'Custom', ...
                                         'CustomBorderColor', [1 0 0]);  % Set white box handling
 htextins = vision.TextInserter('Text', 'Number of White Object(s): %2d', ... % Set text for number of blobs
                                   'Location',[7 2],'Color', [1 1 1], ...% white color
                                   'Font', 'Courier New', 'FontSize', 12);
 htextinsCent = vision.TextInserter('Text', '+      X:%6.2f,  Y:%6.2f', ... % set text for centroid
                                     'LocationSource', 'Input port', ...
                                     'Color', [0 0 0], ...               % black color
                                     'FontSize', 12);
 hVideoIn = vision.VideoPlayer('Name', 'Final Video', ...                % Output video player
                               'Position', [100 100 vidInfo.MaxWidth+20 vidInfo.MaxHeight+30]);

global errSum_x;
errSum_x = 0;
global lastErr_x;
lastErr_x = 0;

global errSum_y;
errSum_y = 0;
global lastErr_y;
lastErr_y = 0;

Setpoint = 240;

kp = 0.27;
ki = 0.011;
kd = 0.012;

pid_range = 300;

global timeChange;
timeChange = 0.04;

sx = servo(a,'D6','MinPulseDuration',7.00e-4,'MaxPulseDuration',2.3e-3)
sy = servo(a,'D9','MinPulseDuration',7.00e-4,'MaxPulseDuration',2.3e-3)

% writePosition(sx, 0.3)
% writePosition(sy, 0.7)
% writePosition(sx, 0.5)
% writePosition(sy, 0.5)

 nFrame = 1;                                                             % Frame number initialization
 
%% Processing Loop
while(nFrame)
    rgbFrame = step(vidDevice);                                          % Acquire single frame
    rgbFrame = flipdim(rgbFrame,2);                                      % obtain the mirror image for displaying
    bwredFrame = im2bw(rgbFrame(:,:,1), thresh);                         % obtain the white component from red layer
    bwgreenFrame = im2bw(rgbFrame(:,:,2), thresh);                       % obtain the white component from green layer
    bwblueFrame = im2bw(rgbFrame(:,:,3), thresh);                        % obtain the white component from blue layer
    binFrame = bwredFrame & bwgreenFrame & bwblueFrame;                  % get the common region
    binFrame = medfilt2(binFrame, [3 3]);                                % Filter out the noise by using median filter
    [centroid, bbox] = step(hblob, binFrame);                            % Get the centroids and bounding boxes of the blobs
    
    rgbFrame(1:15,1:215,:) = 0;                                          % put a black region on the output stream
    vidIn = step(hshapeinsWhiteBox, rgbFrame, bbox);                     % Instert the white box
    
     for object = 1:1:length(bbox(:,1))                                  % Write the corresponding centroids
         
        timeChange = toc;
        tic;
         
        x = centroid(1);
        rx = computepidx(x, kp, ki, kd, Setpoint);
        rx = - rx;
        if rx > pid_range
            rx = pid_range;
        elseif rx < -pid_range
            rx = -pid_range;
        end
        rx
        motor_x = (rx + pid_range)/(pid_range*2);
        writePosition(sx, motor_x);
        
        y = centroid(2);
        ry = computepidy(y, kp, ki, kd, Setpoint);
        if ry > pid_range
            ry = pid_range;
        elseif ry < -pid_range
            ry = -pid_range;
        end
        ry
        motor_y = (ry + pid_range)/(pid_range*2);
        writePosition(sy, motor_y);

        vidIn = step(htextinsCent, vidIn, [centroid(object,1) centroid(object,2)],...
            [centroid(object,1)-6 centroid(object,2)-9]); 
        
     end
    vidIn = step(htextins, vidIn, uint8(length(bbox(:,1))));             % Count the number of blobs
    step(hVideoIn, vidIn);                                               % Output video stream
    nFrame = nFrame+2;
end

%% Clearing Memory
 release(hVideoIn);                                                      % Release all memory and buffer used
 release(vidDevice);
 clc;