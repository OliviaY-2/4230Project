% MTRN4230 Object detection
% Author: Lucas Way
% Additional contributors: Diana Huang, Arthur Ching
% First made (DD/MM/YYY): 28/07/2020
%{
Connect to a ROS topic, obtain an image, classify the objects using YOLOv3
deep learning method, find location of object using point cloud, return
desired centroids.
Reference:
Object Detection Using YOLO v3 Deep Learning
https://au.mathworks.com/help/vision/examples/object-detection-using-yolo-v3-deep-learning.html
Instructions:
    Load variables from gridMix_Network2.mat. These are used for
    yolov3 deep neural network detection function. This folder can be
    obtained by saving variables 'net','anchorBoxes' and 'classNames' after
    training a network using YOLOv3.m
    Also remember to add Parallel Computing, Machine learning, deep learning,
    image processing and computer vision Toolboxes.
    Choose ipaddress to connect to ROS and Gazebo.
    Current set up can use .mat files holding data saved in RGBG_Data\Mix folder 
    or use images obtained online using ROS.
    To use ROS, set 'ROS_Used' to 1.
    Run code. 

Edit History:
28/07/2020 created file
29/07/2020 added point cloud stuff
08/03/2020 load data from .mat file. 
03/08/2020 ROI point cloud segmentation
04/08/2020 Clean up ROI point cloud segmentation, calculate centroids, if
    statement to check if any objects were detected, choose colour and
    shape, GUI
11/08/2020 Set up function to publish info to ros node. Now checks how many
    objects are requested. Wait for ros topic MATLAB to return message
    saying task is complete.
12/08/2020 Changed point cloud bounding box scaling. Tested successfully 
    with trajectory planner python script, can receive
    images + depth data and send coordinates.
16/08/2020 wait for arm to send message of completed task before requesting
    more shapes.
18/08/2020 uses newly trained network (gridMix_Network2.mat)
%}
function MTRN4230_ObjectDetection_GUI()
    % Set ipaddress for ROS connection
    ipaddress = '192.168.56.101';
    % Choose if Ros connection is used to obtain data
    ROS_Used = 1;
       
    % option to automatically check for GPU.
    executionEnvironment = "auto";
    % Choose .mat file with pretrained network and variables
    DetectVariables = load('gridMix_Network2.mat');
    % Deep learning neural network variables
    Net = DetectVariables.network.net;
    NetworkOutputs = ["conv2Detection1"
    "conv2Detection2"
    ];
    AnchorBoxes = DetectVariables.network.anchorBoxes;
    AnchorBoxMasks = {[1,2,3]
        [4,5,6]
        };
    ClassNames = DetectVariables.network.classNames;
    imgSize = [227 227];
    % Classification threshold values. Change to increase possible range of
    % objects to classify if scores are low. 
    ConfidenceThreshold = 0.65;
    OverlapThreshold = 0.3;

    disp('Obtain images for classification');
    
    if ROS_Used == 1
        % Initialise ROS
        rosshutdown;
        disp('Initialising Ros Subscriber');
        rosinit(ipaddress);
    else
        % Otherwise, load information from a .mat file.
        loadMat = load('.\RGBD_Data\Mix\159_Mix.mat');
        img = loadMat.image;   
        ptCloud = loadMat.xyz;
    end

    % Create loop to continue asking for user inputs.
    flag = 1;
    while flag == 1
        % Wait for input from command line
        quest = 'Do you want to process a new image?';
        userInput = questdlg(quest,'userInput');
        %userInput = input("Do you want to process a new image?: ", 's');

        switch userInput
            case 'No'
                % Exit loop
                flag = 0;
            case 'Cancel'
                % Exit loop
                close all;
                flag = 0;
            case 'Yes'
                % Obtain image from Gazebo using ROS topics
                if ROS_Used == 1
                    [img, ptCloud] = obtainImage();
                end
                %number of picks selected
                answer = inputdlg('Enter number of picks:','Picks Input',[1 35],{''});
                no_of_picks = str2double(answer{1});
                if no_of_picks > 20
                    disp('Too Many Picks, choose a number less than or equal to 20');
                    continue
                elseif isnan(no_of_picks)
                    disp('Invalid Input');
                    continue
                end
                 
                % Choose which colour is desired
                answer = inputdlg('Enter colour: Red, Green, Blue, All','Colour Input',[1 35],{''});
                colourInput = char(answer(1,1));
                switch colourInput
                    case {'Red', 'Green', 'Blue', 'All'}
                        desiredColour = colourInput;
                    otherwise
                        disp('Invalid colour');
                        continue;
                end
                
                % Choose which shape is desired
                answer = inputdlg('Enter shape: Cube, Cylinder, Tri, All','Shape Input',[1 35],{''});
                shapeInput = char(answer(1,1));
                switch shapeInput
                    case {'Cube', 'Cylinder', 'Tri', 'All'}
                        desiredShapes = strcat('_',shapeInput);
                    otherwise
                        disp('Invalid shape');
                        continue;
                end
                              
                % Object classification
                disp('Classify objects in image');
                % HSV mask image to remove unwanted colours, grey floor, grey parts of arm
                % and purple box
                switch desiredColour
                    case 'Red'
                        [~,maskedRGBImage] = RedMask(img);                        
                    case 'Green'
                        [~,maskedRGBImage] = GreenMask(img);                        
                    case 'Blue'
                        [~,maskedRGBImage] = BlueMask(img);                        
                    case 'All'
                        [~,maskedRGBImage] = createMask(img);
                end
                
                % Resize and modify image to input into network.
                I_resize = imresize(maskedRGBImage,imgSize);
                I = im2single(I_resize);
                % Classify objects in image
                [Bboxes, ~, Labels] = classifyImage(I,executionEnvironment, ...
                    Net, NetworkOutputs,AnchorBoxes,AnchorBoxMasks, ConfidenceThreshold, ...
                    OverlapThreshold, ClassNames);
                % Remove bounding box values if specified
                if ~strcmp(desiredShapes,'_All')
                    % determine if desired object shape was detected by
                    % checking each label
                    desiredLabels = zeros(length(Labels),1);
                    for LabelCnt = 1:length(Labels)
                        % obtain label string
                        OneLabel = char(Labels(LabelCnt,1));
                        % If label contains name of shape, find all labels with
                        % that name and add logical value to desiredLabels
                        if contains(OneLabel, desiredShapes)
                            desiredLabels = desiredLabels |(Labels(:,1) == OneLabel);
                        end
                    end
                    % Remove all bounding boxes that were not in desired shape
                    % list
                    Bboxes(~desiredLabels,:) = [];
                end
                % If desired objects were classified, find centroid in point cloud
                if ~isempty(Bboxes)
                    Centroids = calculateCentroids(ptCloud,Bboxes);
                    centroidNum = size(Centroids,1);
                    if no_of_picks > centroidNum
                        disp(['Camera could only find ',num2str(centroidNum),' object(s)']);
                        no_of_picks = centroidNum;
                    end
                    %if ROS_Used == 1
                    disp("Publishing Data");
                    publishInfo(Centroids(1:no_of_picks,:));
                        % Wait for a return message, confirming the task
                        % was completed
                    TaskFlag = rossubscriber('/TaskComplete');
                    TaskComplete = receive(TaskFlag);
                    disp(TaskComplete.Data);
                    %end
                else
                    disp('No Objects Found');
                end
                
            otherwise
                % Print for invalid inputs
                disp("Invalid. Options include 'yes' or 'no'."); 
        end
    end
end

% Obtain image, depth data and camera pose from ROS topic
function [image, depthxyz] = obtainImage()
    disp("Getting new image..");
    tic
    % Obtain RGB image
    imsub = rossubscriber('/camera/color/image_raw');
    image_data = receive(imsub);
    image = readImage(image_data);
    % Obtain depth values
    pcsub = rossubscriber('/camera/depth/points');
    depthxyz_data = receive(pcsub);
    depthxyz = readXYZ(depthxyz_data);
    toc
end
% Publish centroid values to ROS topic. Values are adjusted to make
% coordinates relative to base of robot arm, not the camera.
function publishInfo(CentroidList)
    
    disp(CentroidList);
    chatterpub = rospublisher('/MATLAB', 'std_msgs/String');
    pause(1);
    for CentroidCnt = 1:size(CentroidList,1)
        chattermsg = rosmessage(chatterpub);
        % Adjust x coordinate and publish
        chattermsg.Data = num2str(CentroidList(CentroidCnt,1) * -1.0);
        send(chatterpub,chattermsg)
        disp(chattermsg.Data);
        % Adjust y coordinate and publish
        chattermsg.Data = num2str(CentroidList(CentroidCnt,2) - 0.5);
        send(chatterpub,chattermsg)
        disp(chattermsg.Data);
    end
end

% Function to classify objects in image using pre-trained YOLOv3 network
function [bboxes, scores, labels] = classifyImage(I,execution_Environment, ...
    net, networkOutputs,anchorBoxes,anchorBoxMasks, confidenceThreshold, ...
    overlapThreshold, classNames)
    % Convert to dlarray.
    XTest = dlarray(I,'SSCB');
    % If GPU is available, then convert data to gpuArray.
    if (execution_Environment == "auto" && canUseGPU) || execution_Environment == "gpu"
        XTest = gpuArray(XTest);
    end
    [bboxes, scores, labels] = yolov3Detect(net,XTest, networkOutputs,anchorBoxes,anchorBoxMasks, confidenceThreshold, ...
        overlapThreshold, classNames);
    % Display the detections on image.
    if ~isempty(scores)
        I = insertObjectAnnotation(I, 'rectangle', bboxes, labels);
    end
    figure
    imshow(I)
end

% Function to calculate the centroid of objects in point cloud 
function centroid = calculateCentroids(pt,Bbox)
    
    % Remove the floor. Assume the point furthest away in the z axis 
    % is the floor
    floor = max(pt(:,3)) - 0.05;
    pt(pt(:,3) >= floor,:) = [];

    % Convert to type pointcloud
    ptCloudHiRes = pointCloud(pt);
    % Reduce the number of samples to aid computation time
    PTCloud = pcdownsample(ptCloudHiRes, 'gridAverage', 0.005);
    
    % Convert Bounding box coordinates to regions of interest coordinates
    BBox_size = size(Bbox);
    zMin = zeros(BBox_size(1),1);
    zMax = zeros(BBox_size(1),1) + 0.8;
    ROIs = [Bbox(:,1), Bbox(:,1) + Bbox(:,3), ...
    Bbox(:,2), Bbox(:,2) + Bbox(:,4)];
    % Convert scale to match point cloud axis
    ROIs_pt = [(ROIs(:,1:2) .* 0.8/227) - 0.4, ...
        (ROIs(:,3:4) .* 0.75/227)- 0.375,zMin,zMax];

    centroid = zeros(BBox_size(1),3);
    minDistance = 0.01;
    figure();
    title('Point Cloud');
    hold on;
    for roiCnt = 1:BBox_size(1)  
        tic
        % Obtain small section of point cloud based on ROIs
        indices = findPointsInROI(PTCloud,ROIs_pt(roiCnt,:));
        ptCloudROI = select(PTCloud,indices);
        
        % label the groups of points that form a cluster
        labels = pcsegdist(ptCloudROI,minDistance);
        ptROILocations = ptCloudROI.Location;
        % Find the most dominant label
        mainLabel = mode(labels,1);
        % Keep points for most dominant object/label, remove everything
        % else
        labelobjects = (labels(:,1) == mainLabel);
        ptROILocations(~labelobjects,:) = [];
        
        % find the z coordinate for the top most face
        topFace = min(ptROILocations(:,3));
        % Remove points that correspond to the side of the objects by
        % keeping top most face only
        ptROILocations(ptROILocations(:,3) > (topFace + 0.005),:) = [];
        % calculate centroids
        centroid(roiCnt,1) = mean(ptROILocations(:,1));
        centroid(roiCnt,2) = mean(ptROILocations(:,2)); 
        centroid(roiCnt,3) = mean(ptROILocations(:,3)); 
        % show point cloud clusters found using ROI                
        pcshow(ptROILocations,'g');       
        toc
    end
    % show centroid locations
    pcshow(centroid,[1 0 0]);
    hold off;
end

% The yolov3Detect function detects the bounding boxes, scores,
% and labels in an image.
function [bboxes,scores,labels] = yolov3Detect(net, XTest, networkOutputs, anchors, anchorBoxMask, confidenceThreshold, overlapThreshold, classes)


imageSize = size(XTest,[1,2]);

% Find the input image layer and get the network input size.
networkInputIdx = arrayfun( @(x)isa(x,'nnet.cnn.layer.ImageInputLayer'), net.Layers);
networkInputSize = net.Layers(networkInputIdx).InputSize;

% Predict and filter the detections based on confidence threshold.
predictions = yolov3Predict(net,XTest,networkOutputs,anchorBoxMask);
predictions = cellfun(@ gather, predictions,'UniformOutput',false);
predictions = cellfun(@ extractdata, predictions, 'UniformOutput', false);
tiledAnchors = generateTiledAnchors(predictions(:,2:5),anchors,anchorBoxMask);
predictions(:,2:5) = applyAnchorBoxOffsets(tiledAnchors, predictions(:,2:5), networkInputSize);
[bboxes,scores,labels] = generateYOLOv3Detections(predictions, confidenceThreshold, imageSize, classes);

% Apply suppression to the detections to filter out multiple overlapping
% detections.
if ~isempty(scores)
    [bboxes, scores, labels] = selectStrongestBboxMulticlass(bboxes, scores, labels ,...
        'RatioType', 'Union', 'OverlapThreshold', overlapThreshold);
end
end

% Predict the output of network and extract the confidence, x, y,
% width, height, and class.
function YPredCell = yolov3Predict(net,XTrain,networkOutputs,anchorBoxMask)
YPredictions = cell(size(networkOutputs));
[YPredictions{:}] = predict(net, XTrain);
YPredCell = extractPredictions(YPredictions, anchorBoxMask);

% Apply activation to the predicted cell array.
YPredCell = applyActivations(YPredCell);
end

function YPredCell = applyActivations(YPredCell)
YPredCell(:,1:3) = cellfun(@ sigmoid ,YPredCell(:,1:3),'UniformOutput',false);
YPredCell(:,4:5) = cellfun(@ exp,YPredCell(:,4:5),'UniformOutput',false);
YPredCell(:,6) = cellfun(@ sigmoid ,YPredCell(:,6),'UniformOutput',false);
end

function predictions = extractPredictions(YPredictions, anchorBoxMask)
predictions = cell(size(YPredictions, 1),6);
for ii = 1:size(YPredictions, 1)
    numAnchors = size(anchorBoxMask{ii},2);
    % Confidence scores.
    startIdx = 1;
    endIdx = numAnchors;
    predictions{ii,1} = YPredictions{ii}(:,:,startIdx:endIdx,:);
    
    % X positions.
    startIdx = startIdx + numAnchors;
    endIdx = endIdx+numAnchors;
    predictions{ii,2} = YPredictions{ii}(:,:,startIdx:endIdx,:);
    
    % Y positions.
    startIdx = startIdx + numAnchors;
    endIdx = endIdx+numAnchors;
    predictions{ii,3} = YPredictions{ii}(:,:,startIdx:endIdx,:);
    
    % Width.
    startIdx = startIdx + numAnchors;
    endIdx = endIdx+numAnchors;
    predictions{ii,4} = YPredictions{ii}(:,:,startIdx:endIdx,:);
    
    % Height.
    startIdx = startIdx + numAnchors;
    endIdx = endIdx+numAnchors;
    predictions{ii,5} = YPredictions{ii}(:,:,startIdx:endIdx,:);
    
    % Class probabilities.
    startIdx = startIdx + numAnchors;
    predictions{ii,6} = YPredictions{ii}(:,:,startIdx:end,:);
end
end

% Generate tiled anchor offset.
function tiledAnchors = generateTiledAnchors(YPredCell,anchorBoxes,anchorBoxMask)
tiledAnchors = cell(size(YPredCell));
for i=1:size(YPredCell,1)
    anchors = anchorBoxes(anchorBoxMask{i}, :);
    [h,w,~,n] = size(YPredCell{i,1});
    [tiledAnchors{i,2}, tiledAnchors{i,1}] = ndgrid(0:h-1,0:w-1,1:size(anchors,1),1:n);
    [~,~,tiledAnchors{i,3}] = ndgrid(0:h-1,0:w-1,anchors(:,2),1:n);
    [~,~,tiledAnchors{i,4}] = ndgrid(0:h-1,0:w-1,anchors(:,1),1:n);
end
end

% Convert grid cell coordinates to box coordinates.
function tiledAnchors = applyAnchorBoxOffsets(tiledAnchors,YPredCell,inputImageSize)
for i=1:size(YPredCell,1)
    [h,w,~,~] = size(YPredCell{i,1});  
    tiledAnchors{i,1} = (tiledAnchors{i,1}+YPredCell{i,1})./w;
    tiledAnchors{i,2} = (tiledAnchors{i,2}+YPredCell{i,2})./h;
    tiledAnchors{i,3} = (tiledAnchors{i,3}.*YPredCell{i,3})./inputImageSize(2);
    tiledAnchors{i,4} = (tiledAnchors{i,4}.*YPredCell{i,4})./inputImageSize(1);
end
end

% Function used when reading from datastore since images were saved as .mat
% files.
function data = matRead(filename)
    inp = load(filename);
    f = fields(inp);
    data = inp.(f{3});
end

% mask out purple box, grey floor and robot arm, keeping the objects
function [BW,maskedRGBImage] = createMask(RGB)

% Convert RGB image to chosen color space
I = rgb2hsv(RGB);

% Define thresholds for Hue
channel1Min = 0.955;
channel1Max = 0.749;

% Define thresholds for Saturation
channel2Min = 0.201;
channel2Max = 1.000;

% Define thresholds for Value
channel3Min = 0.000;
channel3Max = 1.000;

% Create mask based on chosen histogram thresholds
sliderBW = ( (I(:,:,1) >= channel1Min) | (I(:,:,1) <= channel1Max) ) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BW = sliderBW;

% Initialize output masked image based on input image.
maskedRGBImage = RGB;

% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;

end

% Mask out all shapes except the red ones
function [BW,maskedRGBImage] = RedMask(RGB)

% Convert RGB image to chosen color space
I = rgb2hsv(RGB);

% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.846;
channel1Max = 0.000;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.159;
channel2Max = 1.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.000;
channel3Max = 1.000;

% Create mask based on chosen histogram thresholds
sliderBW = ( (I(:,:,1) >= channel1Min) | (I(:,:,1) <= channel1Max) ) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BW = sliderBW;

% Initialize output masked image based on input image.
maskedRGBImage = RGB;

% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;

end
% Mask out all shapes except the Blues ones
function [BW,maskedRGBImage] = BlueMask(RGB)

% Convert RGB image to chosen color space
I = rgb2hsv(RGB);

% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.402;
channel1Max = 0.761;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.159;
channel2Max = 1.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.000;
channel3Max = 1.000;

% Create mask based on chosen histogram thresholds
sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BW = sliderBW;

% Initialize output masked image based on input image.
maskedRGBImage = RGB;

% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;

end
% Mask out all shapes except the Green ones
function [BW,maskedRGBImage] = GreenMask(RGB)

% Convert RGB image to chosen color space
I = rgb2hsv(RGB);

% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.131;
channel1Max = 0.551;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.159;
channel2Max = 1.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.000;
channel3Max = 1.000;

% Create mask based on chosen histogram thresholds
sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BW = sliderBW;

% Initialize output masked image based on input image.
maskedRGBImage = RGB;

% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;

end


