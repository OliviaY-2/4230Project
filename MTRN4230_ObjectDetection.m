% MTRN4230 Object detection
% Author: Lucas Way z5164204
% First made (DD/MM/YYY): 28/07/2020
%{
Connect to a ROS topic, obtain an image, classify the objects using YOLOv3
deep learning method, find location of object using point cloud, return
desired centroids.

Reference:
Object Detection Using YOLO v3 Deep Learning
https://au.mathworks.com/help/vision/examples/object-detection-using-yolo-v3-deep-learning.html

Instructions:
    Load variables from YOLOv3_Detect_variables.mat. These are used for
    yolov3 detection function.
    Choose ipaddress to connect to ROS and Gazebo
    Run code. 
    Type 'Yes' to obtain and classify and image
    Type 'No' to end program

Edit History:
28/07/2020 created file

%}
function MTRN4230_ObjectDetection()
    % Set ipaddress for ROS connection
    ipaddress = '192.168.56.101';
    % myFolder = '.\RGBD_Data';
    % loadImages = 'Multiple Objects';
    
    % Deep learning neural network variables
    executionEnvironment = "auto";
    DetectVariables = load('YOLOv3_Detect_variables.mat');
    Net = DetectVariables.net;
    NetworkOutputs = DetectVariables.networkOutputs;
    AnchorBoxes = DetectVariables.anchorBoxes;
    AnchorBoxMasks = DetectVariables.anchorBoxMasks;
    ConfidenceThreshold = DetectVariables.confidenceThreshold;
    OverlapThreshold = DetectVariables.overlapThreshold;
    ClassNames = DetectVariables.classNames;
    imgSize = [227 227];

    % % Colour options: 'all_c', 'red', 'green', 'blue'.
    % desiredColour = 'all_c';
    % % Shape options: 'all_s', 'Cube', 'Cylinder', 'Rectangle'.
    % desireShapes = 'all_s';

    % Obtain an image to classify
    disp('Obtain images for classification');

    %     disp(['Obtain image from ',myFolder]);
    %     imdatastore = imageDatastore(fullfile(myFolder,... 
    %         loadImages),...
    %         'LabelSource', 'foldernames', 'FileExtensions', '.mat','ReadFcn',@matRead);
    %     img = readimage(imdatastore,1);

    % Initialise ROS
    rosshutdown;
    rosinit(ipaddress);
    % Create loop to continue asking for user inputs.
    flag = 1;
    while flag == 1
        % Wait for input from command line
        userInput = input("Process a new image: ", 's');
        switch userInput
            case 'No'
                % Exit loop
                flag = 0;
            case 'Yes'
                % Obtain image from Gazebo using ROS topics
                [img, ptCloud] = obtainImage();
                %Object classification
                disp('Classify objects in image');
                % Resize to input into network.
                I_resize = imresize(img,imgSize);
                I = im2single(I_resize);
                % Classify objects in image
                [Bboxes, Scores, Labels] = classifyImage(I,executionEnvironment, ...
                    Net, NetworkOutputs,AnchorBoxes,AnchorBoxMasks, ConfidenceThreshold, ...
                    OverlapThreshold, ClassNames);
                disp(Bboxes);
                Centroids = calculateCentroids(ptCloud);
                disp(Centroids);
            otherwise
                disp("Invalid"); 
        end
    end
end

function [image, depthxyz] = obtainImage()
    disp("Getting new image..");
    tic
%     blockposes = rossubscriber('/gazebo/link_states');
%     posdata = receive(blockposes);
    
    imsub = rossubscriber('/camera/color/image_raw');
    image_data = receive(imsub);
    image = readImage(image_data);
    pcsub = rossubscriber('/camera/depth/points');
    depthxyz_data = receive(pcsub);
    depthxyz = readXYZ(depthxyz_data);
    toc
    %     % Obtain desired shapes and colours
    %     chatSub = rossubscriber('/chatter');
    %     chat = receive(chatSub);
    %     message = chat.LatestMessage;

end

function [bboxes, scores, labels] = classifyImage(I,execution_Environment, ...
    net, networkOutputs,anchorBoxes,anchorBoxMasks, confidenceThreshold, ...
    overlapThreshold, classNames)
    % Convert to dlarray.
    XTest = dlarray(I,'SSCB');
    % If GPU is available, then convert data to gpuArray.
    if (execution_Environment == "auto" && canUseGPU) || execution_Environment == "gpu"
        XTest = gpuArray(XTest);
    end
    
    tic
    [bboxes, scores, labels] = yolov3Detect(net,XTest, networkOutputs,anchorBoxes,anchorBoxMasks, confidenceThreshold, ...
        overlapThreshold, classNames);
    toc
    % Display the detections on image.
    if ~isempty(scores)
        I = insertObjectAnnotation(I, 'rectangle', bboxes, labels);
    end
    figure
    imshow(I)
end

function centroid = calculateCentroids(pt)
    % Remove the floor. Assume the point furthest away in the z axis 
    % is the floor
    floor = max(pt(:,3)) - 0.05;
    pt(pt(:,3) >= floor,:) = [];

    % Convert to type pointcloud
    ptCloudHiRes = pointCloud(pt);
    % Reduce the number of samples to aid computation time
    PTCloud = pcdownsample(ptCloudHiRes, 'gridAverage', 0.005);
    
    % label the groups of points that form a cluster
    minDistance = 0.01;
    [labels,numClusters] = pcsegdist(PTCloud,minDistance);
    locations = PTCloud.Location;
    
    % find out how many points were labelled and create an array
    % where each 'layer' in allObjects is the coordinates for a 
    % cluster of points. Each cluster is represented as object1.
    numLabels = size(labels(:,1),1);
    %size1 = size1(1);
    object1 = NaN(numLabels,3);
    % allObjects = zeros(numLabels,3,numClusters);
    % objectSizes = zeros(numClusters,1);
    centroid = zeros(numLabels,3);
    validObjects = 1;
    toc;
    % loop through all the clusters found
    for clustCnt = 1:1:numClusters
        tic
        % loop through all the labels and store points with the same label
        % in object1
        cnt2 = 1;
        for cnt = 1:1:size(labels)
            if labels(cnt) == clustCnt
                object1(cnt2,:) = locations(cnt,:);
                cnt2 = cnt2 + 1;
            end
        end
        % copy the cluster of points for objects big enough
        % into allObjects and reset object1
        % Also count how many objects were found using validObjects
        if cnt2 > 200
            % store how many points are in the cluster
            % objectSizes(validObjects) = cnt2;
            % find the z coordinate for the top most face
            topFace = min(object1(:,3));
            % set points that correspond to the side of the objects to 0
            object1(object1(:,3) > (topFace + 0.005),:) = 0;
            % store clusters
            % allObjects(:,:,validObjects) = object1;
            % remove NaN and 0 values
            object1(isnan(object1(:,1)),:) = [];
            object1(object1(:,1) == 0,:) = [];

            % calculate centroids
            centroid(validObjects,1) = mean(object1(:,1));
            centroid(validObjects,2) = mean(object1(:,2)); 
            centroid(validObjects,3) = mean(object1(:,3)); 
            validObjects = validObjects + 1;
        end
        % reset object1
        object1 = NaN(numLabels,3);
        toc
    end
    % Correct value for validObjects
    % validObjects = validObjects - 1;
    % Remove the zero values from when the arrays were initialised.  
    % objectSizes(objectSizes == 0) = [];
    centroid(centroid(:,1) == 0,:) = [];
    %allObjects = allObjects(:,:,1:validObjects);
    
    % Show the centroids on the original point cloud
    figure();
    hold on;
    pcshow(locations,[0 0 1]);
    title('Point Cloud');
    pcshow(centroid,[1 0 0]);
end

function [bboxes,scores,labels] = yolov3Detect(net, XTest, networkOutputs, anchors, anchorBoxMask, confidenceThreshold, overlapThreshold, classes)
% The yolov3Detect function detects the bounding boxes, scores, and labels in an image.

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

function YPredCell = yolov3Predict(net,XTrain,networkOutputs,anchorBoxMask)
% Predict the output of network and extract the confidence, x, y,
% width, height, and class.
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

function tiledAnchors = generateTiledAnchors(YPredCell,anchorBoxes,anchorBoxMask)
% Generate tiled anchor offset.
tiledAnchors = cell(size(YPredCell));
for i=1:size(YPredCell,1)
    anchors = anchorBoxes(anchorBoxMask{i}, :);
    [h,w,~,n] = size(YPredCell{i,1});
    [tiledAnchors{i,2}, tiledAnchors{i,1}] = ndgrid(0:h-1,0:w-1,1:size(anchors,1),1:n);
    [~,~,tiledAnchors{i,3}] = ndgrid(0:h-1,0:w-1,anchors(:,2),1:n);
    [~,~,tiledAnchors{i,4}] = ndgrid(0:h-1,0:w-1,anchors(:,1),1:n);
end
end

function tiledAnchors = applyAnchorBoxOffsets(tiledAnchors,YPredCell,inputImageSize)
% Convert grid cell coordinates to box coordinates.
for i=1:size(YPredCell,1)
    [h,w,~,~] = size(YPredCell{i,1});  
    tiledAnchors{i,1} = (tiledAnchors{i,1}+YPredCell{i,1})./w;
    tiledAnchors{i,2} = (tiledAnchors{i,2}+YPredCell{i,2})./h;
    tiledAnchors{i,3} = (tiledAnchors{i,3}.*YPredCell{i,3})./inputImageSize(2);
    tiledAnchors{i,4} = (tiledAnchors{i,4}.*YPredCell{i,4})./inputImageSize(1);
end
end

% function data = matRead(filename)
%     inp = load(filename);
%     f = fields(inp);
%     data = inp.(f{3});
% end

% function data = preprocessData(data, targetSize)
% % Resize the images and scale the pixels to between 0 and 1. Also scale the
% % corresponding bounding boxes.
% 
% for ii = 1:size(data,1)
%     I = data{ii,1};
%     imgSize = size(I);
%     
%     % Convert an input image with single channel to 3 channels.
%     if numel(imgSize) == 1 
%         I = repmat(I,1,1,3);
%     end
%     bboxes = data{ii,2};
%     I = im2single(imresize(I,targetSize(1:2)));
%     scale = targetSize(1:2)./imgSize(1:2);
%     bboxes = bboxresize(bboxes,scale);
%     data(ii,1:2) = {I, bboxes};
% end
% end









