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

Edit History:
28/07/2020 created file

%}
% Choose if image is obtained from a folder or ROS topic
rosConnection = true;
ipaddress = '192.168.56.101';
myFolder = '.\RGBD_Data';
loadImages = 'Multiple Objects';

% Deep learning neural network variables
executionEnvironment = "auto";
Detect_Variables = load('YOLOv3_Detect_variables.mat');
imgSize = [227 227];

% Colour options: 'all_c', 'red', 'green', 'blue'.
desiredColour = 'all_c';
% Shape options: 'all_s', 'Cube', 'Cylinder', 'Rectangle'.
desireShapes = 'all_s';

%% Obtain an image to classify
disp('Obtain images for classification');
if rosConnection == 0
    disp(['Obtain image from ',myFolder]);
    imdatastore = imageDatastore(fullfile(myFolder,... 
        loadImages),...
        'LabelSource', 'foldernames', 'FileExtensions', '.mat','ReadFcn',@matRead);
    img = readimage(imdatastore,1);
else
    % Obtain Image from ROS topic
    robotType = 'Gazebo';
    rosshutdown;
    rosinit(ipaddress);
    disp("Getting new image..");
    tic
    imsub = rossubscriber('/camera/color/image_raw');
    pause(1);
    pcsub = rossubscriber('/camera/depth/points');
    pause(1);
    img = readImage(imsub.LatestMessage);
    % plot the depth data with rgb
    depthxyz = readXYZ(pcsub.LatestMessage);
    depthrgb = readRGB(pcsub.LatestMessage);
    toc
%     % Obtain desired shapes and colours
%     chatSub = rossubscriber('/chatter');
%     chat = receive(chatSub);
%     message = chat.LatestMessage;
end

%% Object classification
disp('Classify objects in image');

% Get the image and resize to input into network.
I = imresize(img,imgSize);
I = im2single(I);
% Convert to dlarray.
XTest = dlarray(I,'SSCB');

% If GPU is available, then convert data to gpuArray.
if (executionEnvironment == "auto" && canUseGPU) || executionEnvironment == "gpu"
    XTest = gpuArray(XTest);
end

net = Detect_Variables.net;
networkOutputs = Detect_Variables.networkOutputs;
anchorBoxes = Detect_Variables.anchorBoxes;
anchorBoxMasks = Detect_Variables.anchorBoxMasks;
confidenceThreshold = Detect_Variables.confidenceThreshold;
overlapThreshold = Detect_Variables.overlapThreshold;
classNames = Detect_Variables.classNames;

[bboxes, scores, labels] = yolov3Detect(net, XTest, networkOutputs, ...
    anchorBoxes, anchorBoxMasks, confidenceThreshold, overlapThreshold, classNames);

% Display the detections on image.
if ~isempty(scores)
    I = insertObjectAnnotation(I, 'rectangle', bboxes, labels);
end
figure
imshow(I)





%% Functions
function data = matRead(filename)
    inp = load(filename);
    f = fields(inp);
    data = inp.(f{3});
end

function data = preprocessData(data, targetSize)
% Resize the images and scale the pixels to between 0 and 1. Also scale the
% corresponding bounding boxes.

for ii = 1:size(data,1)
    I = data{ii,1};
    imgSize = size(I);
    
    % Convert an input image with single channel to 3 channels.
    if numel(imgSize) == 1 
        I = repmat(I,1,1,3);
    end
    bboxes = data{ii,2};
    I = im2single(imresize(I,targetSize(1:2)));
    scale = targetSize(1:2)./imgSize(1:2);
    bboxes = bboxresize(bboxes,scale);
    data(ii,1:2) = {I, bboxes};
end
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











