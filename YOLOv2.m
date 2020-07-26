% YOLO v2 Deep Learning
% Author: Lucas Way z5164204
% First made (DD/MM/YYY): 26/07/2020
%{
Taking a .mat file created using ROS_Connect.m, the objects in the image
will be classified using a trained neural network using YOLOv3 method.

Reference:
Object Detection Using YOLO v2 Deep Learning
https://au.mathworks.com/help/deeplearning/ug/object-detection-using-yolo-v2.html?s_tid=blogs_rc_6

Instructions:
    Set names for folders with data for variable 'classes'
    Set 'doTraining' variable to true if you want to train a network or false
    if you want to use a pretrained one
    In load data, set value for DataTable to either use custom images or
    try out with vehicle images given by example
    Using Add-on explorer, install 'Deep Learning Toolbox Model for
    ResNet-50 network
    DO NOT BOTHER with pretrained network. It just doesn't work
Edit History:
    26/07/2020 create file. Doesn't work with custom images
%}
close all;
clc;
dbstop if error

doTraining = true;
if ~doTraining && ~exist('yolov2ResNet50VehicleExample_19b.mat','file')    
    disp('Downloading pretrained detector (98 MB)...');
    pretrainedURL = 'https://www.mathworks.com/supportfiles/vision/data/yolov2ResNet50VehicleExample_19b.mat';
    websave('yolov2ResNet50VehicleExample_19b.mat',pretrainedURL);
end


%% Example Data
% disp('Load data');
% unzip vehicleDatasetImages.zip
% data = load('vehicleDatasetGroundTruth.mat');
% vehicleDataset = data.vehicleDataset;
% vehicleDataset.imageFilename = fullfile(pwd,vehicleDataset.imageFilename);
% DataTable = vehicleDataset;
%% Custom Data
% 
myFolder = '.\RGBD_Data';
% collect all file paths for .mat data sets
classes = {'PNG Multiple Objects','PNG Red Cube','PNG Red Cylinder','PNG Blue Cube', ...
    'PNG Blue Cylinder','PNG Green Cube','PNG Green Cylinder'};
% Grab all the images in the folder and store in a single folder
imdatastore = imageDatastore(fullfile(myFolder,... 
    classes), ...
    'LabelSource', 'foldernames', 'FileExtensions', '.png'); 
% load bounding boxes obtained using image labeling app
boundingBoxes = load('MultiObjectBoundingBoxes.mat');

% Create a combined table of all the data
DataTable = [cell2table(boundingBoxes.gTruth.DataSource.Source) boundingBoxes.gTruth.LabelData];
DataTable.Properties.VariableNames{1} = 'imageFilename';

%% Split data
disp('Split Data');
rng(0);
shuffledIndices = randperm(height(DataTable));
idx = floor(0.6 * length(shuffledIndices));

% split data set for training, validation and testing
trainingIdx = 1:idx;
trainingDataTbl = DataTable(shuffledIndices(trainingIdx),:);

validationIdx = idx+1 : idx + 1 + floor(0.1 * length(shuffledIndices) );
validationDataTbl = DataTable(shuffledIndices(validationIdx),:);

testIdx = validationIdx(end)+1 : length(shuffledIndices);
testDataTbl = DataTable(shuffledIndices(testIdx),:);

% Create datastores

imdsTrain = imageDatastore(trainingDataTbl{:,'imageFilename'});
bldsTrain = boxLabelDatastore(trainingDataTbl(:,2:end));

imdsValidation = imageDatastore(validationDataTbl{:,'imageFilename'});
bldsValidation = boxLabelDatastore(validationDataTbl(:,2:end));

imdsTest = imageDatastore(testDataTbl{:,'imageFilename'});
bldsTest = boxLabelDatastore(testDataTbl(:,2:end));

% Combine image and bounding box data stores
trainingData = combine(imdsTrain,bldsTrain);
validationData = combine(imdsValidation,bldsValidation);
testData = combine(imdsTest,bldsTest);

% Display an image
data = read(trainingData);
I = data{1};
bbox = data{2};
annotatedImage = insertShape(I,'Rectangle',bbox);
annotatedImage = imresize(annotatedImage,2);
figure
imshow(annotatedImage)

%% Create YOLO v2 Object Detection Network
disp('Create Network');
% Set to be close to size of training image, larger than input size
% required for network
inputSize = [480 480 3];
% Number of object classes to detect
numClasses = width(DataTable)-1;

% Resize images
trainingDataForEstimation = transform(trainingData,@(data)preprocessData(data,inputSize));
% estimate bounding boxes
numAnchors = 7;
[anchorBoxes, meanIoU] = estimateAnchorBoxes(trainingDataForEstimation, numAnchors);

% load pretrained ResNet-50 model
featureExtractionNetwork = resnet50;

% Choose feature extraction layer
featureLayer = 'activation_40_relu';

% Create network
lgraph = yolov2Layers(inputSize,numClasses,anchorBoxes,featureExtractionNetwork,featureLayer);

%% data augmentation
disp('Augment Data');
augmentedTrainingData = transform(trainingData,@augmentData);
% visualise
augmentedData = cell(4,1);
for k = 1:4
    data = read(augmentedTrainingData);
    augmentedData{k} = insertShape(data{1},'Rectangle',data{2});
    reset(augmentedTrainingData);
end
figure
montage(augmentedData,'BorderSize',10)

%% Preprocess data
disp('Preprocess Data');
preprocessedTrainingData = transform(augmentedTrainingData,@(data)preprocessData(data,inputSize));
preprocessedValidationData = transform(validationData,@(data)preprocessData(data,inputSize));

data = read(preprocessedTrainingData);

% Display image and bounding box
I = data{1};
bbox = data{2};
annotatedImage = insertShape(I,'Rectangle',bbox);
annotatedImage = imresize(annotatedImage,2);
figure
imshow(annotatedImage)

%% Train
augmentedData = cell(25,1);
k = 1; 
while hasdata(preprocessedTrainingData)
    T = read(preprocessedTrainingData);
    I = T{1};
    bbox = T{2};
    augmentedData{k} = insertShape(I,'Rectangle',bbox);
    k = k+1; 
end
figure
montage(augmentedData,'BorderSize',2)

disp('Train network');
options = trainingOptions('sgdm', ...
        'MiniBatchSize',16, ....
        'InitialLearnRate',1e-3, ...
        'MaxEpochs',20,...
        'CheckpointPath',tempdir, ...
        'ValidationData',preprocessedValidationData);

if doTraining       
    % Train the YOLO v2 detector.
    [detector,info] = trainYOLOv2ObjectDetector(preprocessedTrainingData,lgraph,options);
else
    % Load pretrained detector for the example.
    pretrained = load('yolov2ResNet50VehicleExample_19b.mat');
    detector = pretrained.detector;
end
% Quick test
I = imread(testDataTbl.imageFilename{1});
I = imresize(I,inputSize(1:2));
imshow(I)
[bboxes,scores] = detect(detector,I);

I = insertObjectAnnotation(I,'rectangle',bboxes,scores);
figure
imshow(I)

%% Evaluate Detector
disp('Evaluate');
preprocessedTestData = transform(testData,@(data)preprocessData(data,inputSize));

detectionResults = detect(detector, preprocessedTestData);

[ap,recall,precision] = evaluateDetectionPrecision(detectionResults, preprocessedTestData);

figure
plot(recall,precision)
xlabel('Recall')
ylabel('Precision')
grid on
title(sprintf('Average Precision = %.2f',ap))

%% Supporting Functions

function B = augmentData(A)
% Apply random horizontal flipping, and random X/Y scaling. Boxes that get
% scaled outside the bounds are clipped if the overlap is above 0.25. Also,
% jitter image color.
B = cell(size(A));

I = A{1};
sz = size(I);
if numel(sz)==3 && sz(3) == 3
    I = jitterColorHSV(I,...
        'Contrast',0.2,...
        'Hue',0,...
        'Saturation',0.1,...
        'Brightness',0.2);
end

% Randomly flip and scale image.
tform = randomAffine2d('XReflection',true,'Scale',[1 1.1]);
rout = affineOutputView(sz,tform,'BoundsStyle','CenterOutput');
B{1} = imwarp(I,tform,'OutputView',rout);

% Apply same transform to boxes.
[B{2},indices] = bboxwarp(A{2},tform,rout,'OverlapThreshold',0.25);
B{3} = A{3}(indices);

% Return original data only when all boxes are removed by warping.
if isempty(indices)
    B = A;
end
end

function data = preprocessData(data,targetSize)
% Resize image and bounding boxes to the targetSize.
scale = targetSize(1:2)./size(data{1},[1 2]);
data{1} = imresize(data{1},targetSize(1:2));
boxEstimate=round(data{2});
boxEstimate(:,1)=max(boxEstimate(:,1),1);
boxEstimate(:,2)=max(boxEstimate(:,2),1);
data{2} = bboxresize(boxEstimate,scale);
end























