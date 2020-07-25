% YOLO v3 Deep Learning
% Author: Lucas Way z5164204
% First made (DD/MM/YYY): 13/07/2020
%{
Taking a .mat file created using ROS_Connect.m, the objects in the image
will be classified using a trained neural network using YOLOv3 method.

Reference:
Object Detection Using YOLO v3 Deep Learning
https://au.mathworks.com/help/vision/examples/object-detection-using-yolo-v3-deep-learning.html

Instructions:
    Set names for folders with data for variable 'classes'

Edit History:
13/07/2020 created file
14/07/2020 added code from Deep_Learning.m. Added creation of array of
    bounding boxes. Still a work in progress. Need to change to use images
    on black table to help with binarising images.
16/07/2020 added new images, Red Cube (new), red cylinder, blue cube, blue cylinder
20/07/2020 create bounding box data store, problem with inputing
    correct type.
21/07/2020 fixed bounding box issue (rounded decimal places). Successfully
    augmented data. Preprocess training data. Defined YOLO v3 network.
    Specified training options.
22/07/2020 Included upsampleLayer.m and generateTargets.m file from
    Matlab\examples\deeplearning_shared\main folder. Error with
    read(preprocessedTrainingData);
25/07/2020 Editted to use .png images in folders, not the .mat files. Also
    recopied data augmentation and transformation functions and 
    training network just worked.
    Evaluation seems to create a result table that is full of empty arrays
    ie []. still need to fix

%}
close all;

%% Load and explore image data
disp('Loading Image Data');
% get path name
myFolder = 'c:\Users\User\Documents\UNSW\MTRN4230\Git Repo\4230Project\RGBD_Data';
% collect all file paths for .mat data sets
classes = {'PNG Red Cube','PNG Red Cylinder','PNG Blue Cube', ...
    'PNG Blue Cylinder','PNG Green Cube','PNG Green Cylinder'};
% imdatastore = imageDatastore(fullfile(myFolder,... 
%     classes), ...
%     'LabelSource', 'foldernames', 'FileExtensions', '.mat','ReadFcn',@matRead); 
imdatastore = imageDatastore(fullfile(myFolder,... 
    classes), ...
    'LabelSource', 'foldernames', 'FileExtensions', '.png'); 
% boxlabelstore = boxLabelDatastore(
classNum = size(imdatastore.Folders(:,1),1);
% load a single .mat file
MatData = imread(imdatastore.Files{1});
% count the number of different labels there are
labelCount = countEachLabel(imdatastore);
% find the dimensions of the image
imagedimension = size(MatData);

% Find folder with fewest objects, count how many images are in the folder
labelNums = table2array(labelCount(:,2));
% Define how many images will be used for training
numTrainingFiles = round(min(labelNums) * 0.6);
% separate image sets for training and validation
[imdsTrain,imdsValidation] = splitEachLabel(imdatastore,numTrainingFiles,'randomize');

% create cell array of boundary box coordinates for training data
trainBoundingBox = cell(size(imdsTrain.Files,1),1);
for cnt = 1:1:size(imdsTrain.Files,1)
    % Load an image from the training set and convert to binary image
    MatData = imread(imdsTrain.Files{cnt});
    grey = rgb2gray(MatData);
    bw = imbinarize(grey);
    % Keep the largest object
    bw = bwareafilt(bw,1);
    % imshow(bw);
    % Calculate values for bounding box
    boundingBox = regionprops(bw,'BoundingBox');
    % Store values in an cell
    trainBoundingBox(cnt,:) = {[round(boundingBox.BoundingBox(1)), ...
        round(boundingBox.BoundingBox(2)),boundingBox.BoundingBox(3),boundingBox.BoundingBox(4)]};%struct2cell(boundingBox);
end
% Convert cell array to a single table column
trainingData = table(trainBoundingBox);
% Create data store of bounding boxes
bldsTrain = boxLabelDatastore(trainingData(:, 1:end));

% create cell array of boundary box coordinates for valdiation data
validateBoundingBox = cell(size(imdsValidation.Files,1),1);
for cnt = 1:1:size(imdsValidation.Files,1)
    % Load an image from the validation set and convert to binary image
    MatData = imread(imdsValidation.Files{cnt});
    grey = rgb2gray(MatData);
    bw = imbinarize(grey);
    % Keep the largest object
    bw = bwareafilt(bw,1);
    % imshow(bw);
    % Calculate values for bounding box
    boundingBox = regionprops(bw,'BoundingBox');
    % Store values in an cell and round decimal values to an integar
    validateBoundingBox(cnt,:) = {[round(boundingBox.BoundingBox(1)), ...
        round(boundingBox.BoundingBox(2)),boundingBox.BoundingBox(3),boundingBox.BoundingBox(4)]};%struct2cell(boundingBox);
end
% Convert cell array to a single table column
validationData = table(validateBoundingBox);
% Create data store of bounding boxes
bldsValidate = boxLabelDatastore(validationData(:, 1:end));

% Set batch size values
miniBatchSize = 8;
imdsTrain.ReadSize = miniBatchSize;
bldsTrain.ReadSize = miniBatchSize;
% Combine data stores
trainingData = combine(imdsTrain, bldsTrain);
testData = combine(imdsValidation, bldsValidate);


%% Example to show data type used by the network
% data = load('vehicleDatasetGroundTruth.mat');
% vehicleDataset = data.vehicleDataset;

%% Data Augmentation
disp('Augmented Data');
% Randomly transform training images to create multiple images using a single image and
% label pair. Increased variety in training data increases the network
% accuracy.

% transform data with functions given by tutorial and save in datastore
augmentedTrainingData = transform(trainingData,@augmentData);
% save each augmented image to a cell, draw a rectangle to represent the
% bounding box and show
augmentedData = cell(4,1);
for k = 1:4
    data = read(augmentedTrainingData);
    augmentedData{k} = insertShape(data{1,1},'Rectangle',data{1,2});
    reset(augmentedTrainingData);
end
figure
montage(augmentedData,'BorderSize',10)

%% Preprocess Training Data
disp('Preprocess Data');
% specify network input size. Input size is similar to training image size.
% Increases consistancy of images used.
networkInputSize = [227 227 3];
% preprocess augmented data to prepare for training
preprocessedTrainingData = transform(augmentedTrainingData, @(data)preprocessData(data, networkInputSize));
%preprocessedTrainingData = trainingData;
% show one image with bounding box as an example

data = read(preprocessedTrainingData);
I = data{1,1};
bbox = data{1,2};
annotatedImage = insertShape(I,'Rectangle',bbox);
annotatedImage = imresize(annotatedImage,2);
figure
imshow(annotatedImage)

%% Define YOLO v3 Network
disp('Define YOLO v3 Network');
% preprocess training data to get more consistent image size
%trainingDataForEstimation = transform(trainingData,@(data)preprocessData(data,networkInputSize));

% Specify number of anchors.
numAnchors = 6;
% Estimate anchor boxes and mean IoU
% Anchor boxes are useful for deep learning object detectors and impact
% efficienty and accuracy.
% Mean IoU is the average Intersection-over-Union distance metric
[anchorBoxes, meanIoU] = estimateAnchorBoxes(preprocessedTrainingData, numAnchors);

% Select anchor boxes to use in detection heads using anchorBoxMasks
% Sort by size, using the 3 largest anchor boxes for first detection head and
% the 3 smallest boxes for second detection head
area = anchorBoxes(:, 1).*anchorBoxes(:, 2);
[~, idx] = sort(area, 'descend');
anchorBoxes = anchorBoxes(idx, :);
anchorBoxMasks = {[1,2,3]
    [4,5,6]
    };

% Load squeezenet network pretrained on Imagenet data set.
baseNetwork = squeezenet;
% remove layers after feature extraction layer 'fire9-concat' since those
% layers are specific to classification tasks and do not help with object
% detection.
lgraph = squeezenetFeatureExtractor(baseNetwork, networkInputSize);

% specify class names, number of object classes to detect, number of
% prediction elements per anchor box
% add detection heads to feature extraction network

classNames = classes;
numClasses = size(classNames,2);
numPredictorsPerAnchor = 5 + numClasses;
lgraph = addFirstDetectionHead(lgraph, anchorBoxMasks{1}, numPredictorsPerAnchor);
lgraph = addSecondDetectionHead(lgraph, anchorBoxMasks{2}, numPredictorsPerAnchor);

% Connect detection heads. Detection heads comprise the output layer of the
%   network
% Connect first detection head to feature extraction layer
% Second detection head connects to output of first detection head
% Upsampled features in second detection head merges with features from
%   'fire5-concat' layer for more meaningful semantic info in second
%   detection head
lgraph = connectLayers(lgraph, 'fire9-concat', 'conv1Detection1');
lgraph = connectLayers(lgraph,'relu1Detection1','upsample1Detection2');
lgraph = connectLayers(lgraph,'fire5-concat','depthConcat1Detection2/in2');

% specify name of detection heads. Useful when extracting output features
networkOutputs = ["conv2Detection1"
    "conv2Detection2"
    ];
% %% Define network architecture
% layers = [
%     imageInputLayer(imagedimension)
%     
%     % set filter size, number of filters, add padding to input feature map,
%     % make spatial output size equal the input size.
%     convolution2dLayer(3,8,'Padding','same')
%     % normalise activations and gradients propagating through network for
%     % easier optimisation
%     batchNormalizationLayer
%     % nonlinear activation using rectified linear unit (ReLU)
%     reluLayer
%     
%     % down-sample to reduce spatial size of feature map and remove
%     % redundant spatial info. Specify max value of rectangular regions of
%     % inputs, set step size using stride for when training function scans 
%     %along input.
%     maxPooling2dLayer(2,'Stride',2)
%     
%     convolution2dLayer(3,16,'Padding','same')
%     batchNormalizationLayer
%     reluLayer
%     
%     maxPooling2dLayer(2,'Stride',2)
%     
%     convolution2dLayer(3,32,'Padding','same')
%     batchNormalizationLayer
%     reluLayer
%     
%     % one final fully connected layer. Neurons connect to all neurons in
%     % preceding layer. combine previous features to classify image.
%     % OutputSize param is number of classes in target data.
%     fullyConnectedLayer(classes)
%     % normalise output of fully connected layer. Output of positive numbers
%     % that sum to one, used for classification probabilities by
%     % classification layer
%     softmaxLayer
%     % assign input to one of the mutually exclusive classes and compute
%     % loss.
%     classificationLayer];

%% Specify training options
disp('Specify training options');
% use stochastic gradient descent with momentum (SGDM). epoch is a full
% training cycle in entire training data. Monitor network accuracy by
% specifying validation data and frequency. Shuffle data every epoch. Train
% network on training data and calculate accuracy on validation data at
% regular intervals during training. 
% GPU is used if Parallel Computing Toolbox is available and CUDA enabled 
% GPU with compute capability >=3.0
% Show training progress plot and turn off command window output
% options = trainingOptions('sgdm', ...
%     'InitialLearnRate',0.01, ...
%     'MaxEpochs',4, ...
%     'Shuffle','every-epoch', ...
%     'ValidationData',imdsValidation, ...
%     'ValidationFrequency',30, ...
%     'Verbose',false, ...
%     'ExecutionEnvironment','auto', ...
%     'Plots','training-progress');

numIterations = 2000;
learningRate = 0.001;
% warmup period = number of iterations to increase learning rate
%   exponentially based on formula: 
%   learningRate * (iteration/warmupPeriod)
warmupPeriod = 1000;
l2Regularization = 0.0005;
penaltyThreshold = 0.5;
% Used by SGDM to store velocity of gradients
velocity = [];

%% Train network
disp('Train network');
% Train network with architecture defines by 'layers', training data and
% training options. 
executionEnvironment = "auto";
net = dlnetwork(lgraph);

% Create subplots for the learning rate and mini-batch loss.
fig = figure;
[lossPlotter, learningRatePlotter] = configureTrainingProgressPlotter(fig);
% Custom training loop.
for iteration = 1:numIterations
    % Reset datastore.
    if ~hasdata(preprocessedTrainingData)
        reset(preprocessedTrainingData);
    end
    % Read batch of data and create batch of images and
    % ground truths.
    data = read(preprocessedTrainingData);
    [XTrain,YTrain] = createBatchData(data, classNames);

    % Convert mini-batch of data to dlarray.
    XTrain = dlarray(single(XTrain),'SSCB');

    % If training on a GPU, then convert data to gpuArray.
    if (executionEnvironment == "auto" && canUseGPU) || executionEnvironment == "gpu"
        XTrain = gpuArray(XTrain);
    end

    % Evaluate the model gradients and loss using dlfeval and the
    % modelGradients function.
    [gradients,loss,state] = dlfeval(@modelGradients, net, XTrain, YTrain, anchorBoxes, anchorBoxMasks, penaltyThreshold, networkOutputs);

    % Apply L2 regularization.
    gradients = dlupdate(@(g,w) g + l2Regularization*w, gradients, net.Learnables);

    % Determine the current learning rate value.
    currentLR = piecewiseLearningRateWithWarmup(iteration, learningRate, warmupPeriod, numIterations);

    % Update the network learnable parameters using the SGDM optimizer.
    [net, velocity] = sgdmupdate(net, gradients, velocity, currentLR);

    % Update the state parameters of dlnetwork.
    net.State = state;

    % Update training plot with new points.
    addpoints(lossPlotter, iteration, double(gather(extractdata(loss))));
    addpoints(learningRatePlotter, iteration, currentLR);
    drawnow  
end
% net = trainNetwork(imdsTrain,layers,options);

%% Evaluate Model
disp('Evaluate Model');
confidenceThreshold = 0.5;
overlapThreshold = 0.5;

% Create the test datastore.
preprocessedTestData = transform(testData,@(data)preprocessData(data,networkInputSize));

% Create a table to hold the bounding boxes, scores, and labels returned by
% the detector. 
numImages = size(validationData,1);
results = table('Size',[numImages 3],...
    'VariableTypes',{'cell','cell','cell'},...
    'VariableNames',{'Boxes','Scores','Labels'});

% Run detector on each image in the test set and collect results.
for i = 1:numImages
%     if ~hasdata(preprocessedTestData)
%         reset(preprocessedTestData);
%     end
    % Read the datastore and get the image.
    data = read(preprocessedTestData);
    I = data{1};
    
    % Convert to dlarray. If GPU is available, then convert data to gpuArray.
    XTest = dlarray(I,'SSCB');
    if (executionEnvironment == "auto" && canUseGPU) || executionEnvironment == "gpu"
        XTest = gpuArray(XTest);
    end
    
    % Run the detector.
    [bboxes, scores, labels] = yolov3Detect(net, XTest, networkOutputs, anchorBoxes, anchorBoxMasks, confidenceThreshold, overlapThreshold, classNames);
    
    % Collect the results.
    results.Boxes{i} = bboxes;
    results.Scores{i} = scores;
    results.Labels{i} = labels;
end

% Evaluate the object detector using Average Precision metric.
[ap, recall, precision] = evaluateDetectionPrecision(results, preprocessedTestData);

% Plot precision-recall curve.
figure
plot(recall, precision)
xlabel('Recall')
ylabel('Precision')
grid on
title(sprintf('Average Precision = %.2f', ap))
%% Predict labels of new data and calculate classification accuracy
% 
% YPred = classify(net,imdsValidation);
% YValidation = imdsValidation.Labels;
% 
% accuracy = sum(YPred == YValidation)/numel(YValidation);

%% Classify Image

% % get path name
% multiFolder = 'c:\Users\User\Documents\UNSW\MTRN4230\Git Repo\4230Project\RGBD_Data';
% % collect all file paths for .mat data sets
% multidatastore = imageDatastore(fullfile(multiFolder,... 
%     {'Cubes and Cylinders'} ...
%     ), 'LabelSource', 'foldernames', 'FileExtensions', '.mat','ReadFcn',@matRead); 
% 
% % load a single .mat file
% MatData = load(multidatastore.Files{1});
% % show an image for testing purposes
% imshow(MatData.image);
% 
% YPred1 = classify(net,multidatastore);

%% Functions

% function to read images from .mat files
% function data = matRead(filename)
% inp = load(filename);
% f = fields(inp);
% data = inp.(f{3});
% end

function [gradients, totalLoss, state] = modelGradients(net, XTrain, YTrain, anchors, mask, penaltyThreshold, networkOutputs)
inputImageSize = size(XTrain,1:2);

% Extract the predictions from the network.
[YPredCell, state] = yolov3Forward(net,XTrain,networkOutputs,mask);

% Gather the activations in the CPU for post processing and extract dlarray data. 
gatheredPredictions = cellfun(@ gather, YPredCell(:,1:6),'UniformOutput',false); 
gatheredPredictions = cellfun(@ extractdata, gatheredPredictions, 'UniformOutput', false);

% Convert predictions from grid cell coordinates to box coordinates.
tiledAnchors = generateTiledAnchors(gatheredPredictions(:,2:5),anchors,mask);
gatheredPredictions(:,2:5) = applyAnchorBoxOffsets(tiledAnchors, gatheredPredictions(:,2:5), inputImageSize);

% Generate target for predictions from the ground truth data.
[boxTarget, objectnessTarget, classTarget, objectMaskTarget, boxErrorScale] = generateTargets(gatheredPredictions, YTrain, inputImageSize, anchors, mask, penaltyThreshold);

% Compute the loss.
boxLoss = bboxOffsetLoss(YPredCell(:,[2 3 7 8]),boxTarget,objectMaskTarget,boxErrorScale);
objLoss = objectnessLoss(YPredCell(:,1),objectnessTarget,objectMaskTarget);
clsLoss = classConfidenceLoss(YPredCell(:,6),classTarget,objectMaskTarget);
totalLoss = boxLoss + objLoss + clsLoss;

% Compute gradients of learnables with regard to loss.
gradients = dlgradient(totalLoss, net.Learnables);
end

function [YPredCell, state] = yolov3Forward(net, XTrain, networkOutputs, anchorBoxMask)
% Predict the output of network and extract the confidence score, x, y,
% width, height, and class.
YPredictions = cell(size(networkOutputs));
[YPredictions{:}, state] = forward(net, XTrain, 'Outputs', networkOutputs);
YPredCell = extractPredictions(YPredictions, anchorBoxMask);

% Append predicted width and height to the end as they are required
% for computing the loss.
YPredCell(:,7:8) = YPredCell(:,4:5);

% Apply sigmoid and exponential activation.
YPredCell(:,1:6) = applyActivations(YPredCell(:,1:6));
end

function boxLoss = bboxOffsetLoss(boxPredCell, boxDeltaTarget, boxMaskTarget, boxErrorScaleTarget)
% Mean squared error for bounding box position.
lossX = sum(cellfun(@(a,b,c,d) mse(a.*c.*d,b.*c.*d),boxPredCell(:,1),boxDeltaTarget(:,1),boxMaskTarget(:,1),boxErrorScaleTarget));
lossY = sum(cellfun(@(a,b,c,d) mse(a.*c.*d,b.*c.*d),boxPredCell(:,2),boxDeltaTarget(:,2),boxMaskTarget(:,1),boxErrorScaleTarget));
lossW = sum(cellfun(@(a,b,c,d) mse(a.*c.*d,b.*c.*d),boxPredCell(:,3),boxDeltaTarget(:,3),boxMaskTarget(:,1),boxErrorScaleTarget));
lossH = sum(cellfun(@(a,b,c,d) mse(a.*c.*d,b.*c.*d),boxPredCell(:,4),boxDeltaTarget(:,4),boxMaskTarget(:,1),boxErrorScaleTarget));
boxLoss = lossX+lossY+lossW+lossH;
end

function objLoss = objectnessLoss(objectnessPredCell, objectnessDeltaTarget, boxMaskTarget)
% Binary cross-entropy loss for objectness score.
objLoss = sum(cellfun(@(a,b,c) crossentropy(a.*c,b.*c,'TargetCategories','independent'),objectnessPredCell,objectnessDeltaTarget,boxMaskTarget(:,2)));
end

function clsLoss = classConfidenceLoss(classPredCell, classTarget, boxMaskTarget)
% Binary cross-entropy loss for class confidence score.
clsLoss = sum(cellfun(@(a,b,c) crossentropy(a.*c,b.*c,'TargetCategories','independent'),classPredCell,classTarget,boxMaskTarget(:,3)));
end

function data = augmentData(A)
% Apply random horizontal flipping, and random X/Y scaling. Boxes that get
% scaled outside the bounds are clipped if the overlap is above 0.25. Also,
% jitter image color.

data = cell(size(A));
for ii = 1:size(A,1)
    I = A{ii,1};
    bboxes = A{ii,2};
    labels = A{ii,3};
    sz = size(I);
    if numel(sz)==3 && sz(3) == 3
        I = jitterColorHSV(I,...
            'Contrast',0.0,...
            'Hue',0.1,...
            'Saturation',0.2,...
            'Brightness',0.2);
    end
    
    % Randomly flip image.
    tform = randomAffine2d('XReflection',true,'Scale',[1 1.1]);
    rout = affineOutputView(sz,tform,'BoundsStyle','centerOutput');
    I = imwarp(I,tform,'OutputView',rout);
    
    % Apply same transform to boxes.
    [bboxes,indices] = bboxwarp(bboxes,tform,rout,'OverlapThreshold',0.25);
    labels = labels(indices);
    
    % Return original data only when all boxes are removed by warping.
    if isempty(indices)
        data = A(ii,:);
    else
        data(ii,:) = {I, bboxes, labels};
    end
end
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

function [x,y] = createBatchData(data, classNames)
% The createBatchData function creates a batch of images and ground truths
% from input data, which is a [Nx3] cell array returned by the transformed
% datastore for YOLO v3. It creates two 4-D arrays by concatenating all the
% images and ground truth boxes along the batch dimension. The function
% performs these operations on the bounding boxes before concatenating
% along the fourth dimension:
% * Convert the class names to numeric class IDs based on their position in
%   the class names.
% * Combine the ground truth boxes, class IDs and network input size into
%   single cell array.
% * Pad with zeros to make the number of ground truths consistent across
%   a mini-batch.

% Concatenate images along the batch dimension.
x = cat(4,data{:,1});

% Get class IDs from the class names.
groundTruthClasses = data(:,3);
classNames = repmat({categorical(classNames')},size(groundTruthClasses));
[~,classIndices] = cellfun(@(a,b)ismember(a,b),groundTruthClasses,classNames,'UniformOutput',false);

% Append the label indexes and training image size to scaled bounding boxes
% and create a single cell array of responses.
groundTruthBoxes = data(:,2);
combinedResponses = cellfun(@(bbox,classid)[bbox,classid],groundTruthBoxes,classIndices,'UniformOutput',false);
len = max( cellfun(@(x)size(x,1), combinedResponses ) );
paddedBBoxes = cellfun( @(v) padarray(v,[len-size(v,1),0],0,'post'), combinedResponses, 'UniformOutput',false);
y = cat(4,paddedBBoxes{:,1});
end

function lgraph = squeezenetFeatureExtractor(net, imageInputSize)
% The squeezenetFeatureExtractor function removes the layers after 'fire9-concat'
% in SqueezeNet and also removes any data normalization used by the image input layer.

% Convert to layerGraph.
lgraph = layerGraph(net);

lgraph = removeLayers(lgraph, {'drop9' 'conv10' 'relu_conv10' 'pool10' 'prob' 'ClassificationLayer_predictions'});
inputLayer = imageInputLayer(imageInputSize,'Normalization','none','Name','data');
lgraph = replaceLayer(lgraph,'data',inputLayer);
end

function lgraph = addFirstDetectionHead(lgraph,anchorBoxMasks,numPredictorsPerAnchor)
% The addFirstDetectionHead function adds the first detection head.

numAnchorsScale1 = size(anchorBoxMasks, 2);
% Compute the number of filters for last convolution layer.
numFilters = numAnchorsScale1*numPredictorsPerAnchor;
firstDetectionSubNetwork = [
    convolution2dLayer(3,256,'Padding','same','Name','conv1Detection1','WeightsInitializer','he')
    reluLayer('Name','relu1Detection1')
    convolution2dLayer(1,numFilters,'Padding','same','Name','conv2Detection1','WeightsInitializer','he')
    ];
lgraph = addLayers(lgraph,firstDetectionSubNetwork);
end

function lgraph = addSecondDetectionHead(lgraph,anchorBoxMasks,numPredictorsPerAnchor)
% The addSecondDetectionHead function adds the second detection head.

numAnchorsScale2 = size(anchorBoxMasks, 2);
% Compute the number of filters for the last convolution layer.
numFilters = numAnchorsScale2*numPredictorsPerAnchor;
secondDetectionSubNetwork = [
    upsampleLayer(2,'upsample1Detection2')
    depthConcatenationLayer(2,'Name','depthConcat1Detection2');
    convolution2dLayer(3,128,'Padding','same','Name','conv1Detection2','WeightsInitializer','he')
    reluLayer('Name','relu1Detection2')
    convolution2dLayer(1,numFilters,'Padding','same','Name','conv2Detection2','WeightsInitializer','he')
    ];
lgraph = addLayers(lgraph,secondDetectionSubNetwork);
end

function currentLR = piecewiseLearningRateWithWarmup(iteration, learningRate, warmupPeriod, numIterations)
% The piecewiseLearningRateWithWarmup function computes the current
% learning rate based on the iteration number.

if iteration <= warmupPeriod
    % Increase the learning rate for number of iterations in warmup period.
    currentLR = learningRate * ((iteration/warmupPeriod)^4);
    
elseif iteration >= warmupPeriod && iteration < warmupPeriod+floor(0.6*(numIterations-warmupPeriod))
    % After warm up period, keep the learning rate constant if the remaining number of iteration is less than 60 percent. 
    currentLR = learningRate;
    
elseif iteration >= warmupPeriod+floor(0.6*(numIterations-warmupPeriod)) && iteration < warmupPeriod+floor(0.9*(numIterations-warmupPeriod))
    % If the remaining number of iteration is more than 60 percent but less
    % than 90 percent multiply the learning rate by 0.1.
    currentLR = learningRate*0.1;
    
else
    % If remaining iteration is more than 90 percent multiply the learning
    % rate by 0.01.
    currentLR = learningRate*0.01;
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

function [lossPlotter, learningRatePlotter] = configureTrainingProgressPlotter(f)
% Create the subplots to display the loss and learning rate.
figure(f);
clf
subplot(2,1,1);
ylabel('Learning Rate');
xlabel('Iteration');
learningRatePlotter = animatedline;
subplot(2,1,2);
ylabel('Total Loss');
xlabel('Iteration');
lossPlotter = animatedline;
end