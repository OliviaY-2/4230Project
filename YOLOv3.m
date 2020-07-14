% YOLO v3 Deep Learning
% Author: Lucas Way z5164204
% First made (DD/MM/YYY): 13/07/2020
% Last Edited: 13/07/2020
% 
%{
Taking a .mat file created using ROS_Connect.m, the objects in the image
will be classified using a trained neural network
https://au.mathworks.com/help/vision/examples/object-detection-using-yolo-v3-deep-learning.html

Instructions:
    ---

13/07/2020 created file
14/07/2020 added code from Deep_Learning.m. Added creation of array of
    bounding boxes. Still a work in progress. Need to change to use images
    on black table to help with binarising images.

%}
%clear all;
close all;

%% Load and explore image data
% get path name
myFolder = 'c:\Users\User\Documents\UNSW\MTRN4230\Git Repo\4230Project\RGBD_Data';
% collect all file paths for .mat data sets
imdatastore = imageDatastore(fullfile(myFolder,... 
    {'Red Cube','Blue Cylinder'} ...
    ), 'LabelSource', 'foldernames', 'FileExtensions', '.mat','ReadFcn',@matRead); 
%boxlabelstore = boxLabelDatastore(

% load a single .mat file
MatData = load(imdatastore.Files{1});
% count the number of different labels there are
labelCount = countEachLabel(imdatastore);
% find the dimensions of the image
imagedimension = size(MatData.image);

% show bounding box for one image
% grey = rgb2gray(MatData.image);
% bw = imbinarize(grey,'adaptive','ForegroundPolarity','bright');
% imshow(MatData.image);
% 
% testBoundingBox = regionprops(bw,'BoundingBox');
% hold on
% for k = 1 : length(testBoundingBox)
%      BB = testBoundingBox(k).BoundingBox;
%      rectangle('Position', [BB(1),BB(2),BB(3),BB(4)],'EdgeColor','r','LineWidth',2) ;
% end

%separate image sets for training and validation
labelNums = table2array(labelCount(:,2));
numTrainingFiles = round(min(labelNums) * 0.6);
[imdsTrain,imdsValidation] = splitEachLabel(imdatastore,numTrainingFiles,'randomize');

trainBoundingBox = zeros(size(imdsTrain.Files,1),4);
for cnt = 1:1:size(imdsTrain.Files,1)
    MatData = load(imdsTrain.Files{cnt});
    grey = rgb2gray(MatData.image);
    bw = imbinarize(grey,'adaptive','ForegroundPolarity','bright');
    imshow(MatData.image);
    boundindBox = regionprops(bw,'BoundingBox');
    trainBoundingBox(cnt,:) = boundindBox.BoundingBox;
end

validateBoundingBox = zeros(size(imdsValidation.Files,1),4);
for cnt = 1:1:size(imdsValidation.Files,1)
    MatData = load(imdsValidation.Files{cnt});
    grey = rgb2gray(MatData.image);
    bw = imbinarize(grey);
    imshow(MatData.image);
    boundindBox = regionprops(bw,'BoundingBox');
    validateBoundingBox(cnt,:) = boundindBox.BoundingBox;
end
%%
hold on;
imshow(bw);
    for k = 1 : length(boundindBox)
     BB = boundindBox(k).BoundingBox;
     rectangle('Position', [BB(1),BB(2),BB(3),BB(4)],'EdgeColor','r','LineWidth',2) ;
    end
    hold off;

%% Define network architecture
layers = [
    imageInputLayer(imagedimension)
    
    % set filter size, number of filters, add padding to input feature map,
    % make spatial output size equal the input size.
    convolution2dLayer(3,8,'Padding','same')
    % normalise activations and gradients propagating through network for
    % easier optimisation
    batchNormalizationLayer
    % nonlinear activation using rectified linear unit (ReLU)
    reluLayer
    
    % down-sample to reduce spatial size of feature map and remove
    % redundant spatial info. Specify max value of rectangular regions of
    % inputs, set step size using stride for when training function scans 
    %along input.
    maxPooling2dLayer(2,'Stride',2)
    
    convolution2dLayer(3,16,'Padding','same')
    batchNormalizationLayer
    reluLayer
    
    maxPooling2dLayer(2,'Stride',2)
    
    convolution2dLayer(3,32,'Padding','same')
    batchNormalizationLayer
    reluLayer
    
    % one final fully connected layer. Neurons connect to all neurons in
    % preceding layer. combine previous features to classify image.
    % OutputSize param is number of classes in target data.
    fullyConnectedLayer(2)
    % normalise output of fully connected layer. Output of positive numbers
    % that sum to one, used for classification probabilities by
    % classification layer
    softmaxLayer
    % assign input to one of the mutually exclusive classes and compute
    % loss.
    classificationLayer];

%% Specify training options

% use stochastic gradient descent with momentum (SGDM). epoch is a full
% training cycle in entire training data. Monitor network accuracy by
% specifying validation data and frequency. Shuffle data every epoch. Train
% network on training data and calculate accuracy on validation data at
% regular intervals during training. 
% GPU is used if Parallel Computing Toolbox is available and CUDA enabled 
% GPU with compute capability >=3.0
% Show training progress plot and turn off command window output
options = trainingOptions('sgdm', ...
    'InitialLearnRate',0.01, ...
    'MaxEpochs',4, ...
    'Shuffle','every-epoch', ...
    'ValidationData',imdsValidation, ...
    'ValidationFrequency',30, ...
    'Verbose',false, ...
    'ExecutionEnvironment','auto', ...
    'Plots','training-progress');

%% Train network

% Train network with architecture defines by 'layers', training data and
% training options. 

net = trainNetwork(imdsTrain,layers,options);

%% Predict labels of new data and calculate classification accuracy

YPred = classify(net,imdsValidation);
YValidation = imdsValidation.Labels;

accuracy = sum(YPred == YValidation)/numel(YValidation);

%% Classify Image

% get path name
multiFolder = 'c:\Users\User\Documents\UNSW\MTRN4230\Git Repo\4230Project\RGBD_Data';
% collect all file paths for .mat data sets
multidatastore = imageDatastore(fullfile(multiFolder,... 
    {'Cubes and Cylinders'} ...
    ), 'LabelSource', 'foldernames', 'FileExtensions', '.mat','ReadFcn',@matRead); 

% load a single .mat file
MatData = load(multidatastore.Files{1});
% show an image for testing purposes
imshow(MatData.image);

YPred1 = classify(net,multidatastore);

%% function to read images from .mat files
function data = matRead(filename)
inp = load(filename);
f = fields(inp);
data = inp.(f{3});
end
