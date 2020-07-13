% Deep Learning
% Author: Lucas Way z5164204
% First made (DD/MM/YYY): 11/07/2020
% Last Edited: 13/07/2020
% 
%{
Taking a file created using ROS_Connect.m, the images were used to train
and validate a deep learning network following instructions from 
https://au.mathworks.com/help/deeplearning/ug/create-simple-deep-learning-network-for-classification.html

Instructions:
    Running the full code creates a set of training image data sets and a
    set for validation. These images are used to train a neural network
    which is saved as the variable 'net'. 
    Using the 'classify(net,_)' function to classify a set of images. The entire
    code does not need to be re-run assuming the workspace was saved. The
    net variable can be re-used. Run the last two sections (ignoring the 
    function matRead at the very end) 

11/07/2020 created file, added images to cell array.
13/07/2020 changed to use image data store, successfully trained neural
    network.

%}

%clear all;
close all;
imshow(image);

%% Load and explore image data

% folder location
% myFolder = 'c:\Users\User\Documents\UNSW\MTRN4230\Git Repo\4230Project\RGBD_Data\Red Cube (straight)';
% % get path for any file of type .mat
% digitDatasetPath = myFolder + "\*.mat";
% 
% % Obtain a list of all the files of type .mat
% imds = dir(digitDatasetPath);
% Imdata = zeros(length(imds),1);
% images = {};
% imageLabels = {};
% 
% % copy all image sets to a cell
% for k = 1:length(imds)
%   baseFileName = imds(k).name;
%   fullFileName = fullfile(myFolder, baseFileName);
%   MatData = load(fullFileName);
%   images = cat(1,images,MatData.image);
%   imageLabels = cat(1,imageLabels, 'red cube');
% end
% ImageWithLabels = cat(2,images,imageLabels);
% test = cell2mat(images(:,1));
% imshow(test);

% get path name
myFolder = 'c:\Users\User\Documents\UNSW\MTRN4230\Git Repo\4230Project\RGBD_Data';
% collect all file paths for .mat data sets
imdatastore = imageDatastore(fullfile(myFolder,... 
    {'Red Cube (Diagonal)','Red Cube (straight)'} ...
    ), 'LabelSource', 'foldernames', 'FileExtensions', '.mat','ReadFcn',@matRead); 

% load a single .mat file
MatData = load(imdatastore.Files{1});
% show an image for testing purposes
imshow(MatData.image);

% count the number of different labels there are
labelCount = countEachLabel(imdatastore);
% find the dimensions of the image
imagedimension = size(MatData.image);

%separate image sets for training and validation
numTrainingFiles = 10;
[imdsTrain,imdsValidation] = splitEachLabel(imdatastore,numTrainingFiles,'randomize');

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
% Show training progress plot and turn off command window output
options = trainingOptions('sgdm', ...
    'InitialLearnRate',0.01, ...
    'MaxEpochs',4, ...
    'Shuffle','every-epoch', ...
    'ValidationData',imdsValidation, ...
    'ValidationFrequency',30, ...
    'Verbose',false, ...
    'Plots','training-progress');

%% Train network

% Train network with architecture defines by 'layers', training data and
% training options. GPU is used if Parallel Computing Toolbox is available and CUDA
% enabled GPU with compute capability >=3.0

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
    {'Multiple Objects'} ...
    ), 'LabelSource', 'foldernames', 'FileExtensions', '.mat','ReadFcn',@matRead); 

YPred = classify(net,multidatastore);

%% function to read images from .mat files
function data = matRead(filename)
inp = load(filename);
f = fields(inp);
data = inp.(f{3});
end
