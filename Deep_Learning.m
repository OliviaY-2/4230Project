% Deep Learning
% Author: Lucas Way z5164204
% First made (DD/MM/YYY): 11/07/2020
% Last Edited: 11/07/2020
% Taking a file created using ROS_Connect.m, the images were used to train
% and validate a deep learning network following instructions from 
% https://au.mathworks.com/help/deeplearning/ug/create-simple-deep-learning-network-for-classification.html

% Load and explore image data
myFolder = 'c:\Users\User\Documents\UNSW\MTRN4230\Git Repo\4230Project\RGBD_Data\Red Cube (straight)';
digitDatasetPath = myFolder + "\*.mat";

imds = dir(digitDatasetPath);
Imdata = zeros(length(imds),1);
for k = 1:length(imds)
  baseFileName = imds(k).name;
  fullFileName = fullfile(myFolder, baseFileName);
  MatData = load(fullFileName);
  hello = cat(1,hello,MatData.image);
end


% Define network architecture
% Specify training options
% Train network
% Predict labels of new data and calculate classification accuracy