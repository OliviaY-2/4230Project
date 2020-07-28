% Segment Image
% Author: Lucas Way z5164204
% First made (DD/MM/YYY): 26/07/2020
%{
Taking a .mat file created using ROS_Connect.m, the objects in the image
will be separated as individual images

Reference:
    
Instructions:
    Set 'rosConnection' to false if you want to use images from a file and
    true if you want to obtain images from the simulated camera in Gazebo
    simulation.
    Set 'imgSize' to desired image dimension when feeding into neural
    network.
    Set 'ipaddress' to appropriate address for connection to ROS topic.
    Change 'myFolder' to choose the folder with all the images.
    Change 'classes' variable to choose which folders to use.
    Change 'net' to a pretrained deep learning network
Edit History:
    26/07/2020 create file. Show each object as an individual image.
    27/07/2020 create cell array with all the individual images. Also
        resize image. Added ROS connection option. Added classification
        with pretrained network. Added functions to create colour masked
        images
%}

close all;
clc;

rosConnection = false;
imgSize = [227 227];
ipaddress = '192.168.1.118';
myFolder = '.\RGBD_Data';
classes = 'Multiple Objects';
net = load('net.mat','net');
desiredColour = 'all';
pause(1);
%% Obtain images 
% from .mat file
if rosConnection == 0
    disp(['Obtain image from ',myFolder]);
    % collect all file paths for .mat data sets
    % Grab all the images in the folder and store in a single folder
%     imdatastore = imageDatastore(fullfile(myFolder,... 
%         classes), ...
%         'LabelSource', 'foldernames', 'FileExtensions', '.png'); 
%     % Take a single image
%     img = readimage(imdatastore,2);
    
    imdatastore = imageDatastore(fullfile(myFolder,... 
        classes),...
        'LabelSource', 'foldernames', 'FileExtensions', '.mat','ReadFcn',@matRead);
    img = readimage(imdatastore,2);
else
    % Obtain Image from ROS topic
    robotType = 'Gazebo';
    rosshutdown;
    rosinit(ipaddress);
    blockposes = rossubscriber('/gazebo/link_states');
    pause(2);
    disp("Getting new image..");
    tic
    posdata = receive(blockposes, 10);
    imsub = rossubscriber('/camera/color/image_raw');
    pcsub = rossubscriber('/camera/depth/points');
    pause(1);
    img = readImage(imsub.LatestMessage);
    % plot the depth data with rgb
    depthxyz = readXYZ(pcsub.LatestMessage);
    depthrgb = readRGB(pcsub.LatestMessage);
    
    % Obtain desired shapes and colours
    chatSub = rossubscriber('/chatter');
    chat = receive(chatSub);
    message = chat.LatestMessage;
end
%% Create mask 
% Convert to binary image. Set sensitivity to detect the dark blue objects
% with the black background
switch desiredColour
    case 'red'
        [BW,img_masked] = createREDMask(img);
    case 'green'
        [BW,img_masked] = createGREENMask(img);
    case 'blue'
        [BW,img_masked] = createBLUEMask(img);
    case 'all'
        img_masked = img;
end
img_grey = rgb2gray(img_masked);
img_bw = imbinarize(img_grey,'adaptive','ForegroundPolarity','dark','Sensitivity',0.9);
img_bw = bwareaopen(img_bw, 3000);
img_bw = bwpropfilt(img_bw, 'Eccentricity', 10); 
% separate images more
se = strel('disk',10);
img_bw = imclose(img_bw,se);
% Separate images and remove some noise
se = strel('disk',6);
img_bw = imerode(img_bw,se);
% Create more solid blobs
img_bw = imfill(img_bw,'holes');

figure(1);
imshow(img_bw);
%% locate individual objects in image
% Find centroids
stats = regionprops(img_bw,'centroid');
% Draw centroids on image
figure(2);
centroids = cat(1,stats.Centroid);
imshow(img)
hold on
plot(centroids(:,1),centroids(:,2),'b*')
hold off
% Create separate image for each object
numObjects = size(centroids, 1);
centroids = round(centroids);

%% Create each image
if centroids ~= 0
    miniImages = [];
    cellArrayOfImages = cell(1,numObjects);
    YPred = cell(1,numObjects);
    for cnt = 1:numObjects
        % Set values to obtain pixels surrounding centroid position
        % Set x values
        xLeft = centroids(cnt,2) - 80; 
        % additional condition for the edge of the image
        if (xLeft <= 0)
            xLeft = 1; end
        xRight = centroids(cnt,2) + 80; 
        if (xRight >= 480)
            xRight = 480; end
        % Set y values
        yLeft = centroids(cnt,1) - 80; 
        if (yLeft <= 0) 
            yLeft = 1; end    
        yRight = centroids(cnt,1) + 80; 
        if (yRight >= 640)
            yRight = 640; end
        % Obtain pixels in defines area
        section = img(xLeft:xRight,yLeft:yRight,:);
        % Store in cell array
        cellArrayOfImages{cnt} = section;
        figure();
        imshow(section);
        resizeImage = imresize(section,imgSize);
        YPred{cnt} = classify(net.net,resizeImage);
        title(YPred{cnt});
    end
end
%% Deep Learning Network
% for cnt = 1:numObjects
%     image = cell2mat(cellArrayOfImages(1));
%     % Resize image
%     resizeImage = imresize(image,imgSize);
%     YPred = classify(net.net,resizeImage);
% end
%% Functions
% function to read images from .mat files
function data = matRead(filename)
inp = load(filename);
f = fields(inp);
data = inp.(f{3});
end

function [BW,maskedRGBImage] = createREDMask(RGB)

% Convert RGB image to chosen color space
I = rgb2hsv(RGB);

% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.770;
channel1Max = 0.226;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.210;
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

function [BW,maskedRGBImage] = createGREENMask(RGB)

% Convert RGB image to chosen color space
I = rgb2hsv(RGB);

% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.073;
channel1Max = 0.628;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.000;
channel2Max = 1.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.210;
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

function [BW,maskedRGBImage] = createBLUEMask(RGB)

% Convert RGB image to chosen color space
I = rgb2hsv(RGB);

% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.448;
channel1Max = 0.929;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.000;
channel2Max = 1.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.210;
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

