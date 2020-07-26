% Segment Image
% Author: Lucas Way z5164204
% First made (DD/MM/YYY): 26/07/2020
%{
Taking a .mat file created using ROS_Connect.m, the objects in the image
will be separated as individual images

Reference:

Instructions:
    change 'myFolder' to choose the folder with all the images
    change 'classes' variable to choose which folders to use
Edit History:
    26/07/2020 create file. Doesn't work with custom images
%}

close all;
clc;
dbstop if error
myFolder = '.\RGBD_Data';
% collect all file paths for .mat data sets
classes = {'PNG Multiple Objects'};
% Grab all the images in the folder and store in a single folder
imdatastore = imageDatastore(fullfile(myFolder,... 
    classes), ...
    'LabelSource', 'foldernames', 'FileExtensions', '.png'); 
% Take a single image
data = readimage(imdatastore,5);
img = data;

% Convert to binary image. Set sensitivity to detect the dark blue objects
% with the black background
img_grey = rgb2gray(img);
img_bw = imbinarize(img_grey,'adaptive','ForegroundPolarity','dark','Sensitivity',0.9);
% separate images more
se = strel('disk',8);
img_bw = imclose(img_bw,se);
% Separate images and remove some noise
se = strel('disk',6);
img_bw = imerode(img_bw,se);
% Create more solid blobs
img_bw = imfill(img_bw,'holes');

figure(1);
imshow(img_bw);

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
    % Show image
    figure(cnt + 3);
    imshow(section);
end

