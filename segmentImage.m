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
data = read(imdatastore);
img = data;

img_grey = rgb2gray(img);
img_bw = imbinarize(img_grey);
figure(1);
imshow(img_bw);

stats = regionprops(img_bw,'centroid');
figure(2);
centroids = cat(1,stats.Centroid);
imshow(img)
hold on
plot(centroids(:,1),centroids(:,2),'b*')
hold off
numObjects = size(centroids, 1);
centroids = round(centroids);
for cnt = 1:numObjects
    topLeft = centroids(cnt,2) - 80; %round((centroids(2,1)/900) * 640)- 100;
    if (topLeft <= 0)
        topLeft = 1; end
    topRight = centroids(cnt,2) + 80; %round((centroids(2,1)/900) * 640)+ 100;
    if (topRight >= 480)
        topRight = 480; end    
    bottomLeft = centroids(cnt,1) - 80; %round((centroids(2,2)/400) * 480) - 100;
    if (bottomLeft <= 0) 
        bottomLeft = 1; end    
    bottomRight = centroids(cnt,1) + 80; %round((centroids(2,2)/400) * 480)+ 100;
    if (bottomRight >= 640)
        bottomRight = 640; end

    section = img(topLeft:topRight,bottomLeft:bottomRight,:);
    figure(cnt + 3);
    imshow(section);
end

