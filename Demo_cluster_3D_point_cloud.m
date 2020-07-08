% Demo_cluster_3D_point_cloud
% Author: Lucas Way z5164204
% First made (DD/MM/YYY): 07/07/2020
% Last Edited: 08/07/2020
% Taking a file created using ROS_Connect.m, the XYZ coordinates 
% of the centroid of the objects is returned.

clc;
clear all;
close all;

% Obtain file with coordinates
tic;
filename = 'LODESOFSTUFF.mat';
cube = load(filename);
% Remove the floor. Assume the point furthest away in the z axis 
% is the floor
floor = max(cube.xyz(:,3)) - 0.05;
cube.xyz(cube.xyz(:,3) >= floor,:) = [];

% Convert to type pointcloud
ptCloudHiRes = pointCloud(cube.xyz);
% Reduce the number of samples to aid computation time
ptCloud = pcdownsample(ptCloudHiRes, 'gridAverage', 0.005);

%show the obtained point cloud
%pcshow(ptCloud)
%title('Point Cloud')

% label the groups of points that form a cluster
minDistance = 0.01;
[labels,numClusters] = pcsegdist(ptCloud,minDistance);

%copy the x,y,z coordinates of the points
locations = ptCloud.Location;

% find out how many points were labelled and create an array
% where each 'layer' in allObjects is the coordinates for a 
% cluster of points. Each cluster is represented as object1.
size1 = size(labels(:,1));
size1 = size1(1);
object1 = zeros(size1,3);
allObjects = zeros(size1,3,numClusters);
objectSizes = zeros(numClusters,1);
centroid = zeros(size1,3);
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
        objectSizes(validObjects) = cnt2;
        allObjects(:,:,validObjects) = object1;
        object1(object1(:,1) == 0,:) = [];
        centroid(validObjects,1) = mean(object1(:,1));
        centroid(validObjects,2) = mean(object1(:,2)); 
        centroid(validObjects,3) = mean(object1(:,3)); 
        validObjects = validObjects + 1;
    end
    object1 = zeros(size1,3);
    toc
end
% Correct value for validObjects
validObjects = validObjects - 1;
% Remove the zero values from when the arrays were initialised.  
objectSizes(objectSizes == 0) = [];
centroid(centroid(:,1) == 0,:) = [];
allObjects = allObjects(:,:,1:validObjects);

% loop through and show each cluster
figureNum = 2;
for counter = 1:validObjects
    figure(figureNum);
    figureNum = figureNum + 1;
    pcshow(allObjects(:,:,counter));
    
end

% Show the centroids on the original point cloud
figure(figureNum);
hold on;
pcshow(ptCloud.Location,[0 0 1]);
title('Point Cloud');
pcshow(centroid,[1 0 0]);



