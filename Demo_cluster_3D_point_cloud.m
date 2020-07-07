clc;
clear all;
close all;

[X,Y,Z] = sphere(100);
loc1 = [X(:),Y(:),Z(:)];
loc2 = 2*loc1;
loc3 = 3*loc2;
ptCloud = pointCloud([loc1;loc2;loc3]);
pcshow(ptCloud)
title('Point Cloud')
minDistance = 0.5;
[labels,numClusters] = pcsegdist(ptCloud,minDistance);

labelsCopy = labels;
locations = ptCloud.Location;

size1 = size(labels(:,1));
size1 = size1(1);
object1 = zeros(size1,3);
allObjects = zeros(size1,3,numClusters);

for clustCnt = 1:1:numClusters
    tic
    cnt2 = 1;
    for cnt = 1:1:size(labels)
        if labels(cnt) == clustCnt
            %disp(locations(cnt,:));
            object1(cnt2,:) = locations(cnt,:);
            cnt2 = cnt2 + 1;
        end

    end
    
    allObjects(:,:,clustCnt) = object1;
    object1 = zeros(size1,3);
    toc

end

for counter = 1:1:numClusters
    figure(counter);
    pcshow(allObjects(:,:,counter));
    
end


figure(4);
pcshow(ptCloud.Location,labels)
colormap(hsv(numClusters))
title('Point Cloud Clusters')