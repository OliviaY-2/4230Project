% Convert images from .mat file to .png
% Author: Lucas Way z5164204
% First made (DD/MM/YYY): 25/07/2020
%{
Taking a .mat file created using ROS_Connect.m, the image will be converted
to a png and saved in a new file

Reference:

Instructions:
    ---

Edit History:
25/07/2020 created file


%}
close all;

%%
% Specify the folder where the files live.
myFolder = 'C:\Users\User\Documents\UNSW\MTRN4230\Git Repo\4230Project\RGBD_Data\Red Cube (New)';
% Check to make sure that folder actually exists.  Warn user if it doesn't.
if ~isfolder(myFolder)
    errorMessage = sprintf('Error: The following folder does not exist:\n%s\nPlease specify a new folder.', myFolder);
    uiwait(warndlg(errorMessage));
    myFolder = uigetdir(); % Ask for a new one.
    if myFolder == 0
         % User clicked Cancel
         return;
    end
end
% Get a list of all files in the folder with the desired file name pattern.
PNGFolder = 'C:\Users\User\Documents\UNSW\MTRN4230\Git Repo\4230Project\RGBD_Data\Red Cube PNG';
filePattern = fullfile(myFolder, '*.mat'); % Change to whatever pattern you need.
theFiles = dir(filePattern);
for k = 1 : length(theFiles)
    baseFileName = theFiles(k).name;
    fullFileName = fullfile(theFiles(k).folder, baseFileName);
    fprintf(1, 'Now reading %s\n', fullFileName);
    % Now do whatever you want with this file name,
    % such as reading it in as an image array with imread()
    imageArray = load(fullFileName);
    imshow(imageArray.image);  % Display image.
    drawnow; % Force display to update immediately.
    PNGbaseFileName = strrep(baseFileName, '.mat', '.png');
    PNGfullFileName = fullfile(PNGFolder, PNGbaseFileName);
    imwrite(imageArray.image, PNGfullFileName);
end