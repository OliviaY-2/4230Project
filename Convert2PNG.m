% Convert images from .mat file to .png
% Author: Lucas Way z5164204
% First made (DD/MM/YYY): 25/07/2020
%{
Taking a .mat file created using ROS_Connect.m, the image will be converted
to a png and saved in a new file

Reference:
    https://matlab.fandom.com/wiki/FAQ#How_can_I_process_a_sequence_of_files.3F
Instructions:
    - change the name for 'myFolder' so that it finds the .mat files in the
    desired folder
    - change the name for 'PNGFolder' so that it saves the .png in the new
    folder

Edit History:
    25/07/2020 created file

%}
close all;

%%
% Specify the folder where the files live.
myFolder = 'C:\Users\User\Documents\UNSW\MTRN4230\Git Repo\4230Project\RGBD_Data\Multiple Objects';
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
PNGFolder = 'C:\Users\User\Documents\UNSW\MTRN4230\Git Repo\4230Project\RGBD_Data\PNG Multiple Objects';
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
    % change name so that it is saved as a png
    PNGbaseFileName = strrep(baseFileName, '.mat', '.png');
    PNGfullFileName = fullfile(PNGFolder, PNGbaseFileName);
    % save to the folder
    imwrite(imageArray.image, PNGfullFileName);
end