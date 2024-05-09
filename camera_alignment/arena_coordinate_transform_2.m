%% Load in GUI and label the key points:
% feeder centers, water dish corners, center point

% update the file paths and cam_array_file as needed
clear all
close all;

codePath = 'C:\Users\ilow1\Documents\code\Label3D';
addpath(genpath(codePath))

alignPath = 'C:\Users\ilow1\Documents\code\il_rig_control\camera_alignment\';
cam_array_file = '240426_opt_cam_array_laser_pts.mat';
load(fullfile(alignPath, cam_array_file))

calib_img_date = '240507';
imgRoot = 'Z:\Isabel\arena\camera_calibration_files\';
imgFolder = [calib_img_date, '_calibration_videos'];
imgPath = fullfile(imgRoot, imgFolder);

% for saving key points etc.
save_dir = imgPath;
save_camera_ind = [calib_img_date, '_camera_ind.npy'];
save_point_ind = [calib_img_date, '_point_ind.npy'];
save_points_2d = [calib_img_date, '_points_2d.npy'];

% data params
camNames = {'red_cam', 'yellow_cam', 'green_cam', 'blue_cam'};
nFrames = 5;
nCams = length(camNames);
frame_idx = round(linspace(1, 420, nFrames));

% read in and reformat camera array
allParams = cell(nCams,1);
for cam_idx = 1:nCams
    f = optCamArray(cam_idx, 7);
    tmp = struct;
    prinpoint = optCamArray(cam_idx,10:11);
    tmp.K = cat(1,[f, 0, 0], [0, f, 0], [prinpoint, 1]);
    tmp.RDistort = optCamArray(cam_idx,8:9);
    tmp.TDistort = [0, 0];
    tmp.r = rotationVectorToMatrix(optCamArray(cam_idx,1:3));
    tmp.t = optCamArray(cam_idx,4:6);
    allParams{cam_idx} = tmp;
end

% read in images
videos = cell(nCams,1);
for cam_idx = 1:nCams
    disp(camNames(cam_idx))
    cam_folder = [camNames{cam_idx},'_images'];
    allImages = dir(fullfile(imgPath, cam_folder, '*.tiff'));
    thisFile = fullfile(allImages(1).folder, allImages(1).name);
    img = imread(thisFile);
    vid = zeros(size(img, 1), size(img, 2), 1, nFrames, 'uint8');
    for f = 1:nFrames
        thisFile = fullfile(allImages(f).folder, allImages(f).name);
        img = imread(thisFile);
        vid(:,:,f) = img(:, :, 1);
    end
    videos{cam_idx} = vid;
end

% define skeleton
skeleton.joint_names = {'center', 'red_feeder', 'yellow_feeder', 'green_feeder', 'blue_feeder',...
    'water_ry', 'water_yg', 'water_gb', 'water_br'};
skeleton.color = lines(16);
skeleton.joints_idx = repmat(cat(1, [2, 3], [3, 4], [4, 5], [5, 2],...
    [6, 1], [7, 1], [8, 1], [9, 1]), 2, 1);

% start GUI
labelGui = Label3D(allParams, videos, skeleton, 'defScale', .06);
colormap(labelGui.h{1}.Parent, 'gray'),

%% store the value, camera, and point index for each point
frame_idx = labelGui.frame;
key_pts = labelGui.camPoints(:, :, :, frame_idx);
n_pts = size(key_pts, 1);

pt_idx = 1:9;
for i = 1:nCams
    cam = camNames(i);
    pts = squeeze(key_pts(:, i, :));
    if i == 1
        camera_ind = ones(1, n_pts) * i;
        point_ind = pt_idx;
        points_2d = pts;
    else
        camera_ind = [camera_ind, ones(1, n_pts) * i];
        point_ind = [point_ind, pt_idx];
        points_2d = [points_2d; pts];
    end
end

% optionally adjust for pythonic indexing
camera_ind = camera_ind - 1;
point_ind = point_ind - 1;

%% save keypoints and indices as numpy arrays
writeNPY(points_2d, fullfile(save_dir, save_points_2d));
writeNPY(camera_ind, fullfile(save_dir, save_camera_ind));
writeNPY(point_ind, fullfile(save_dir, save_point_ind));

%% Find transform array to rotate and translate camera views into arena coords
% maybe not needed with new optimization protocol
arenaData = load('arena_im_2_4.mat');
cacheCent = cat(1,arenaData.caches.Centroid);% cacheCent = cacheCent([58,59,71,70],:);
tmp = (0:6:30) + [1,6]';
cacheCent = cacheCent(tmp(:),:);
dz = 0;
nFrame = labelGui.frame;
cnrPt = labelGui.points3D(:,:,nFrame);
trgPt = cacheCent;
trgPt(:,end+1) = 0;
% trgPt = cat(1,[-dx, -dx, 0],[-dx,dx,0],[dx,dx,0],[dx,-dx,0]);
% trgPt = cat(1,[-dx1, -dx1, 0],[-dx2, -dx2, 0],[-dx1,dx1,0],[-dx2,dx2,0],...
%     [dx1,dx1,0],[dx2,dx2,0],[dx1,-dx1,0],[dx2,-dx2,0]);
% trgPt = [];
% for i=1:4
%     trgPt = cat(1, trgPt, feederCent(i,:), cacheCent(i,:));
% end
% trgPt(:,end+1) = 0;

% find mean column wise
centroid = mean(cnrPt);

% calculate covariance matrix
H = (cnrPt-centroid)' * trgPt;

% find rotation
[U,S,V] = svd(H);
R = V*U';
% find translation
t = -R*centroid';
% find scale and correct translation vector to set z-height at dz
newPts = R * cnrPt' + t;
% s = sqrt(2*dx1^2)./mean(sqrt(sum(newPts(1:2,:).^2)));
s = sqrt(sum(reshape(trgPt(:,1:2),[],1).^2)) ./ sqrt(sum(reshape(newPts(1:2,:),[],1).^2));
t(3) = t(3) + dz/s;
% print new points
newPts = s * (R * cnrPt' + t),

figure,imshow(repmat(arenaData.arena_im,[1,1,3]),arenaData.arena_ref),hold on
for i = 1:6
    ind = (i-1)*2 + ([1,2]);
    plot(newPts(1,ind),newPts(2,ind)),
end
%% Project all cache sites onto arena for display
cacheCent = cat(1,arenaData.caches.Centroid);
cacheCent(:,3) = 0;
transformCent = R'*(cacheCent'/s - t);

for cam_idx = 1:6
    thisParam = labelGui.cameraParams{cam_idx};
    thisRot = thisParam.RotationMatrices;
    thisTrans = thisParam.TranslationVectors;
    imPts = worldToImage(thisParam, thisRot, thisTrans, transformCent', 'ApplyDistortion', true);
    figure,imshow(videos{cam_idx}(:,:,:,frame)),
    hold on,
    plot(imPts(:,1),imPts(:,2),'*'),
    title(sprintf('Cam # %d',cam_idx)),
end
