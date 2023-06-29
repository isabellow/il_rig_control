%% from the RealignCentroidsButton function in view_bottom_camera
disp('Re-computing Bottom Camera Alignment')

% load files
root_dir = "C:\Users\arono\Documents\code\il_rig_control\arena_alignment\";
load(root_dir + "arena_model.mat"),
load(root_dir + "ref_obj.mat"),
load(root_dir + "arena_items.mat"),
load(root_dir + "bot_cam_intrinsics.mat")

% set anchor points for alignment
anchorCaches = [1, 2, 5, 6, 9, 10,...
                13, 14, 21, 22,...
                37, 38, 47, 48,...
                63, 64, 71, 72,...
                75, 76, 79, 80, 83, 84];
nSites = length(anchorCaches);
cache_colors = lines(nSites);

% reload uncropped data (cap at 10 images, no need to read all?)
baseFolder = "Z:\Isabel\data\acquisition\bot_calibration";
allImages = dir(fullfile(baseFolder,'botImages','*.tiff'));
[~,imOrder] = sort([allImages.datenum],'ascend');
allImages = allImages(imOrder);
nImages = min(length(allImages),10);
imInfo = imfinfo(fullfile(allImages(1).folder,allImages(1).name));
w = imInfo.Width;
h = imInfo.Height;
refIm = zeros(h,w,nImages,'uint8');
for i=1:nImages
    thisFile = fullfile(allImages(i).folder,allImages(i).name);
    refIm(:,:,i) = imread(thisFile);
end
refIm = uint8(mean(refIm,3));

% Manually adjust Cache Locations
h1=figure;
imshow(arena_model, arena_ref),
arenaVertices=[];
for thisCache = 1:nSites
    thisSite = caches(anchorCaches(thisCache)).Centroid;
    h = images.roi.Point(gca,'Position',thisSite,'Color',cache_colors(thisCache,:));
    arenaVertices = cat(1,arenaVertices, h.Position);
%                 thisBox = caches(anchorCaches(thisCache)).BoundingBox;
%                 thisBox = [thisBox(1,:),thisBox(2,:)-thisBox(1,:)];
%                 h = images.roi.Rectangle(gca,'Position',thisBox,'Color',col(thisCache,:));
%                 arenaVertices = cat(1,arenaVertices, h.Vertices);
end
h2=figure;
imshow(refIm),
clear h
for i=1:nSites
    display(i),
    h(i) = drawpoint(gca,'Color',cache_colors(i,:));
end
display('Waiting to Confirm Locations'),
pause,
imageVertices = cat(1,h.Position);
close(h1),close(h2),

% Compute Alignment w/ and w/out distortion
cacheLoc = cat(1,caches.Centroid);
figure,imshow(refIm),hold on,
% no distortion
tform = fitgeotrans(arenaVertices,imageVertices,'projective');
[cacheLocBot(:,1), cacheLocBot(:,2)] = tform.transformPointsForward(cacheLoc(:,1), cacheLoc(:,2));
plot(cacheLocBot(:,1),cacheLocBot(:,2),'*'),
% w/ distortion
pp = botCamInt.PrincipalPoint;
fl = mean(botCamInt.FocalLength);
imgPts_undistorted = undistortPoints(imageVertices,botCamInt);
tform = fitgeotrans(arenaVertices,imgPts_undistorted,'projective');
[cacheLocBot(:,1), cacheLocBot(:,2)] = tform.transformPointsForward(cacheLoc(:,1), cacheLoc(:,2));
normPts = (cacheLocBot-pp)./fl;
r = sqrt(sum(normPts.^2,2));
rDistort = 1 + botCamInt.RadialDistortion(1).* (r.^2) + botCamInt.RadialDistortion(2).* (r.^4);
cacheLocBot_distort = fl.*(normPts.*rDistort) + pp;
plot(cacheLocBot_distort(:,1),cacheLocBot_distort(:,2),'*'),

doSave = input('Accept & Save Results?');
app.cacheLocBot = cacheLocBot_distort;
if doSave
    % Save results
    SaveCentroidsButtonPushed(app, event);
    % Re-load images using new results
    LoadImagesButtonPushed(app, event);
end