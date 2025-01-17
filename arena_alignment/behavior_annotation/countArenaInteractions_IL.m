function data = countArenaInteractions_IL(smPts, footSpd, bodyReproj, arenaDat)
%%
% outputs primal action detections for IL v1 arena grid design.
% Note that interactions are preprocessed to prevent
% transient irregular or false interactions.

%% set params
params.reprojThresh = 10; % maximum reproj error in count as valid frame (px)
params.speedThresh = 1/2; % threshold for feet 'not moving' (norm units/sec, ie output of kalman filter)
params.cacheHeightThresh = 0.02; % threshold for beak low enough for site interaction
params.mergeDurThresh = 50; % threshold below which events at same site *must* be merged (in frames at 50fps)
params.feederHeightThresh = .05; % threshold for beak low enough for feeder interaction
params.feederRadiusThresh = 1.75/15; % threshold for beak close enough to center for feeder interaction - todo needed?
params.waterHeightThresh = .06; % threshold for beak low enough for water interaction
params.waterRadiusThresh = .75/15; % threshold for beak close enough to center for water dish interaction - todo needed?
params.beakFootDistThresh = 0.029; % distance threshold to count beak and feet near enough for eating
params.cacheRadiusTol = 1.005; % scale factor to adjust cache site locations (>1 further from arena center) to better match center of beak interactions - todo needed?
params.stateMedianWin = 5; % median window for filtering state status to prevent transient blips

%% calculate temporary variables
beakPos = mean(smPts(:, :, [1, 2]), 3);
footPos = mean(smPts(:, :, [13, 18]), 3);
beak_foot_dist = sqrt(sum((beak_pos - foot_pos).^2, 2));
nFrames = size(beak_pos, 1);
nCacheSites = length(arenaDat.caches);
nPerches = length(arenaDat.perches);
nFeederPerches = length(arenaDat.feeder_perches);

%% utility indicators
validFrames = bodyReproj < params.reprojThresh;
feetStill = footSpd < params.speedThresh;
beakLow_cache = beak_pos(:, 3) < params.cacheHeightThresh;
beakLow_feeder = beak_pos(:, 3) < params.feederHeightThresh;
beakLow_water = beak_pos(:, 3) < params.waterHeightThresh;
beakLow_feet = beak_foot_dist < params.beakFootDistThresh;
beakRadius_water = sqrt(sum(beak_pos(:, 1:2).^2, 2)) < params.waterRadiusThresh;
beakRadius_feeder = sqrt(sum(beak_pos(:, 1:2).^2, 2)) < params.feederRadiusThresh;

% bounding boxes and feeder/water radius
sitePerchBoxes = cat(3,arenaDat.perchWithSite.BoundingBox);
nositePerchBoxes = cat(3,arenaDat.perchNoSite.BoundingBox);
cacheBoxes = cat(3,arenaDat.caches.BoundingBox);
feederCentroids = cat(3,arenaDat.feeders.Centroid);
feederRadius = mean(squeeze(diff(cat(3,arenaDat.feeders.BoundingBox))))/2;
waterTrayCentroids = cat(3,arenaDat.waterTray.Centroid);
waterTrayRadius = mean(squeeze(diff(cat(3,arenaDat.waterTray.BoundingBox))))/2;
% 'is in box' matrix calculation (for perches and cache sites)
footSitePerchMat = isInBox(footPos, sitePerchBoxes);
footNositePerchMat = isInBox(footPos, nositePerchBoxes);
beakSitePerchMat = isInBox(beakPos, sitePerchBoxes);
beakNositePerchMat = isInBox(beakPos, nositePerchBoxes);
beakCacheMat = isInBox(beakPos, cacheBoxes);
% 'is within radius' calculation
beakFeederMat = isInRadius(beakPos, feederCentroids, mean(feederRadius));
beakWatertrayMat = isInRadius(beakPos, waterTrayCentroids, mean(waterTrayRadius));
% define state vectors
footOnSitePerch = any(footSitePerchMat,2) & feetStill & validFrames;
footOnNositePerch = any(footNositePerchMat,2) & feetStill & validFrames;
beakOnSitePerch = any(beakSitePerchMat,2) & beakLow_perch & feetStill & validFrames;
beakOnNositePerch = any(beakNositePerchMat,2) & beakLow_perch & feetStill & validFrames;
beakOnCache = any(beakCacheMat,2) & beakLow_cache & feetStill & validFrames;
beakOnFeeder = any(beakFeederMat,2) & beakLow_cache & feetStill & validFrames;
beakOnWatertray = beakWatertrayMat & beakLow_cache & feetStill & validFrames;

% correct for possible beakOnNositePerch bounding box being too generous by
% excluding any times when beakOnFeeder
beakOnNositePerch = beakOnNositePerch & ~beakOnFeeder;

%% detect state changes

% combine all perches to get unique perch visits
perched = footOnSitePerch | footOnNositePerch;
footPerchMat = cat(2, footSitePerchMat, footNositePerchMat);
[newPerch, endPerch, perchNum] = detectStateChanges_selfmerge(perched, footPerchMat);

% for water and feeder detections, merge all sequential events with the same
% object unless the bird has moved btw perches
[newFeeder, endFeeder, feederNum] = detectStateChanges_othermerge(...
    beakOnFeeder, beakFeederMat, newPerch, params.mergeDurThresh);
[newWater, endWater, waterNum] = detectStateChanges_othermerge(...
    beakOnWatertray, beakWatertrayMat, newPerch, params.mergeDurThresh);

% for site interactions, do not merge if bird eats (or moves btw perches) btw interactions, 
% and vice-versa for perch/eating interactions
% combine beak on noSite and Site perches
beakPerch = beakOnSitePerch | beakOnNositePerch;
beakPerchMat = cat(2, beakSitePerchMat, beakNositePerchMat);
eatMergeExclusion = cat(1, newPerch, find(diff([false;beakOnCache])>.5));
[newBeakPerch, endBeakPerch, beakPerchNum] = detectStateChanges_othermerge(...
    beakPerch, beakPerchMat, eatMergeExclusion, params.mergeDurThresh);
checkMergeExclusion = cat(1, newPerch, find(diff([false;beakPerch])>.5));
[newSite, endSite, siteNum] = detectStateChanges_othermerge(...
    beakOnCache, beakCacheMat, checkMergeExclusion, params.mergeDurThresh);

% give feeder perches negative object numbers
nSitePerches = size(footSitePerchMat,2);
perchNum(perchNum>nSitePerches) = nSitePerches-perchNum(perchNum>nSitePerches);
beakPerchNum(beakPerchNum>nSitePerches) = nSitePerches-beakPerchNum(beakPerchNum>nSitePerches);

%%
data.newPerch = newPerch; data.endPerch = endPerch; data.perchNum = perchNum;
data.newSite = newSite; data.endSite = endSite; data.siteNum = siteNum;
data.newFeeder = newFeeder; data.endFeeder = endFeeder; data.feederNum = feederNum;
data.newWater = newWater; data.endWater = endWater; data.waterNum = waterNum;
data.newBeakPerch = newBeakPerch; data.endBeakPerch = endBeakPerch; data.beakPerchNum = beakPerchNum;

data.params = params;
data.footOnSitePerch = footOnSitePerch;
data.footOnNositePerch = footOnNositePerch;
data.beakOnSitePerch = beakOnSitePerch;
data.beakOnNositePerch = beakOnNositePerch;
data.beakOnCache = beakOnCache;
data.beakOnFeeder = beakOnFeeder;
data.beakOnWatertray = beakOnWatertray;

end

function inBoxMat = isInBox(bodyPart, boundingBoxes)
    inBoxMat = squeeze(bodyPart(:,1)>=boundingBoxes(1,1,:) & bodyPart(:,1)<=boundingBoxes(2,1,:) ...
        & bodyPart(:,2)>=boundingBoxes(1,2,:) & bodyPart(:,2)<=boundingBoxes(2,2,:));
end

function inRadiusMat = isInRadius(bodyPart, centroids, radius)
    inRadiusMat = squeeze(sqrt(sum((bodyPart(:,1:2)-centroids).^2,2))) < radius;
end

function [onsetTimes, offsetTimes, objNum] = detectStateChanges_selfmerge(stateVector, stateMatrix)
    % count state changes, including start and final
    onsetTimes = find(diff([false;stateVector;false])>.5);
    offsetTimes = find(diff([false;stateVector;false])<-.5);
    [objNum,~] = find(stateMatrix(onsetTimes,:)');

    % Correct for movements which do not move between perches
    assert(all(offsetTimes-onsetTimes>1/2)), % check that we've got our state crossings right
    sameObj = find(diff(objNum)==0);
    onsetTimes(sameObj+1) = []; %next entry is false
    offsetTimes(sameObj) = []; %this exit is false
    [objNum,~] = find(stateMatrix(onsetTimes,:)');
end

function [onsetTimes, offsetTimes, objNum] = detectStateChanges_othermerge(stateVector, stateMatrix, otherTimes, durThresh)
    % count state changes, including start and final
    onsetTimes = find(diff([false;stateVector;false])>.5);
    offsetTimes = find(diff([false;stateVector;false])<-.5);
    interEventDur = onsetTimes(2:end)-offsetTimes(1:end-1);
    [objNum,~] = find(stateMatrix(onsetTimes,:)');
    assert(all(offsetTimes-onsetTimes>1/2)), % check that we've got our state crossings right
    
    sameObj = find(diff(objNum)==0);
    % for each interaction with same obj, look for intervening otherTimes
    mergeSame = true(length(sameObj),1);
    for i = 1:length(sameObj)
        nInt = sameObj(i);
        if any(otherTimes>onsetTimes(nInt) & otherTimes<onsetTimes(nInt+1)) && interEventDur(nInt) > durThresh
            % must have interposing event and minimum inter-event interval
            mergeSame(i) = false;
        else
            mergeSame(i) = true;
        end
    end
    mergeInteractions = sameObj(mergeSame);
    
    % merge interactions selected above
    onsetTimes(mergeInteractions+1) = []; %next entry is false
    offsetTimes(mergeInteractions) = []; %this exit is false
    [objNum,~] = find(stateMatrix(onsetTimes,:)');
end