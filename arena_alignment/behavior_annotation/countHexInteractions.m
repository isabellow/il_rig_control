
function data = countHexInteractions(smPts, footSpd, bodyReproj, arenaDat)
%%
% outputs primal action detections for SC v2.4 arena hex design.
% Note that interactions are a bit preprocessed to prevent
% transient irregular or false interactions.

% perch numbers 1-36 are 6 arms, 37-42 are 6 feeder perches
% cache site numbers 1-36 are not matched to perch numbers
for n = 1:6
    data.arms(n).perches = [(n-1)*6 + (1:6), n + 36];
    data.arms(n).sites = (n-1)*6 + (1:6);
end

%% set params
params.reprojThresh = 10; % maximum reproj error in count as valid frame (px)
params.speedThresh = 1/2; % threshold for feet 'not moving' (norm units/sec, ie output of kalman filter)
params.cacheHeightThresh = 0.02; % threshold for beak low enough for site interaction
params.mergeDurThresh = 60; % threshold below which events at same site *must* be merged (in frames at 60fps)
params.feederHeightThresh = .05; % threshold for beak low enough for feeder interaction
params.feederRadiusThresh = 1.75/15; % threshold for beak close enough to center for feeder interaction
params.waterHeightThresh = .06; % threshold for beak low enough for water interaction
params.waterRadiusThresh = .75/15; % threshold for beak close enough to center for water dish interaction
params.beakFootDistThresh = 0.025; % distance threshold to count beak and feet near enough for eating
params.cacheRadiusTol = 1.005; % scale factor to adjust cache site locations (>1 further from arena center) to better match center of beak interactions
params.stateMedianWin = 5; % median window for filtering state status to prevent transient blips

%% calculate temporary variables
beak_pos = mean(smPts(:,:,[1,2]),3);
foot_pos = mean(smPts(:,:,[13,18]),3);
beak_foot_dist = sqrt(sum((beak_pos-foot_pos).^2,2));
nFrames = size(beak_pos,1);
nCacheSites = length(arenaDat.caches);
nPerches = length(arenaDat.perches);
nFeederPerches = length(arenaDat.feeder_perches);

%% utility indicators
validFrames = bodyReproj < params.reprojThresh;
feetStill = footSpd < params.speedThresh;
beakLow_cache = beak_pos(:,3) < params.cacheHeightThresh;
beakLow_feeder = beak_pos(:,3) < params.feederHeightThresh;
beakLow_water = beak_pos(:,3) < params.waterHeightThresh;
beakLow_feet = beak_foot_dist < params.beakFootDistThresh;
beakRadius_water = sqrt(sum(beak_pos(:,1:2).^2,2)) < params.waterRadiusThresh;
beakRadius_feeder = sqrt(sum(beak_pos(:,1:2).^2,2)) < params.feederRadiusThresh;
beakRadius_feeder = beakRadius_feeder & ~ beakRadius_water; % exclude when beak is central enough for water dish

%% detection matrices

feetOnPerch = false(nFrames, nPerches + nFeederPerches);
for n = 1:nPerches
    tmp = inpolygon(foot_pos(:,1),foot_pos(:,2),arenaDat.perches(n).ConvexHull(:,1),arenaDat.perches(n).ConvexHull(:,2));
    feetOnPerch(:,n) = tmp & feetStill & validFrames;
end
for n = 1:nFeederPerches
    tmp = inpolygon(foot_pos(:,1),foot_pos(:,2),arenaDat.feeder_perches(n).ConvexHull(:,1),arenaDat.feeder_perches(n).ConvexHull(:,2));
    feetOnPerch(:,nPerches+n) = tmp & feetStill & validFrames;
end

beakOnFeet = false(nFrames, nPerches + nFeederPerches);
for n = 1:nPerches
    tmp = inpolygon(beak_pos(:,1),beak_pos(:,2),arenaDat.perches(n).ConvexHull(:,1),arenaDat.perches(n).ConvexHull(:,2));
    beakOnFeet(:,n) = tmp & beakLow_feet & feetStill & validFrames;
end
for n = 1:nFeederPerches
    tmp = inpolygon(beak_pos(:,1),beak_pos(:,2),arenaDat.feeder_perches(n).ConvexHull(:,1),arenaDat.feeder_perches(n).ConvexHull(:,2));
    beakOnFeet(:,nPerches+n) = tmp & beakLow_feet & feetStill & validFrames;
end

% for cache sites, scale cache site convex hull by factor
beakOnCache = false(nFrames, nCacheSites);
for n = 1:nCacheSites
    tmp = inpolygon(beak_pos(:,1),beak_pos(:,2),...
        params.cacheRadiusTol*arenaDat.caches(n).ConvexHull(:,1),params.cacheRadiusTol*arenaDat.caches(n).ConvexHull(:,2));
    beakOnCache(:,n) = tmp & beakLow_cache & feetStill & validFrames;
end

beakOnFeeder = beakLow_feeder & beakRadius_feeder & feetStill & validFrames;
beakOnWater = beakLow_water & beakRadius_water & feetStill & validFrames;
% median filter to prevent blips confusing these two
beakOnFeeder = logical(movmedian(beakOnFeeder,params.stateMedianWin,1));
beakOnWater = logical(movmedian(beakOnWater,params.stateMedianWin,1));

% median filter perch, eating and cache site interaction status
fprintf('\n median state filtering with %d frames\n', params.stateMedianWin)
feetOnPerch = logical(movmedian(feetOnPerch,params.stateMedianWin,1));
beakOnFeet = logical(movmedian(beakOnFeet,params.stateMedianWin,1));
beakOnCache = logical(movmedian(beakOnCache,params.stateMedianWin,1));

%% Merge Operations
% define exclusion times
exc_eat = find(diff([false;any(beakOnFeet,2);false])>.5);
exc_site = find(diff([false;any(beakOnCache,2);false])>.5);
exc_feed = find(diff([false;any(beakOnFeeder,2);false])>.5);
exc_water = find(diff([false;any(beakOnWater,2);false])>.5);

% always merge perch interactions at same site
[newPerch, endPerch, perchNum] = detectStateChanges_selfmerge(feetOnPerch);

% merge feeder/water interactions unless the bird does something else btw
tmp = cat(1,newPerch,exc_eat,exc_site,exc_water);
[newFeeder, endFeeder, feederNum] = detectStateChanges_othermerge(...
    beakOnFeeder, tmp, params.mergeDurThresh);
tmp = cat(1,newPerch,newFeeder,exc_eat,exc_site);
[newWater, endWater, waterNum] = detectStateChanges_othermerge(...
    beakOnWater, tmp, params.mergeDurThresh);

% merge eating interactions
tmp = cat(1,newPerch,newFeeder,newWater,exc_site);
[newBeakPerch, endBeakPerch, beakPerchNum] = detectStateChanges_othermerge(...
    beakOnFeet, tmp, params.mergeDurThresh);

% merge cache site interactions
tmp = cat(1,newPerch,newWater,newFeeder,newBeakPerch);
[newSite, endSite, siteNum] = detectStateChanges_othermerge(...
    beakOnCache, tmp, params.mergeDurThresh);

%% Collect Data
data.newPerch = newPerch; data.endPerch = endPerch; data.perchNum = perchNum;
data.newSite = newSite; data.endSite = endSite; data.siteNum = siteNum;
data.newFeeder = newFeeder; data.endFeeder = endFeeder; data.feederNum = feederNum;
data.newWater = newWater; data.endWater = endWater; data.waterNum = waterNum;
data.newBeakPerch = newBeakPerch; data.endBeakPerch = endBeakPerch; data.beakPerchNum = beakPerchNum;
data.params = params;

%% helper functions

function [onsetTimes, offsetTimes, objNum] = detectStateChanges_selfmerge(stateMatrix)
    % count state changes, including start and final
    stateVector = any(stateMatrix,2);
    onsetTimes = find(diff([false;stateVector;false])>.5);
    offsetTimes = find(diff([false;stateVector;false])<-.5);
    [objNum,~] = find(stateMatrix(onsetTimes,:)');
    assert(length(objNum)==length(onsetTimes)), % make sure we found an entry for each onset

    % Correct for movements which do not move between perches
    assert(all(offsetTimes-onsetTimes>1/2)), % check that we've got our state crossing times right
    sameObj = find(diff(objNum)==0);
    onsetTimes(sameObj+1) = []; %next entry is false
    offsetTimes(sameObj) = []; %this exit is false
    [objNum,~] = find(stateMatrix(onsetTimes,:)');
    assert(length(objNum)==length(onsetTimes)), % make sure we found an entry for each onset
end

function [onsetTimes, offsetTimes, objNum] = detectStateChanges_othermerge(stateMatrix, otherTimes, durThresh)
    % count state changes, including start and final
    stateVector = any(stateMatrix,2);
    onsetTimes = find(diff([false;stateVector;false])>.5);
    offsetTimes = find(diff([false;stateVector;false])<-.5);
    interEventDur = onsetTimes(2:end)-offsetTimes(1:end-1);
    [objNum,~] = find(stateMatrix(onsetTimes,:)');
    assert(length(objNum)==length(onsetTimes)), % make sure we found an entry for each onset
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

end