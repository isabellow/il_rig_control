%% sort arena items
% modified from SC v1.1 RigControl\arena alignment\sort_arena_items.mat
%
% classifies perches, caches, dishes
% converts centroids to normalized coords
% normalizes and reformats bounding boxes
%
% Params (from format_arena_image):
% ---------------------------------
%   arena_model : binary matrix, size (n_pixels, n_pixels)
%       binary image of arena model with all items in white
%   arena_ref : reference object
%       converts from image coordinates to normalized coordinates
%       created using imref2d(size(arena_model), [-1 1], [-1 1])
%
% Returns:
% --------
% area, centroid, bounding box, and orientation for key arena items:
%   all_perches
% 	perch_no_site (feeder perches)
% 	perch_w_site (cache perches)
% 	caches
% 	feeders
% 	water_dish
clear all; close all; clc

%% set paths, load arena model and reference object
data_dir = "C:\Users\arono\Documents\code\il_rig_control\arena_alignment\";
load(data_dir + "arena_model.mat");
load(data_dir + "ref_obj.mat");

%% find relevant items
% find all the arena items (white areas in image)
stats = regionprops(arena_model > 0.5,...
                    'Area', 'Centroid',...
                    'BoundingBox', 'Orientation');

% perches and caches - larger than screws, smaller than dishes
perches = stats([stats.Area] > 2e5 & [stats.Area] < 9e5);
caches = stats([stats.Area] > 6e4 & [stats.Area] < 2e5);
n_caches = length(caches);

% feeders and water dish
feeders = stats([stats.Area] > 4e6);
water_dish = stats([stats.Area] > 9e5 & [stats.Area] < 4e6);
n_feeders = length(feeders);

%% identify feeder perches vs. cache perches
feeder_perches = abs(abs([perches.Orientation])-45)<2;
perch_w_site = perches(~feeder_perches);
perch_no_site = perches(feeder_perches);
assert(length(perch_w_site) == n_caches,...
        "error: " + length(perch_w_site) + " perches for "...
        + n_caches + " caches!");

%% organize perches and caches
% convert to normalized coords, reformat bounding boxes
for i=1:n_caches
    perch_w_site(i).Centroid = perch_w_site(i).Centroid *...
        arena_ref.PixelExtentInWorldX-1;
    old_box = perch_w_site(i).BoundingBox;
    old_box(3:4) = old_box(1:2)+old_box(3:4);
    perch_w_site(i).BoundingBox = reshape(old_box,...
        [2,2])' * arena_ref.PixelExtentInWorldX-1;
    
    caches(i).Centroid = caches(i).Centroid *...
        arena_ref.PixelExtentInWorldX-1;
    old_box = caches(i).BoundingBox;
    old_box(3:4) = old_box(1:2)+old_box(3:4);
    caches(i).BoundingBox = reshape(old_box,...
        [2,2])' * arena_ref.PixelExtentInWorldX-1;
end

% sort to pair each perch with its cache site
perch_centers = cat(1, perch_w_site.Centroid);
cache_centers = cat(1, caches.Centroid);
h = histogram(cache_centers(:, 1), 11);
edges = h.BinEdges;
bin_edges = cat(2, edges(1:5), cat(2, 0, edges(8:end)));
for col=1:length(bin_edges)-1
    % find perches and caches for a given arena column
    perch_idx = perch_centers(:, 1) < bin_edges(col+1) &...
                                perch_centers(:, 1) > bin_edges(col);
    cache_idx = cache_centers(:,1) < bin_edges(col+1) &...
                                cache_centers(:,1) > bin_edges(col);
    assert(sum(perch_idx) == sum(cache_idx)) % same num perches and caches per col
    
    % sort perches
    these_perches = perch_w_site(perch_idx);
    tmp = cat(1, these_perches.Centroid);
    [~,sort_idx] = sort(tmp(:, 2));
    perch_w_site(perch_idx) = these_perches(sort_idx);
    
    % sort caches
    these_cache = caches(cache_idx);
    tmp = cat(1,these_cache.Centroid);
    [~,sort_idx] = sort(tmp(:,2));
    caches(cache_idx) = these_cache(sort_idx);
end
perch_centers = cat(1, perch_w_site.Centroid);
cache_centers = cat(1, caches.Centroid);
fprintf('\n found %d improper perch-cache matches \n', ...
        sum(sqrt(sum((perch_centers - cache_centers).^2, 2)) > .05))

%% organize feeders and feeder perches
% convert to normalized coords, reformat bounding boxes
for i=1:n_feeders
    perch_no_site(i).Centroid = perch_no_site(i).Centroid *...
        arena_ref.PixelExtentInWorldX-1;
    old_box = perch_no_site(i).BoundingBox;
    old_box(3:4) = old_box(1:2)+old_box(3:4);
    perch_no_site(i).BoundingBox = reshape(old_box,...
        [2,2])' * arena_ref.PixelExtentInWorldX-1;
    
    feeders(i).Centroid = feeders(i).Centroid *...
        arena_ref.PixelExtentInWorldX-1;
    old_box = feeders(i).BoundingBox;
    old_box(3:4) = old_box(1:2)+old_box(3:4);
    feeders(i).BoundingBox = reshape(old_box,...
        [2,2])' * arena_ref.PixelExtentInWorldX-1;
end

% pair each perch with its feeder
% perch_no_site = perch_no_site([1, 2, 3, 4]); % manually re-arrange to pair feeder and perch
fprintf('\n found %d improper perch-feeder matches \n', ...
    sum(sqrt(sum((cat(1, perch_no_site.Centroid) - cat(1, feeders.Centroid)).^2,2))>.2)),

% normalize and reformat water dish coords
water_dish.Centroid = water_dish.Centroid * arena_ref.PixelExtentInWorldX-1;
old_box = water_dish.BoundingBox;
old_box(3:4) = old_box(1:2)+old_box(3:4);
water_dish.BoundingBox = reshape(old_box,...
    [2,2])' * arena_ref.PixelExtentInWorldX-1;

%% plot to check alignment
figure,imshow(arena_model, arena_ref)
hold on
tmp = cat(1,caches.Centroid); plot(tmp(:,1),tmp(:,2),'*'),
tmp = cat(1,perch_w_site.Centroid); plot(tmp(:,1),tmp(:,2),'*'),
tmp = cat(1,perch_no_site.Centroid); plot(tmp(:,1),tmp(:,2),'*')
tmp = cat(1,feeders.Centroid); plot(tmp(:,1),tmp(:,2),'*')
plot(water_dish.Centroid(1), water_dish.Centroid(2),'*'),

%% save everything
all_perches = cat(1, perch_w_site, perch_no_site);
save(data_dir + "arena_items.mat", ...
        "all_perches", "perch_no_site", "perch_w_site",...
            "caches", "feeders", "water_dish")