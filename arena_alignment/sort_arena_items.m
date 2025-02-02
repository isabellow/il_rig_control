%% sort arena items
% modified from SC v2.4 RigControl\arena alignment\sort_arena_items.mat
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
% area, centroid, convex hull, and orientation for key arena items:
%   all_perches
% 	perch_no_site (feeder perches)
% 	perch_w_site (cache perches)
% 	caches
% 	feeders
% 	water_dish
clear all; close all; clc

%% set paths, load arena model and reference object
data_dir = "C:\Users\ilow1\Documents\code\il_rig_control\arena_alignment\";
load(data_dir + "arena_model.mat");
load(data_dir + "ref_obj.mat");

%% find relevant items
% find all the arena items (white areas in image)
stats = regionprops(arena_model > 0.5,...
                    'Area', 'Centroid',...
                    'ConvexHull', 'Orientation');

% sort arena items by area
perches = stats([stats.Area] > 2e5 & [stats.Area] < 9e5);
caches = stats([stats.Area] > 6e4 & [stats.Area] < 2e5);
n_caches = length(caches);
feeders = stats([stats.Area] > 4e6);
water_dish = stats([stats.Area] > 9e5 & [stats.Area] < 4e6);
n_feeders = length(feeders);

%% convert to normalized coordinates
for i=1:length(perches)
    [perches(i).Centroid(1), perches(i).Centroid(2)] = arena_ref.intrinsicToWorld(...
        perches(i).Centroid(1),perches(i).Centroid(2));
    [perches(i).ConvexHull(:,1), perches(i).ConvexHull(:,2)] = arena_ref.intrinsicToWorld(...
        perches(i).ConvexHull(:,1),perches(i).ConvexHull(:,2));
end
for i=1:n_caches
    [caches(i).Centroid(1), caches(i).Centroid(2)] = arena_ref.intrinsicToWorld(...
         caches(i).Centroid(1), caches(i).Centroid(2));
    [caches(i).ConvexHull(:,1), caches(i).ConvexHull(:,2)] = arena_ref.intrinsicToWorld(...
         caches(i).ConvexHull(:,1), caches(i).ConvexHull(:,2));
end
for i=1:n_feeders
    [feeders(i).Centroid(1), feeders(i).Centroid(2)] = arena_ref.intrinsicToWorld(...
         feeders(i).Centroid(1), feeders(i).Centroid(2));
    [feeders(i).ConvexHull(:,1), feeders(i).ConvexHull(:,2)] = arena_ref.intrinsicToWorld(...
         feeders(i).ConvexHull(:,1), feeders(i).ConvexHull(:,2));
end
[water_dish.Centroid(1), water_dish.Centroid(2)] = arena_ref.intrinsicToWorld(...
     water_dish.Centroid(1), water_dish.Centroid(2));
[water_dish.ConvexHull(:,1), water_dish.ConvexHull(:,2)] = arena_ref.intrinsicToWorld(...
     water_dish.ConvexHull(:,1), water_dish.ConvexHull(:,2));

%% identify feeder perches vs. cache perches
feeder_perches = abs(abs([perches.Orientation])-45)<2;
perch_w_site = perches(~feeder_perches);
perch_no_site = perches(feeder_perches);
assert(length(perch_w_site) == n_caches,...
        "error: " + length(perch_w_site) + " perches for "...
        + n_caches + " caches!");

%% sort to pair each perch with its cache site
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

%% pair each feeder perch with its feeder
% feeder_perches = perch_no_site([1, 2, 3, 4]); % manually re-arrange to pair feeder and perch
fprintf('\n found %d improper perch-feeder matches \n', ...
    sum(sqrt(sum((cat(1, perch_no_site.Centroid) - cat(1, feeders.Centroid)).^2,2))>.2)),

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
save(data_dir + "arena_items_2.mat", ...
        "all_perches", "perch_no_site", "perch_w_site",...
            "caches", "feeders", "water_dish")