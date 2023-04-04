%% format arena model image
% define paths
data_dir = "Z:\Isabel\data\behavior\";
ref_im_file = "ref_image.tiff";
arena_map_file = "arena_model-01.png";
save_dir = "C:\Users\arono\Documents\code\il_rig_control\arena_alignment\";
addpath(data_dir)

%% load arena model image and convert to a binary uint8 array
[raw_map, ~, ~] = imread(data_dir + arena_map_file);
arena_model = imbinarize(raw_map, "global");
arena_model = arena_model(:, :, 1);

%% save map and referencing object
arena_ref = imref2d(size(arena_model), [-1 1], [-1 1]);
save(save_dir + "arena_model.mat", 'arena_model')
save(save_dir + "ref_obj.mat", 'arena_ref')