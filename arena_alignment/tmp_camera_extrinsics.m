clear all,clc,

% translation vector
d = sqrt(2); % normalized distance of each camera from the arena center
h = 1/2; % height of each camera above the arena
cam_trans = [0, h, d];

% define a set of points in world coords that are easy to translate
% into camera coords
% these are the corners ordered red, yellow, green, blue
% plus a point camera height above the arena center
world_pts = cat(1, [1, 1, 0], [-1, 1, 0], [-1, -1, 0], [1, -1, 0], [0, 0, h]);

% find the camera coords of the points for the camera in each corner
red_pts = cat(1, [0, h, 0], [-d, h, d], [0, h, 2*d], [d, h, d], [0, 0, d]);
yellow_pts = cat(1, [d, h, d], [0, h, 0], [-d, h, d], [0, h, 2*d], [0, 0, d]);
green_pts = cat(1, [0, h, 2*d], [d, h, d], [0, h, 0], [-d, h, d], [0, 0, d]);
blue_pts = cat(1, [-d, h, d], [0, h, 2*d], [d, h, d], [0, h, 0], [0, 0, d]);

% solve the system of equations to get the rotation matrix that tranforms
% world points into each camera's frame of reference
red_rot = mldivide(world_pts, red_pts - cam_trans);
yellow_rot = mldivide(world_pts, yellow_pts - cam_trans);
green_rot = mldivide(world_pts, green_pts - cam_trans);
blue_rot = mldivide(world_pts, blue_pts  - cam_trans);

% convert into a vector and display
red_vec = rotationMatrixToVector(red_rot);
yellow_vec = rotationMatrixToVector(yellow_rot);
green_vec = rotationMatrixToVector(green_rot);
blue_vec = rotationMatrixToVector(blue_rot);