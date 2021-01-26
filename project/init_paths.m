% Initialization paths
% Modificare con il path completo dei programmi usati dalla pipeline

% cartella principale del progetto (full path, se Matlab non lo ha gia')
home_folder = '/home/ilaria/progetto/progetto_offset_assile/';

ply = '.ply';
ply_ps = 'ply/'; % ply path short
ply_pl = 'ply/'; % ply path long

% programmi
cut_prog = [home_folder 'cgal/Cut/cut '];
outlier_prog = [home_folder 'cgal/Outliers/outliers '];
norm_prog = [home_folder 'cgal/Normals/compute_onormals '];
detect_prog = [home_folder 'cgal/Detect_shape/RANSAC/detect_shapes_ransac --verbose --defaults ']; % default parameters
clear_prog = [home_folder 'cgal/Clear_shape/clear_shape ']; % add --keep-color to preserve planes
