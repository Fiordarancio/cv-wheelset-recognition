clear all
close all
clc

% PREPARAZIONE DEI FILE PLY PER CGAL
% apro i file rosbag
bag = [ rosbag('2019-06-13-16-03-53.bag');
        rosbag('scansione_freno_short.bag');
        rosbag('2019-08-02-12-05-08.bag');
];  
bagname = [ "b_190613"; "scan_freno_short"; "b_190802"];

for i=1:3
    bag_select = select(bag(i),'Topic','/rtabmap/cloud_map');

    %Estraggo i messaggi che interessano, ad es l'ultimo 
    %pcd_array = readMessages(bag_select,'DataFormat','struct');
    pcd = readMessages(bag_select, round(bag_select.NumMessages*0.99));

    %Per la pointcloud:
    %Converto il messaggio in array di punti (single) e salvo il ply
    pcd{1}.PreserveStructureOnRead = 1;
    pcobj = pointCloud(readXYZ(pcd{1}),'Color',uint8(255*readRGB(pcd{1})));
    pcwrite(pcobj, "ply/"+bagname(i), 'PLYFormat', 'binary');
end