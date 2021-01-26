clear all
close all
clc

%apro i file rosbag
bag1 = rosbag('2019-06-13-16-03-53.bag'); %Buona globale
bag2 = rosbag('scansione_freno_short.bag'); %Buona globale
bag3 = rosbag('2019-08-02-12-05-08.bag'); %Buona globale


bag_select1 = select(bag1,'Topic','/rtabmap/cloud_map');
bag_select2 = select(bag2,'Topic','/rtabmap/cloud_map');
bag_select3 = select(bag3,'Topic','/rtabmap/cloud_map');

%Estraggo i messaggi che interessano, ad es l'ultimo 
%pcd_array = readMessages(bag_select,'DataFormat','struct');
pcd1 = readMessages(bag_select1, round(bag_select1.NumMessages*0.99));
pcd2 = readMessages(bag_select2, round(bag_select2.NumMessages*0.99));
pcd3 = readMessages(bag_select3, round(bag_select3.NumMessages*0.99));

%Per la pointcloud:
%Converto il messaggio in array di punti
pcd1{1}.PreserveStructureOnRead = 1;
% points_xyz = readXYZ(pcd{1});


%plot
scatter3(pcd1{1}); title("Full cloud bag 1: june 19");
figure;
scatter3(pcd2{1}); title("Full cloud bag 2: short");
figure;
scatter3(pcd3{1}); title("Full cloud bag 3: aug 19");
%--------------------------------------------------------- USER CODE -----
% figure;
% plot3(dpoints(:,1), dpoints(:,2), dpoints(:,3));
% figure;
% surf(dpoints);

% dpoints = double(points_xyz);
% cpoints = readRGB(pcd{1});
% [filt, clrs] = outlier_filter_0(dpoints, cpoints, 0.8);
% figure;
% scatter3(filt(:,1), filt(:,2), filt(:,3), clrs); title("Filtered cloud"); 
% xlabel("X"); ylabel("Y"); zlabel("Z");

%--------------------------------------------------------- USER CODE -----

% save progress over a new pcd file: original and filtered point cloud
% pcobj = pointCloud(readXYZ(pcd1{1}),'Color',uint8(255*readRGB(pcd{1})));
% pcwrite(pcobj,'pcdOutput','PLYFormat','binary');