clear all
close all
clc

pntcloud = pcread('ply/b_190802.ply'); 
pcshow(pntcloud); title ('Original 190802 point cloud');
figure;
% pntcloud = pcread('ply/norm_190802.ply'); 
pntcloud = pcread('ply/norm_190802.ply');
pcshow(pntcloud); title('Cloud 190802 with normals');
% hold on
% x = pntcloud.Location(:,1); y = pntcloud.Location(:,2); z = pntcloud.Location(:,3);
% u = pntcloud.Normal(:,1); v = pntcloud.Normal(:,2); w = pntcloud.Normal(:,3);
% quiver3(x,y,z,u,v,w);
% hold off
% figure;
% pcshow(pntcloud); title('Structured cloud');

figure;
pntcloud = pcread('ply/detect_190802.ply'); 
pcshow(pntcloud); title('Shape detection');
figure;
pntcloud = pcread('ply/cleardetect_190802.ply'); 
pcshow(pntcloud); title('Isolated detection');

