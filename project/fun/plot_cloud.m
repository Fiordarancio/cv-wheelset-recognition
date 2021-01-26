% plotta una cloud salvata in un file con il titolo indicato. Funzione
% usata per visualizzare velocemente come procede l'algoritmo
function [] = plot_cloud (pcfile, pctitle)
    pc = pcread(pcfile);
    figure;
    pcshow(pc); title(pctitle); xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    fprintf('\n');
end