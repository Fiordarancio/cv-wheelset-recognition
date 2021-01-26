% Il programma divide una point cloud grande in una piu' piccola, 
% dividendola a meta' seguendo l'asse su cui essa si dispiega in
% lunghezza (in questo caso, x)
function [] = divide_ptclouds (infile, outfile1, outfile2)
    ptcloud = pcread(infile);
    loc1 = zeros(1,3); loc2 = zeros (1,3);
    col1 = zeros(1,3); col2 = zeros (1,3);
    xrange = ptcloud.XLimits(2) - ptcloud.XLimits(1);
    j1 = 1; j2 = 1;
    for i=1 : ptcloud.Count
        if ptcloud.Location(i, 1) < xrange/2
            loc1(j1, :) = ptcloud.Location(i, :);
            col1(j1, :) = ptcloud.Color(i, :);
            j1 = j1 + 1;
        else
            loc2(j2, :) = ptcloud.Location(i, :);
            col2(j2, :) = ptcloud.Color(i, :);
            j2 = j2 + 1;
        end
    end

    newptcloud1 = pointCloud(loc1, 'Color', col1);
    newptcloud2 = pointCloud(loc2, 'Color', col2);

    % assegna i colori
    for k=1 : j1-1
        newptcloud1.Color(k, :) = col1(k, :);
    end

    for k=1 : j2-1
        newptcloud2.Color(k, :) = col2(k, :);
    end

    pcwrite(newptcloud1,outfile1,'PLYFormat','ascii');
    pcwrite(newptcloud2,outfile2,'PLYFormat','ascii');
end