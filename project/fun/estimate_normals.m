function [] = estimate_normals(input, output)
    fprintf('Estimating normals...\n');
    pc = pcread(input);
    pc.Normal = pcnormals(pc);
    pcwrite(pc, output, 'PLYFormat', 'ascii');
end