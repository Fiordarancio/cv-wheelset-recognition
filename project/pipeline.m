% Pipeline da seguire (ver 0.2 del 30-08-2019)
% Ogni passo, invece che con una funzione, si concretizza nel chiamare un
% eseguibile in C++. Questa scelta e' dovuta al fatto che le dipendenze di
% CGAL sono piuttosto complesse e quindi scrivere molte funzioni diverse
% sarebbe risultato inutilmente lungo. Ogni passo e' stato testato a se',
% pertanto il tempo di computazione "sprecato" con questo approccio
% consiste esclusivamente nella successiva lettura/scrittura degli stessi
% files che contengono la pointcloud processata
clear
close all
clc

msg = 'File name (freno/190613a/190613b/190802): ';
name = input(msg, 's');

% preparazione dei programmi C++ da chiamare via matlab (caricare in uno
% script modificabile a parte)
run('init_paths.m');

% i seguenti parametri dipendono dalla qualita' della nuvola (default: 2,2)
outer_iterations = input('# of outer pipeline iterations: '); % # di ripetizioni dell'intera pipeline
inner_iterations = input('# of inner iterations (outlier removal): '); % " della rimozione di outliers

% scegli se plottare o no tutti gli steps
msg = 'Do you want to plot all steps? [yes/no] ';
anw = input(msg, 's');
if strcmp(anw, 'yes') == 1
    plot_all_steps = true;
    fprintf('Plotting all outputs...\n');
else
    plot_all_steps = false;
end

% misuriamo il tempo di computazione dell'intera pipeline facendo partire
% il cronometro di Matlab...
tic

% preparazione dell'ingresso: calcolo delle normali della nuvola
input_file = [ply_ps 'b_' name ply];
output_file = [ply_ps 'c_' name ply];
estimate_normals (input_file, output_file);
if plot_all_steps == true
    plot_cloud (output_file, 'Original cloud');
end

% PIPELINE DI RICONOSCIMENTO ASSILE
% 1) cut off delle coordinate conosciute (valori parametrici salvati in
% limits.ply)
input_file = [ply_pl 'c_' name ply];
output_file = [ply_pl 'cut_' name ply];
limits = [ply_pl 'limits.ply'];
command = [cut_prog input_file ' ' output_file ' ' limits];
if system(command) ~= 0
    fprintf('Error at: step %d, iteration %d, cloud %s\n', 1, 1, name);
    return;
end
if plot_all_steps == true
    plot_cloud (output_file, 'Step 1 - cut');
end



% v 0.1 : i prossimi step si fanno due volte
for i=1 : outer_iterations
    % v 0.2 : la rimozione di outliers e' doppia
    for j=1 : inner_iterations
        % 2) remove outliers
        input_file = output_file;
        output_file = [ply_pl 'out_' name ply]; % la prima volta e' cut_, poi cleardetect_
        command = [outlier_prog input_file ' ' output_file];
        if system(command) ~= 0
            fprintf('Error at: step %d, iteration %d, cloud %s\n', 1, i, name);
            return;
        end
        if plot_all_steps == true
            plot_cloud (output_file, ['Step 2.' num2str(i) '.' num2str(j) ' - outlier removal']);
        end
    end

    % 3) detection
    input_file = output_file;
    output_file = [ply_pl 'detect_' name ply];
    command = [detect_prog input_file ' ' output_file]
    if system(command) ~= 0
        fprintf('Error at: step %d, iteration %d, cloud %s\n', 3, i, name);
        return;
    end
    if plot_all_steps == true
        plot_cloud (output_file, ['Step 3.' num2str(i) ' - shape detection']);
    end
    % 4) cleaning: rimangono solo i cilindri
    input_file = output_file;
    output_file = [ply_pl 'cleardetect_' name ply];
    command = [clear_prog input_file ' ' output_file]
    if system(command) ~= 0
        fprintf('Error at: step %d, iteration %d, cloud %s\n', 4, i, name);
        return;        
    end
    if plot_all_steps == true
        plot_cloud (output_file, ['Step 4.' num2str(i) ' - isolating cylinders']);
    end
    % output_file = '...ply/cleardetect_out_<filename>.ply'
end

% ...quanto tempo e' passato? 
comp_time = toc;
fprintf('Elapsed time is %f seconds.\n', comp_time);

% 6) find offset
output_file = [ply_pl 'cleardetect_' name ply];
ptcloud = pcread(output_file);
plot_cloud (output_file, ['Final point cloud of ' name]);
cloud_baricenter = baricenter(ptcloud)

% extra: the algorithm we use is based on Efficient RANSAC, which exploits
% a random seed to select points in each iteration. Hence, to define the
% error margin in which we are, we can save the results in workspace and
% see how it changes over N iterations

% controllare la presenza di nan, quindi salvare i risultati nel workspace
% specificando i parametri del test, ovvero:
%   - nome del file cloud usata
%   - outer and inner values
% I dati salvati sono i baricentri (per ottenere la media sul bunch di
% test) e i tempi di computazione necessari (per avere una stima del tempo
% medio dati certi valori di inner e outer)
% Importantissimo, inoltre, salvere a parte il valore medio dell'offset per
% quella nuvola
if (isnan(cloud_baricenter) ~= ones(1,3))
    % importazione degli altri baricentri per poter fare la media alla fine
    bmat_file = ['test/' name '/baricenters_' num2str(outer_iterations) num2str(inner_iterations) '.mat'];
    tmat_file = ['test/' name '/ctimes_' num2str(outer_iterations) num2str(inner_iterations) '.mat'];
    omat_file = ['test/' name '/offset_' num2str(outer_iterations) num2str(inner_iterations) '.mat'];
    if exist(bmat_file, 'file') > 0
        load(bmat_file);
        % v. 0.3 controllo dell'offset sotto una threshold
        [offset, gap_0] = avg_gap_var(baricenters);
        if size(gap_0(1), 1) == 1
            gap_0 = gap_0/100;
        end
        [offset, gap_1] = avg_gap_var(cat (1, baricenters, cloud_baricenter));
        if abs(gap_1(1) - gap_0(1)) < 15.0 % cm
            baricenters = cat (1, baricenters, cloud_baricenter);
            [offset, gap, vrz] = avg_gap_var(baricenters);
            % importazione dei tempi per la media
            if exist(tmat_file, 'file') > 0
                load(tmat_file);
                ctimes = cat (1, ctimes, comp_time);
                fprintf('Average computation time: %f sec\n', mean(ctimes));
            else
                ctimes = comp_time;
            end
            save(tmat_file, 'ctimes');
        else % else non si aggiunge questo risultato
            fprintf('\tERROR: too high error in the new result. Kept old values\n');
        end
    else
        baricenters = cloud_baricenter;
        offset = cloud_baricenter;
        % importazione dei tempi per la media
        if exist(tmat_file, 'file') > 0
            load(tmat_file);
            ctimes = cat (1, ctimes, comp_time);
            fprintf('Average computation time: %f sec\n', mean(ctimes));
        else
            ctimes = comp_time;
        end
        save(tmat_file, 'ctimes');
    end
    
     % l'offset da trasmettere sul rostopic
    offset = offset(1);
    fprintf('\nOffset on X: %f\n', offset);    
   
    % salvataggio del workspace coi dati sensibili
    save(omat_file, 'offset');
    save(bmat_file, 'baricenters');
else
    fprintf('\tERROR: no acceptabe shape has been detected on this cloud\n');
end

