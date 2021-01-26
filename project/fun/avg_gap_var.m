% Stima e stampa media ed "errore massimo" (massimo gap tra i valori) in un array di punti
% v. 0.3 introdotta la varianza
function [avg, gap, vrz] = avg_gap_var(baricenters)
    avg = mean(baricenters); 
    % max/min restituiscono un vettore: ogni colonna ha max/min su quella
    gap = abs(max(baricenters) - min(baricenters)) * 100;
    % varianza (offre una stima di quanto "balliamo" intorno alla media
    vrz = var(baricenters) * 100;
    
    if size(baricenters, 1) > 1
        fprintf('Average\t\ton x: %f cm\ton y: %f cm\ton z: %f cm\n', avg(1), avg(2), avg(3));
        fprintf('Max gap\t\ton x: %f cm\ton y: %f cm\ton z: %f cm\n', gap(1), gap(2), gap(3));
        fprintf('Variance\ton x: %f cm\ton y: %f cm\ton z: %f cm\n', vrz(1), vrz(2), vrz(3));
    end
end