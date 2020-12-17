%% Clean up console and variables
%clc; 
close all; clear all;
addpath('classes');
addpath('utilities');
% should i plot the results
plotting = 0;
%% Load saved data
%load('cache/tempRelated')
%load('cache/kalmanIns/newDis/Re_2neigh')
%load('cache/kalmanIns/newDisold/Re_4neigh')
%nodiff/

%load('cache/paper/4neigh/berg-frobenius.mat')
%load('cache/newDis/nodiff/4neigh/set-frobenius.mat')

%load('cache/temp');
load('cache/DKF/DKF_6neigh')

%load('cache/negone');
node_ids = nm.getNodeIds();
node_names = nm.getNodeNames();

%% Plot Position Convergence
%t_history = nm.getAllMeasurementTimes();
p_errors = [];
downsample = 10;
names = {};
err_matrix = [];
sup_err_matrix =[];
infi_err_matrix=[];
mean_arr =[];

nodeIdx = 1;
%   xyz_tru = nm.nodes{ srcIdx }.getTruePosition();
xyz_array = [];
sup_array =[];
infi_array =[];
err_array =[];
rad_array =[];
for j=1:length(p_history)
    if j > length(p_history)
        break;
    end
    t = t_history(j);
    
    xyz_tru = targetLoc_history{j};
    xyz_est = p_history{j}( nodeIdx,: );
    xyz_err = norm(xyz_tru' - xyz_est);
    
    sup_est = pSupremumAll_history{j}( nodeIdx,: );
    
    
    infi_est = pInfimumAll_history{j}( nodeIdx,: );
    
    xyz_array = [xyz_array; t xyz_tru(1) xyz_tru(2)];
    err_array = [err_array; t xyz_err];
    sup_array = [sup_array; t sup_est(1) sup_est(2)];
    infi_array = [infi_array; t infi_est(1) infi_est(2)];
    rad_array =[rad_array;t 0.5*(sup_est(1)-infi_est(1)) 0.5*(sup_est(2)-infi_est(2))];
end

rad_total = [rad_array(1:length(rad_array),2);rad_array(1:length(rad_array),3)];
fprintf('distance max=%.3f, mean,std= %.3f & %.3f \n ',max(disList), mean(disList),std(disList));
fprintf('radius (mean,std)=%.3f & %.3f\n',mean(rad_total),std(rad_total));   
fprintf('Total mean = %.3f & %.3f= std \n',mean(err_array(1:length(rad_array),2)),std(err_array(1:length(rad_array),2)));


if(plotting)
    
    %   xyz_tru = nm.nodes{ srcIdx }.getTruePosition();
    xyz_array = [];
    sup_array =[];
    infi_array =[];
    err_array =[];
    rad_array =[];
    for j=1:length(p_history_plot)
        if j > length(p_history_plot)
            break;
        end
        t = t_history_plot(j);
        
        xyz_tru = targetLoc_history_plot{j};
        xyz_est = p_history_plot{j}( nodeIdx,: );
        xyz_err = norm(xyz_tru' - xyz_est);
        
        sup_est = pSupremumAll_history_plot{j}( nodeIdx,: );
        
        
        infi_est = pInfimumAll_history_plot{j}( nodeIdx,: );
        
        xyz_array = [xyz_array; t xyz_tru(1) xyz_tru(2)];
        err_array = [err_array; t xyz_err];
        sup_array = [sup_array; t sup_est(1) sup_est(2)];
        infi_array = [infi_array; t infi_est(1) infi_est(2)];
    end
    
    
    figure();
    % it is state plus one
    statenum =2 ;
    %plot(xyz_array(1:downsample:end,1) - xyz_array(1,1), xyz_array(1:downsample:end,statenum), '-o', 'Color', [0 0 1]);
    plot(xyz_array(1:downsample:end,1) - xyz_array(1,1), xyz_array(1:downsample:end,statenum), '-', 'Color', [0 0 1]);
    
    hold on
    plot(xyz_array(1:downsample:end,1) - xyz_array(1,1), sup_array(1:downsample:end,statenum), '-x', 'Color', [0 0 0]);
    plot(xyz_array(1:downsample:end,1) - xyz_array(1,1), infi_array(1:downsample:end,statenum), '-o', 'Color', [1 0 0]);
    xlabel('Time Step');
    ylabel('State x[1] (m)');
    grid on;
    xlim([0 1000]);
    % ylim([-10 11]);
    legend('true value','upper bound','lower bound')
    ax = gca;
    ax.FontSize = 16;
    set(gcf, 'Position',  [50, 50, 800, 400])
    ax = gca;
    outerpos = ax.OuterPosition;
    ti = ax.TightInset;
    left = outerpos(1) + ti(1);
    bottom = outerpos(2) + ti(2);
    ax_width = outerpos(3) - ti(1) - ti(3);
    ax_height = outerpos(4) - ti(2) - ti(4);
    ax.Position = [left bottom ax_width ax_height];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    figure
    % it is state plus one
    statenum =3 ;
    plot(xyz_array(1:downsample:end,1) - xyz_array(1,1), xyz_array(1:downsample:end,statenum), '-', 'Color', [0 0 1]);
    hold on
    plot(xyz_array(1:downsample:end,1) - xyz_array(1,1), sup_array(1:downsample:end,statenum), '-x', 'Color', [0 0 0]);
    plot(xyz_array(1:downsample:end,1) - xyz_array(1,1), infi_array(1:downsample:end,statenum), '-o', 'Color', [1 0 0]);
    xlabel('Time Step');
    ylabel('State x[2] (m)');
    grid on;
    xlim([0 1000]);
    % ylim([-10 11]);
    legend('true value','upper bound','lower bound')
    ax = gca;
    ax.FontSize = 16;
    set(gcf, 'Position',  [50, 50, 800, 400])
    ax = gca;
    outerpos = ax.OuterPosition;
    ti = ax.TightInset;
    left = outerpos(1) + ti(1);
    bottom = outerpos(2) + ti(2);
    ax_width = outerpos(3) - ti(1) - ti(3);
    ax_height = outerpos(4) - ti(2) - ti(4);
    ax.Position = [left bottom ax_width ax_height];
    
end