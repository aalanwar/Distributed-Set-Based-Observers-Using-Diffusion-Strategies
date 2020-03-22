%% Clean up console and variables
clc; close all; clear all;
addpath('classes');
addpath('utilities');

dbstop if error
%% Raw Data Log Folder
logfolder = 'logs/';
%logfolder = 'C:\Users\Alanwar\Dropbox\Pauldata\good\static';
%logfolder = 'C:\Users\Alanwar\Dropbox\Pauldata\good/ped04/';


%% Node/Network configuration
configfile = 'config/nodepositions';
%configfile = 'config/nodepositions_nesl_mobile';

%% Create Network Manager
% NetworkManager(nodeconfig, datafolder, <owr_corrections>, <twr_corrections>)
nm = NetworkManager_R(configfile, logfolder );
node_ids = nm.getNodeIds();
node_names = nm.getNodeNames();


rand('state',123)
randn('state',223)
%nm.skipTime(100);


xlength = 2;
debugEnable=0;
logfilename='log.txt';
for nidx=1:length(node_ids)
    nm.nodes{nidx}.initReach(xlength,debugEnable,logfilename);
    if nm.nodes{nidx}.isMobile() %this is Q
        nm.nodes{nidx}.setPositionCovar(0.50); % three values in x,y,z
    else
        nm.nodes{nidx}.setPositionCovar(0.002);% this is Q
    end
end


%% Process Covariances
% Process and Initial noise and state vals are determined by each node object
Q = diag([0.5 0.5]);
P = diag([0.002 0.002]);


%% Save as movie
SAVEMOVIE = false;
if SAVEMOVIE
    vidObj = VideoWriter('output/Set_fro.avi');
    vidObj.FrameRate=10;
    open(vidObj);
end

nm.setneigh_forall();
intialstate = [0;0];
nm.init_x_P_forall_reach(intialstate,P);
%% Position Visualization
% get current true and estimated positions
pTruStatic = nm.getTrueStaticPositions();
pEstAll = nm.getEstPositions();
fig = cfigure(23,23); grid on; hold on; axis equal;

htruStatic = plot3(pTruStatic(:,2),pTruStatic(:,3),pTruStatic(:,4),'^k','MarkerSize',10, 'MarkerFaceColor', 'k');
hestAll = plot3(pEstAll(:,1),pEstAll(:,2),pEstAll(:,3),'r+');
hTarget = plot3(0, 0, 0, 'sb', 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'LineWidth', 2);

hvar = zeros(nm.getNumNodes(),1);
h_reach = zeros(1,1);
varscale = 1.00;


if (debugEnable)
    logMsg(logfilename,'This is the start of the program\n*********************\n');
end


for i=1:nm.getNumNodes()
    %initialize reachability
    nm.nodes{i}.initReach(xlength,debugEnable,logfilename);
    nid = node_ids(i);
    
    % add text for each node
    if ~nm.nodes{i}.isMobile()
        xyz = nm.nodes{i}.getTruePosition();
        text( xyz(1)+3, xyz(2) + 5.0, xyz(3)-4, nm.nodes{i}.getName() );
    end  
    nidx = (i-1)*3 + 1;
end

%plot initial zonotopes for all nodes
index=1;
for i =1:nm.numnodes
  h_reach(i) = plotZono(nm.nodes{i}.x_zonotope,[1,2],'r');
end


 xlim([-90 90]);
 ylim([0 4]);
zlim([-90 90]);
%  xlim([-70 70]);
%  ylim([0 4]);
% zlim([-70 70]);
%zlim([-40 70]);
%ylim([0 4]);
%xlim([-70 20]);
%zlim([-8 8]);
xlabel('X Position (m)', 'FontSize',14);
ylabel('Y Position (m)', 'FontSize',14);
%zlabel('Z Position (m)', 'FontSize',14);
zlabel('Y Position (m)', 'FontSize',14);
htitle = title('Network Localization (t = 0.00s)','FontSize',14);
view(180,0);
%legend(herr, nm.getNodeNames());
drawnow;

%% Replay data and run EKF
% analysis stop time
t_stop = 1000;
nm.attack_nodes = zeros(1,nm.getNumNodes());
% last global time update
meas1 = nm.getNextMeasurement();
t_start = meas1.getTime();
t_last = t_start;
k = 0;

% plotting
plot_delay = 10; % sec
plot_last = t_start;
dispt_last = t_start;


meas_last = meas1;


p_history = {};
pSupremumAll_history={};
pInfimumAll_history={};
targetLoc_history= {};
t_history = [];

p_history_plot = {};
pSupremumAll_history_plot={};
pInfimumAll_history_plot={};
targetLoc_history_plot= {};
t_history_plot = [];
diffEnable =1;
numofneig = length(nm.network{1})-1;
algorithm = 'set-membership';
%set-membership
%luenberger

method = 'frobenius';
%frobenius
%svd
%diffKF
%volume
disList =[];
timepast =0;
while (t_last - t_start) < t_stop
    k = k + 1;
    
    %----
    currentTime=t_last - t_start;
    if (debugEnable)
        logMsg(logfilename,'\n**********\n\tTime=%f \t Step=%d\n*********************\n',currentTime,k);
    end
    
    % get next measurement object
    meas = nm.getNextMeasurement();
    % [meas.R_ij , meas.r_ij, meas.R_ij_orig , meas.r_ij_orig]
    
    if isempty(meas)
        k
        break;
    end
    
    idx = meas.getNodeIdx();
    walltime = meas.getTime();
    z = meas.vectorize();
    R = meas.getCovariance();
    
  
    xyGndTruth = meas.getTruePosition();
    
    meas_last = meas;
        
    % delta-time uses wallclock (desktop timestamp) for now
    dt_ref = meas.getTime() - t_last;
    t_last = walltime;
    
    
    if walltime - dispt_last > 50.00
        fprintf('Time: %.2f (%.2f%%)\n', (walltime - t_start), 100*(walltime - t_start)/t_stop);
        dispt_last = walltime;
        

        timepast =1;
        
    end
    

    % get network state vector
    %s = nm.getState();
    
    % configure process and measurement functions
    f = @(s) nm.processFcn(s);
    h = @(s) nm.measurementFcn(s, meas);

    %if( any(nm.network{srcIdx}==dstIdx) )
    %[s, P] = ekf(f, s, P, h, z, dt_ref*Q, R);
    %end
    
   
    nm.publishMeasForNeigh(meas,h);
    nm.checkEkfP1(diffEnable,f,algorithm,method,Q);
    if(diffEnable)
        nm.publishEitaForNeigh();
        nm.checkEkfP2(f,Q ,diffEnable,algorithm);
    end
    

    
    % update position estimates
    pTruStatic = nm.getTrueStaticPositions();
    pEstAll = nm.getEstPositions();
    pSupremumAll = nm.getSupremumPositions();
    pInfimumAll = nm.getInfimumPositions();
    targetLoc = meas.getTruePosition();

    if walltime - plot_last >= plot_delay
        plot_last = walltime;
        % update plot title
        tstr = sprintf('Network Localization (t = %.2fs)', (t_last - t_start));
        set(htitle, 'String', tstr);
        drawnow;
        set(hestAll,'xdata',pEstAll(:,1),'ydata', pEstAll(:,2),'zdata',pEstAll(:,3));
        for i=1:nm.numnodes
            updatePlotZono(h_reach(i),nm.nodes{i}.x_zonotope,[1,2],'r');
        end
        set(hTarget, 'XData', targetLoc(1), 'YData', 0, 'ZData', targetLoc(2));
        
    end
 
    if timepast==1
        allready=1;
        for ii=1:length(nm.nodes)
            if nm.nodes{ii}.readytotakeDis==0
                allready=0;
            end
        end
        
        if allready==1
            zonolist ={};
            for ii=1:length(nm.nodes)
                zonolist{ii}=nm.nodes{ii}.zonoforDisSave;
            end
            disList=[disList ;zonoDiff(zonolist)];
            for ii=1:length(nm.nodes)
                nm.nodes{ii}.readytotakeDis=0;
                nm.nodes{ii}.zonoforDisSave =0;
            end
            timepast=0;
        end
        
    end
        
    algdoneallready=1;
    for ii=1:length(nm.nodes)
        if nm.nodes{ii}.algdone==0
            algdoneallready=0;
        end
    end

    
    
    if algdoneallready==1
        
        for ii=1:length(nm.nodes)
            nm.nodes{ii}.algdone=0;
        end
        
        
        % append state estimate & measurement to history
        %neglect y's
        p_history = [p_history; pEstAll(:,[1 3])];
        pSupremumAll_history = [pSupremumAll_history; pSupremumAll];
        pInfimumAll_history = [pInfimumAll_history; pInfimumAll];
        targetLoc_history = [targetLoc_history ;  targetLoc];
        
        t_history = [t_history; walltime];
    end
        p_history_plot = [p_history_plot; pEstAll(:,[1 3])];
        pSupremumAll_history_plot = [pSupremumAll_history_plot; pSupremumAll];
        pInfimumAll_history_plot = [pInfimumAll_history_plot; pInfimumAll];
        targetLoc_history_plot = [targetLoc_history_plot ;  targetLoc];    
        
        t_history_plot = [t_history_plot; walltime];

    if SAVEMOVIE
        f = getframe(fig);
        writeVideo(vidObj,f);
    end

end

if SAVEMOVIE
    close(vidObj);
end
%sync_history(end,:)
%[mean(abs(sync_history(end,:))),std(abs(sync_history(end,:)))]

% % save data
%save('cache/temp', 'nm', 'k', 'p_history','targetLoc_history' ,'t_history','pSupremumAll_history','pInfimumAll_history');
if(strcmp(algorithm,'set-membership'))%set-membership
nameportion='set';
elseif (strcmp(algorithm,'luenberger'))
 nameportion='berg';
end
%with diffusion
%name = strcat('cache/linear/',num2str(numofneig),'neigh/',nameportion,'-',method);
%without diff
if diffEnable==1
name = strcat('cache/newDis/',num2str(numofneig),'neigh/',nameportion,'-',method);
else
name = strcat('cache/newDis/nodiff/',num2str(numofneig),'neigh/',nameportion,'-',method);    
end
%save(name, 'nm', 'k', 'nm', 'k', 'p_history','targetLoc_history' ,'t_history','pSupremumAll_history','pInfimumAll_history','disList','p_history_plot','targetLoc_history_plot' ,'t_history_plot','pSupremumAll_history_plot','pInfimumAll_history_plot');

save('cache/temp_nodiff_del', 'nm', 'k', 'p_history','targetLoc_history' ,'t_history','pSupremumAll_history','pInfimumAll_history','disList','p_history_plot','targetLoc_history_plot' ,'t_history_plot','pSupremumAll_history_plot','pInfimumAll_history_plot');

return;
