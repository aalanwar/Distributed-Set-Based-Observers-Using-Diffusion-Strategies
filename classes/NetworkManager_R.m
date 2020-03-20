classdef NetworkManager_R < handle
    %NETWORKMANAGER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        LIGHTSPEED = 299792458; % m/s
        %PLTSTYLES = {'-o', '-s', '-^', '-x', '-*', '--o', '--s', '--^', '--x', '--*'};
        PLTSTYLES = {'-o', '-s', '-^', '--x', '-*', '--o', '--s', '--^', '-*', '--*'};
        PLTCOLORS = [
            1 0 0
            0 0 1
            0 0 0
            0.8 0.8 0
            1 0 1
            0 0.7 0.7
            0 1 0
            0.5 0.5 0.5
            0.25 0.25 1
            1 0.25 0.25
            0.25 1 0.25
            ];
        PASS = 1;
        BLOCK = 0;
        MSGTYPE1 = 1;
        MSGTYPE2 = 2;
        MSGTYPE3 = 3;
        MEASTYPE_d = 1;
        MEASTYPE_r = 2;
        MEASTYPE_b = 3;
        MEASTYPE_B = 4;
        MEASTYPE_R = 5;
        MAXMEASUREMENTS = 1e6;
        MEASBLOCKBKOFF = 0.010;
    end
    
    properties
        %Propagation delay
        propdelay2alpha=[];
        
        %zonotopes
        x0_z;


        % node variables
        nodes = {};
        nodeinfo = {};
        numnodes = 0;
        

        % message / data variables
        dataparser = [];
        antennacorrections = [];
        twrcorrections = [];
        owrcorrections = [];
        msgidx = 1;
        measurement_history = {};
        meascnt = 0;
        
        % range bias state
        rangebiases = [];
        var_rangebias = 0.10;  % m/s
        vari_rangebias = 1.0^2; % m
        
        % message filtering (pass=1,block=0)
        filter_type1 = 0;
        filter_type2 = 0;
        filter_type3 = 0;
        filter_meas_d = 0;
        filter_meas_r = 0;
        filter_meas_b = 0;
        filter_meas_B = 0;
        filter_meas_R = 0;
        %%%%%%%%%%%%%%%%%
        attack_nodes=[];
        attack_values=[];
        attack_offsets=[];

        % min period between message types (sec)
        period_type1 = 0;
        period_type2 = 0;
        period_type3 = 0;
        
        % node blacklist
        blacklist = [];
        
        % message queues (src, dest, type)
        measQ = [];
        
        % reference node id
        refNode = [];
        
        % enable SLATS?
        enable_slats = false;
                % number of saved messages
        savemsgsM =0;
        %number of sent messages
        sentmsgsM=0;
        sentmsgsD=0;
        M_hop = false;
        measTable=[0,0,0,0];
        resetTable = 0;
        indexTable=1;
        pl_list;
        att;
        attGndTruth;
        numberofTabRows=500;


        %neighbours including myself
        %  network= { [1 2 3 ],[ 1 2 3 4 5],[ 1 2 3 4],[3 4 5],[4 5 6],[5 6 7],[6 7 8],[1 2 3 4 5 6 7 8]};
        % network= { [1 2 3],[ 1 2 3 4 5],[ 1 2 3 4 5],[3 4 5 6],[4 5 6 7],[5 6 7 8],[4 5 6 7 8],[1 2 3 4 5 6 7 8]};
        %network= {[1 2 3 4 5 ],[1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[2 3 4 5 6 7 8],[ 2 3 4 5 6 7 8],[ 2 3 4 5 6 7 8]};
        %network= { [9 1 2 ],[1 2 3 4 5 6 7 8],[2 3 4],[3 4 5],[4 5 6],[5 6 7],[6 7 8],[7 8 9],[8 9 1]};
        %  network= { [8 1 2 3],[1 2 3 4 5 6 7 8],[1 2 3 4],[2 3 4 5],[3 4 5 6],[4 5 6 7],[4 6 7 8],[6 7 8 1]};
        %fully connected
        %network= { [1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[ 1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[ 1 2 3 4 5 6 7 8]};
        
        %rm 2 connection
        %network= { [1 2 3 4 5 6 7],[1 2 3 4 5 6 7 8],[ 1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[2 3 4 5 6 7 8]};
        %network= { [1 2 3 4 5 6 7 8],[1 2 3 5 6 7 8],[ 1 2 3 4 5 6 7],[1 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8]};
        
        %1 neig
        % network= { [1 2],[2 3],[3 4],[4 5],[5 6],[6 7],[7 8],[8 1]};
        
        %2 neig
        network= { [8 1 2],[1 2 3],[2 3 4],[3 4 5],[4 5 6],[5 6 7],[6 7 8],[7 8 1]};
        
        %3 neig
        %network={[1 3 4 6],[2 4 5 8],[1 3 5 7],[1 2 4 7],[2 3 5 8],[1 6 7 8],[3 4 6 7],[2 5 6 8]};
        
        %4 nei
        %network = {[1 2 5 7 8],[2 3 6 7 1],[3 4 6 8 2],[4 5 6 8 3],[5 6 7 1 4],[6 2 3 4 5],[7 8 1 2 5],[8 1 3 4 7]};
        
        %5 nei
        %network={[1 2 3 5 7 8],[2 3 4 5 8 1],[3 4 6 8 1 2],[4 5 6 7 2 3],[5 6 7 1 2 4],[6 7 8 3 4 5],[7 8 1 4 5 6],[8 1 2 3 6 7]};
        
        %6 nei
        %                 1               2               3               4               5               6               7               8
        %network= { [8 1 2 3 4 5 6],[1 2 3 4 5 6 7],[1 2 3 4 5 7 8],[3 4 6 7 8 1 2],[3 5 6 7 8 1 2],[4 5 6 7 8 1 2],[6 7 8 2 3 4 5],[7 8 1 3 4 5 6]};
        
        %conditional msg
        %network= { [1 2 3 4 5 6 7 8 9],[1 2 3 4 5 6 7 8 9],[1 2 3 4 5 6 7 8 9],[1 2 3 4 5 6 7 8 9],[1 2 3 4 5 6 7 8 9],[1 2 3 4 5 6 7 8  9],[1 2 3 4 5 6 7 8 9],[1 2 3 4 5 6 7 8 9],[1 2 3 4 5 6 7 8 9]};
        %Gps
        %network= { [1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8],[1 2 3 4 5 6 7 8]};
        % mobile
        %network= { [1 2 3 4 5 6 7 8 9],[1 2 3 4 5 6 7 8 9],[3 1 2  4 5 6 7 8 9],[4 1 2 3  5 6 7 8 9],[5 1 2 3 4  6 7 8 9],[6 1 2 3 4 5  7 8 9],[7 1 2 3 4 5 6  8 9],[ 1 2 3 4 5 6 7 8 9],[ 1 2 3 4 5 6 7 8 9]};
        
    end
    
    methods
        % =============================================
        %                  CONSTRUCTOR
        % =============================================
        function obj = NetworkManager_R(configfile, logfolder)
            % create dataparser
            obj.dataparser = DataParserR(configfile, logfolder);
            obj.nodeinfo = obj.dataparser.getNodeInfo();
            % create node objects
            node_names = obj.nodeinfo{1};
            node_ids = obj.nodeinfo{2};
            node_dynamics = obj.nodeinfo{6};
            for i=1:length(node_ids)
                name = node_names{i};
                id = node_ids(i);
                xyz = [obj.nodeinfo{3}(i); obj.nodeinfo{4}(i); obj.nodeinfo{5}(i)];
                
                obj.nodes{i} = Node_R(id, name);
                
                obj.nodes{i}.setTruePosition(xyz);
                % is this a mobile node?
                if strcmp(node_dynamics{i}, 'mobile');
                    obj.nodes{i}.setToMobile();
                end
            end
            obj.numnodes = length(obj.nodeinfo{1});
            
            % pre-allocate measurement memory
            obj.measurement_history = cell(obj.MAXMEASUREMENTS,1);

        end
        


        % =============================================
        %                MESSAGE FILTERING
        % =============================================
        
        % queue a specific message type
        function queueMessage(obj, srcId, dstId, type)
            % add if Q empty
            if isempty(obj.measQ)
                obj.measQ = [obj.measQ; [srcId dstId type]];
                return;
            end
            % otherwise only add if not already queued
            if isempty( find( obj.measQ(:,1) == srcId & obj.measQ(:,2) == dstId & obj.measQ(:,3) == type ) )
                obj.measQ = [obj.measQ; [srcId dstId type]];
            end
        end
        
        % enable / disable specific message types
        function enableMessageType(obj, type, enable)
            switch type
                case obj.MSGTYPE1
                    obj.filter_type1 = enable;
                case obj.MSGTYPE2
                    obj.filter_type2 = enable;
                case obj.MSGTYPE3
                    obj.filter_type3 = enable;
                otherwise
                    error('Message type not recognized');
            end
        end
        
        % downsample specific message types to a given period lower bound
        function setMessageMinInterval(obj, type, interval)
            switch type
                case obj.MSGTYPE1
                    obj.period_type1 = interval;
                case obj.MSGTYPE2
                    obj.period_type2 = interval;
                case obj.MSGTYPE3
                    obj.period_type3 = interval;
                otherwise
                    error('Message type not recognized');
            end
        end
        
        % enable / disable specific message types
        function enableMeasurementType(obj, type, enable)
            switch type
                case obj.MEASTYPE_d
                    obj.filter_meas_d = enable;
                case obj.MEASTYPE_r
                    obj.filter_meas_r = enable;
                case obj.MEASTYPE_b
                    obj.filter_meas_b = enable;
                case obj.MEASTYPE_B
                    obj.filter_meas_B = enable;
                case obj.MEASTYPE_R
                    obj.filter_meas_R = enable;
                otherwise
                    error('Message type not recognized');
            end
        end
        
        % determine whether or not this measurement should be received
        function valid = filterMeasurement(obj, meas)
            srcId = meas.getSourceId();
            dstId = meas.getDestId();
            type = meas.getType();
            tlastSent = obj.nodes{ obj.getNodeIdx(srcId) }.getLastMeasTime( type );
            
            % ignore measurement if src or dst are blacklisted
            if ~isempty( find(obj.blacklist == srcId, 1) ) || ~isempty( find(obj.blacklist == dstId, 1) )
                valid = obj.BLOCK;
                return;
            end
            
            % set allowed measurement types % to know for vectorizing
            if obj.filter_meas_d
                meas.allowMeasType_d();
            end
            if obj.filter_meas_r
                meas.allowMeasType_r();
            end
            if obj.filter_meas_R
                meas.allowMeasType_R();
            end
            
            % next, pass immediately if measurement type has been requested
            if ~isempty( obj.measQ )
                qIdx = find(obj.measQ(:,1) ==  srcId & obj.measQ(:,2) == dstId);
                if ~isempty(qIdx)
                    % force this message to be the right type
                    meas.type = obj.measQ(qIdx,3);
                    meas.queued = 1;
                    % a message of this type has been requested, pass it and remove
                    % from the message queue
                    
                    % block if the range or time is obviously incorrect
                    if meas.r_ij_orig < -1 || meas.r_ij_orig > 20 || meas.R_ij_orig < -1 || meas.R_ij_orig > 20
                        valid = obj.BLOCK;
                        obj.measQ(qIdx,:) = [];
                        return;
                    end
                    valid = obj.PASS;
                    obj.measQ(qIdx,:) = [];
                    return;
                end
            end
            
            % block if no measurements supported
            if ~(obj.filter_meas_d || obj.filter_meas_r || obj.filter_meas_R)
                valid = obj.BLOCK;
                return;
            end
            
            %block if the range or time is obviously incorrect
            if meas.r_ij_orig < -1 || meas.r_ij_orig > 20 || meas.R_ij_orig < -1 || meas.R_ij_orig > 20
                valid = obj.BLOCK;
                return;
            end
            
            
            % pass as a type 3 message if they're allowed and it's been
            % long enough since the last type 3 message
            if obj.filter_type3 == obj.PASS
                
                meas.setType(obj.MSGTYPE3);
                tlastSent = obj.nodes{ obj.getNodeIdx(srcId) }.getLastMeasTime( obj.MSGTYPE3 );
                dt = meas.getTime() - tlastSent;
                if dt >= obj.period_type3 || dt < obj.MEASBLOCKBKOFF
                    valid = obj.PASS;
                    obj.nodes{ obj.getNodeIdx(srcId) }.setLastMeasTime( obj.MSGTYPE3, meas.getTime() );
                    return;
                end
            end
            % otherwise make this a type 2 message and pass it if they're
            % allowed and if it's been long enough since the last type 2.
            if obj.filter_type2 == obj.PASS
                % downgrade this to a type 2 message
                meas.setType(obj.MSGTYPE2);
                tlastSent = obj.nodes{ obj.getNodeIdx(srcId) }.getLastMeasTime( obj.MSGTYPE2 );
                dt = meas.getTime() - tlastSent;
                if dt >= obj.period_type2 || dt < obj.MEASBLOCKBKOFF
                    valid = obj.PASS;
                    obj.nodes{ obj.getNodeIdx(srcId) }.setLastMeasTime( obj.MSGTYPE2, meas.getTime() );
                    return;
                end
            end
            % finally, we can at the very least make this a type 1 message
            % if the other 2 messages are blocked for this time period.
            % Disallow this too, though, if not enough time has elapsed
            % since the last type 1.
            if obj.filter_type1 == obj.PASS
                % downgrade this to a type 1 message
                meas.setType(obj.MSGTYPE1);
                tlastSent = obj.nodes{ obj.getNodeIdx(srcId) }.getLastMeasTime( obj.MSGTYPE1 );
                dt = meas.getTime() - tlastSent;
                if dt >= obj.period_type1 || dt < obj.MEASBLOCKBKOFF
                    valid = obj.PASS;
                    obj.nodes{ obj.getNodeIdx(srcId) }.setLastMeasTime( obj.MSGTYPE1, meas.getTime() );
                    return;
                end
            end
            
            % otherwise, block it
            valid = obj.BLOCK;
        end
        
        % =============================================
        %                  PLOTTING
        % =============================================
        
        % plot styles and colors
        function style = getPlotStyle(obj, nodeId)
            nidx = obj.dataparser.getNodeIdx(nodeId);
            style = obj.PLTSTYLES{nidx};
        end
        function c = getPlotColor(obj, nodeId)
            nidx = obj.dataparser.getNodeIdx(nodeId);
            c = obj.PLTCOLORS(nidx,:);
        end
        
        % get number of nodes
        function n = getNumNodes(obj)
            n = obj.numnodes;
        end
        
        % get number of static nodes
        function n = getNumStaticNodes(obj)
            n = 0;
            for i=1:obj.getNumNodes()
                dynamics = obj.nodeinfo{6}{i};
                if strcmp(dynamics, 'static')
                    n = n + 1;
                end
            end
        end
        
        % get number of mobile nodes
        function n = getNumMobileNodes(obj)
            n = obj.getNumNodes() - obj.getNumStaticNodes();
        end
        
        % =============================================
        %                  NODE INFO
        % =============================================
        
        % get node ascii names
        function n = getNodeNames(obj)
            n = obj.nodeinfo{1};
        end
        
        % get numeric node is: 0, 1, ...
        function id = getNodeIds(obj)
            id = obj.nodeinfo{2};
        end
        
        % get the index of a specific node
        function idx = getNodeIdx(obj, nodeid)
            idx = obj.dataparser.getNodeIdx( nodeid );
        end
        
        % blacklist a certain node (don't use it)
        function blacklistNode(obj, nodeid)
            idx = obj.getNodeIdx( nodeid );
            nodeids = obj.getNodeIds();
            id = nodeids(idx);
            obj.blacklist = [obj.blacklist; id];
        end
        
        % =============================================
        %               STATE ACCESSORS
        % =============================================
        
        % get network state
        function s = getState(obj)
            s = [];
            for i=1:length(obj.nodes)
                s = [s; obj.nodes{i}.getState()];
            end
        end
        % get network state
        function s = getState_gps(obj)
            s = [];
            for i=1:length(obj.nodes)
                s = [s; obj.nodes{i}.getState_gps()];
            end
        end

        % get the vectorized range biases
        function sb = getRangeBiasVec(obj)
            sb = [];
            for i=1:obj.getNumNodes()
                for j=(i+1):obj.getNumNodes()
                    sb = [sb; obj.rangebiases(i,j)];
                end
            end
        end
        
        % range bias vector to matrix
        function SB = rangeBiasVec2Mat(obj, sb)
            SB = [];
            idx = 0;
            for i=1:obj.getNumNodes()
                for j=(i+1):obj.getNumNodes()
                    idx = idx + 1;
                    SB(i,j) = sb(idx);
                    SB(j,i) = sb(idx);
                end
            end
        end
        
        % set the vectorized range biases
        function setRangeBiasState(obj, sb)
            obj.rangebiases = obj.rangeBiasVec2Mat(sb);
        end
        
        % set network state
        function setState(obj, s)
            idx = 1;
            for i=1:length(obj.nodes)
                stateSize = length(obj.nodes{i}.getState());
                si = s(idx:(idx+stateSize-1));
                obj.nodes{i}.setState(si);
                idx = idx + stateSize;
            end
            
        end

         %every node participate with its state only
        function s=getAndFixStateConc(obj)
            idx = 1;
            s = [];
            for i=1:length(obj.nodes)
                stateSize = length(obj.nodes{i}.getState());
                si = obj.nodes{i}.x(idx:(idx+stateSize-1));
                obj.nodes{i}.setState(si);
                s = [ s ; si];
                idx = idx + stateSize;
            end
            
        end       

        % get estimated node positions for only static nodes
        function P = getEstimatedStaticPositions(obj)
            P = [];
            for i=1:length(obj.nodes)
                if ~obj.nodes{i}.isMobile()
                    if (size(obj.nodes{i}.getStatePosition()) == [ 3 1])
                        P = [P; obj.nodes{i}.getId() obj.nodes{i}.getStatePosition()'];
                    else
                        P = [  P; obj.nodes{i}.getId() obj.nodes{i}.getStatePosition()];
                    end
                end
            end
        end
        
        function P = getTransformedPositions_gps(obj)
            P = [];
            %stupid
            
            for i=1:length(obj.nodes)
                if ~obj.nodes{i}.isMobile()
                    if size(obj.nodes{i}.getStatePosition()',1)==1
                        P = [P; obj.nodes{i}.getId() obj.nodes{i}.getStatePosition()'];
                    else
                        P = [P; obj.nodes{i}.getId() obj.nodes{i}.getStatePosition()];
                    end
                end
            end
        end
        % get estimated node positions for all nodes
        function P = getEstimatedPositions(obj)
            P = [];
            for i=1:length(obj.nodes)
                if size(obj.nodes{i}.getStatePosition()',1)==1
                    P = [P; obj.nodes{i}.getStatePosition()'];
                else
                    P = [P; obj.nodes{i}.getStatePosition()];
                end
            end
        end
        
        % get true node positions
        function P = getTrueStaticPositions(obj)
            P = [];
            for i=1:length(obj.nodes)
                if ~obj.nodes{i}.isMobile()
                    P = [P; obj.nodes{i}.getId() obj.nodes{i}.getTruePosition()'];
                end
            end
        end
        

       function [est] = getEstPositions(obj)            
            for i=1:obj.numnodes
                % put
                est(i,[1 3])=obj.nodes{i}.x_zonotope.center;
                est(i,2) =0;
            end
       end
       
       function [sup] = getSupremumPositions(obj)
           for i=1:obj.numnodes
               % put
               sup(i,[1 2])=supremum(interval(obj.nodes{i}.x_zonotope));
           end
       end
       
       function [infi] = getInfimumPositions(obj)
           for i=1:obj.numnodes
               % put
               infi(i,[1 2])=infimum(interval(obj.nodes{i}.x_zonotope));
           end
       end 
       
       function [P, zonotopePlot] = getTransformedPositionsZonotopes(obj)
           index=1;
           for i=1:obj.numnodes
               centers(index:index+2,1)=obj.nodes{i}.x_zonotope.Z(index:index+2,1);
               index = index+3;
           end
            centerXYZ=[];
            for i=1:3:length(centers)
               centerXYZ = [centerXYZ ;centers(i:i+2)'];
            end            
            truStatic = obj.getTrueStaticPositions();
            estStatic = obj.getEstimatedStaticPositions();
            % procrustes ignoring first column (node IDs)
            [~,~,Transform] = procrustes(truStatic(:,2:end), estStatic(:,2:end), 'Scaling', false);
            % transform all points
            estAll = obj.getEstimatedPositions();
            ofst = repmat(Transform.c(1,:), obj.getNumNodes(), 1);
            P = Transform.b*estAll*Transform.T + ofst;
            %transform center of zonotopes
            centerXYZTransform = Transform.b*centerXYZ*Transform.T + ofst;
            longcenter=[];
            index =1;
            for i=1:3:length(centers)
               longcenter = [longcenter ;centerXYZTransform(index,1:3)'];
               index = index+1;
            end  
            zonotopePlot = zonotope([longcenter, obj.nodes{2}.x_zonotope.Z(:,2:end)]);
        end
        
        % get estimated positions transformed to match true as closely as
        % possible (procrustes)
        function P = getTransformedPositionsDis(obj)
            truStatic = obj.getTrueStaticPositions();
            truStatic(:,1)= truStatic(:,1) +1; %index instead of id
            P_i = [];
            for i=1:length(obj.network)
                %estStatic = obj.getEstimatedStaticPositions();
                index =1;
                estStatic_i =[];
                for j=sort(obj.network{i})
                    estStatic_i(index,1) = j;
                    estStatic_i(index,2:4) = obj.nodes{i}.pl_list{j} ;
                    if (j ==i) %it is me, save index
                       saveindex = index; 
                    end
                    index = index +1;
                end
                truStatic_i = truStatic(sort(obj.network{i}),:);
                % procrustes ignoring first column (node IDs)
                [~,~,Transform_i] = procrustes(truStatic_i(:,2:end), estStatic_i(:,2:end), 'Scaling', false);
                % transform all points
                %estAll = obj.getEstimatedPositions();
                estAll_i = estStatic_i(:,2:end); % will not work for mobile node
                ofst = repmat(Transform_i.c(1,:), length(obj.network{i}), 1);
                P_temp = Transform_i.b*estAll_i*Transform_i.T + ofst;
                P_i = [ P_i ; [i  P_temp(saveindex,:) ]];
            end
            P = sortrows(P_i);
        end

        
        function P = getTransformedPositionsDis_DKAL(obj)
            truStatic = obj.getTrueStaticPositions();
            truStatic(:,1)= truStatic(:,1) +1; %index instead of id
            P_i = [];
            for i=1:length(obj.network)
                %estStatic = obj.getEstimatedStaticPositions();
                index =1;
                estStatic_i =[];
                for j=sort(obj.network{i})
                    estStatic_i(index,1) = j;
                    %estStatic_i(index,2:4) = obj.nodes{i}.pl_list{j} ;
                    
                    stateSize = length(obj.nodes{1}.getState());
                    start_i = (j-1)*stateSize + 1;
                    %start_j = (dstIdx-1)*stateSize + 1;
                    % extract the needed state info
                    
                    state = obj.nodes{j}.x ;%obj.nodes{i}.getState();
                    pj = state(start_i:(start_i+2));
                    estStatic_i(index,2:4) = pj';
                    if (j ==i) %it is me, save index
                        saveindex = index;
                    end
                    index = index +1;
                end
                truStatic_i = truStatic(sort(obj.network{i}),:);
                % procrustes ignoring first column (node IDs)
                [~,~,Transform_i] = procrustes(truStatic_i(:,2:end), estStatic_i(:,2:end), 'Scaling', false);
                % transform all points
                %estAll = obj.getEstimatedPositions();
                estAll_i = estStatic_i(:,2:end); % will not work for mobile node
                ofst = repmat(Transform_i.c(1,:), length(obj.network{i}), 1);
                P_temp = Transform_i.b*estAll_i*Transform_i.T + ofst;
                P_i = [ P_i ; [i  P_temp(saveindex,:) ]];
            end
            P_temp = sortrows(P_i);
            P = P_temp(:,2:end);%rm index
        end
        
        function P = getTransformedPositionsDis_DKAL_v2(obj)
            truStatic = obj.getTrueStaticPositions();
            truStatic(:,1)= truStatic(:,1) +1; %index instead of id
            P_i = [];
            for i=1:length(obj.network)
                %estStatic = obj.getEstimatedStaticPositions();
                index =1;
                estStatic_i =[];
                for j=1:8%sort(obj.network{i})
                    estStatic_i(index,1) = j;
                    %estStatic_i(index,2:4) = obj.nodes{i}.pl_list{j} ;
                    
                    stateSize = length(obj.nodes{1}.getState());
                    start_i = (j-1)*stateSize + 1;
                    %start_j = (dstIdx-1)*stateSize + 1;
                    % extract the needed state info
                    
                    state = obj.nodes{j}.x ;%obj.nodes{i}.getState();
                    pj = state(start_i:(start_i+2));
                    estStatic_i(index,2:4) = pj';
                    if (j ==i) %it is me, save index
                        saveindex = index;
                    end
                    index = index +1;
                end
                truStatic_i = truStatic;%(sort(obj.network{i}),:);
                % procrustes ignoring first column (node IDs)
                [~,~,Transform_i] = procrustes(truStatic_i(:,2:end), estStatic_i(:,2:end), 'Scaling', false);
                % transform all points
                %estAll = obj.getEstimatedPositions();
                estAll_i = estStatic_i(:,2:end); % will not work for mobile node
                %ofst = repmat(Transform_i.c(1,:), length(obj.network{i}), 1);
                ofst = repmat(Transform_i.c(1,:), 8, 1);
                P_temp = Transform_i.b*estAll_i*Transform_i.T + ofst;
                P_i = [ P_i ; [i  P_temp(saveindex,:) ]];
            end
            P_temp = sortrows(P_i);
            P = P_temp(:,2:end);%rm index
        end
        
        
        % set reference node (for EKF)
        function setReferenceNode(obj, nodeId)
            nidx = obj.dataparser.getNodeIdx(nodeId);
            obj.nodes{nidx}.setAsReference();
            obj.refNode = obj.nodes{nidx}.getId();
        end
        
        % =============================================
        %               PROCESS & MEASUREMENT
        % =============================================
        
        % get process variance
        function P = getProcessVar(obj)
            p = [];
            for i=1:length(obj.nodes)
                p = [p; obj.nodes{i}.getProcessVar()];
            end
            P = diag(p);
        end

        % get process variance
        function P = getEitaProcessVar(obj)
            p = [];
            for i=1:length(obj.nodes)
                p = [p; obj.nodes{i}.getEitaProcessVar()];
            end
            P = diag(p);
        end
        
        function P = getEitaMeasVar(obj)
            p = [];
            for i=1:length(obj.nodes)
                p = [p; obj.nodes{i}.getEitaMeasVar()];
            end
            P = diag(p);
        end
        
        
        % get process variance
        function P = getEitaProcessVarImpState(obj)
            p = [];
            for i=1:length(obj.nodes)
                p = [p; obj.nodes{i}.getEitaProcessVarImpState()];
            end
            P = diag(p);
        end
        
        function P = getEitaMeasVarImpState(obj)
            p = [];
            for i=1:length(obj.nodes)
                p = [p; obj.nodes{i}.getEitaMeasVarImpState()];
            end
            P = diag(p);
        end
        
        % get initial variance
        function P = getInitialVar(obj)
            p = [];
            for i=1:length(obj.nodes)
                p = [p; obj.nodes{i}.getInitialVar()];
            end
            P = diag(p);
        end
        

        % state process function
        function snew = processFcn(obj, s)
            F= [0.992 -0.1247; 0.1247 0.992];
            snew = F*s;       
        end
        
        function attacks= getAttack(obj)
            attacks=[];
            for i=1:length(obj.nodes)
                attacks = [attacks ; obj.nodes{i}.getStateAttack() ];
            end
        end
        
       function attacks= getAttackOffset(obj)
            attacks=[];
            for i=1:length(obj.nodes)
                attacks = [attacks ; obj.nodes{i}.getStateAttackOffset() ];
            end
        end
        % state measurement function

        function y = measurementFcn(obj, s, meas)
            y=meas.getMeasFunc()*s;
        end 
        function y = measurementFcn2(obj, s, meas)
            type = meas.getType();
            
            srcIdx = obj.getNodeIdx(meas.getSourceId());
            dstIdx = obj.getNodeIdx(meas.getDestId());
            
            % find state indices for nodes with index i and j
            stateSize = length(obj.nodes{1}.getState());
            start_i = (srcIdx-1)*stateSize + 1;
            start_j = (dstIdx-1)*stateSize + 1;
            
            % extract the needed state info
            pi = s(start_i:(start_i+2));
            pj = s(start_j:(start_j+2));
            oi = s(start_i+3);
            oj = s(start_j+3);
            bi = s(start_i+4);
            bj = s(start_j+4);
            attj=s(start_j+5);
            attOffj=s(start_j+6);
            %attOffi=s(start_i+6);
            % predicted measurement y
            % format: y = [d_ij; r_ij; B_ij; R_ij];
            y = [];
            
            c = obj.LIGHTSPEED;
            
            % d_ij = range/speed_of_light + offset
            if obj.filter_meas_d
                if obj.enable_slats
                  % y = [y; ( sqrt(sum((pj-pi).^2))  )/c + (oj - oi)+attOffj];
                  % y = [y;  (oj - oi)+attOffj]; 
                   % y = [y; ( sqrt(sum((pj-pi).^2)) + 2.0 )/c + (oj - oi)];
                else
                 %   y = [y; ( sqrt(sum((pj-pi).^2))  )/c + (oj - oi)+attOffj];
                    y = [y; (oj - oi)+attOffj];
                   % y = [y; (oj - oi)+attOffj];
                   % y = [y; (oj - oi)];
                end
            end
            
            % type 1 only has offset
            if type == 1
                return;
            end
            
            % r_ij = range + t_error
            if obj.filter_meas_r
                if obj.enable_slats
                  %  y = [y; sqrt(sum((pj-pi).^2))- atti - attj + (c*meas.T_rsp1/2)*(bj - bi)*1e-9];
                    y = [y; sqrt(sum((pj-pi).^2))+ attj  + (c*meas.T_rsp1/2)*(bj - bi)*1e-9];
                else
                  % y = [y; sqrt(sum((pj-pi).^2))- atti - attj];
                  meanTrsp =0.001001598446731;
                  minTrsp = 0.001001594582590;
                  maxTrsp = 0.001001602564088;
                   y = [y; (1+bi)*sqrt(sum((pj-pi).^2))+ attj  ];% + (c*meanTrsp/2)*(bj - bi)*1e-9 ];
                end
            end
            
            % type 2 only has offset and noisy range
            if type == 2
                return;
            end
            
            % R_ij = range + t_error
            if obj.filter_meas_R
                if obj.enable_slats
                    err = c*( 1e-9*(bi-bj)*(meas.T_rnd0*meas.T_rnd1 - meas.T_rsp0*meas.T_rsp1 ))/( (1+ 1e-9*(bi-bj))*meas.T_rnd0 + meas.T_rnd1 + meas.T_rsp0 + (1+ 1e-9*(bi-bj))*meas.T_rsp1 );
                   % y = [y; sqrt(sum((pj-pi).^2))- atti - attj + err ];
                    y = [y; sqrt(sum((pj-pi).^2))+ attj + err ];
                else
                  %  y = [y; sqrt(sum((pj-pi).^2))- atti - attj];
                  y = [y; (1+bi)*sqrt(sum((pj-pi).^2))+ attj ];
                end
            end
        end
        % state measurement function
  
function y = measurementFcnUnsec2(obj, s, meas)
            type = meas.getType();
            
            srcIdx = obj.getNodeIdx(meas.getSourceId());
            dstIdx = obj.getNodeIdx(meas.getDestId());
            
            % find state indices for nodes with index i and j
            stateSize = 5;
            start_i = (srcIdx-1)*stateSize + 1;
            start_j = (dstIdx-1)*stateSize + 1;
            
            % extract the needed state info
            pi = s(start_i:(start_i+2));
            pj = s(start_j:(start_j+2));
            oi = s(start_i+3);
            oj = s(start_j+3);
            bi = s(start_i+4);
            bj = s(start_j+4);
            
            % predicted measurement y
            % format: y = [d_ij; r_ij; B_ij; R_ij];
            y = [];
            
            c = obj.LIGHTSPEED;
            
            % d_ij = range/speed_of_light + offset
            if obj.filter_meas_d
                if obj.enable_slats
                   % y = [y; ( sqrt(sum((pj-pi).^2)) + 2.0 )/c + (oj - oi)];
                    y = [y;  (oj - oi)];
                else
                    y = [y; (oj - oi)];
                end
            end
            
            % type 1 only has offset
            if type == 1
                return;
            end
            
            % r_ij = range + t_error
            if obj.filter_meas_r
                if obj.enable_slats
                    y = [y;(1+bi)* sqrt(sum((pj-pi).^2)) + (c*meas.T_rsp1/2)*(bj - bi)*1e-9];
                else
                    y = [y; (1+bi)* sqrt(sum((pj-pi).^2))];
                end
            end
            
            % type 2 only has offset and noisy range
            if type == 2
                return;
            end
            
            % R_ij = range + t_error
            if obj.filter_meas_R
                if obj.enable_slats
                    err = c*( 1e-9*(bi-bj)*(meas.T_rnd0*meas.T_rnd1 - meas.T_rsp0*meas.T_rsp1 ))/( (1+ 1e-9*(bi-bj))*meas.T_rnd0 + meas.T_rnd1 + meas.T_rsp0 + (1+ 1e-9*(bi-bj))*meas.T_rsp1 );
                    y = [y; (1+bi)*sqrt(sum((pj-pi).^2)) + err ];
                else
                    y = [y; (1+bi)*sqrt(sum((pj-pi).^2))];
                end
            end
        end         

        
        % request the next (filtered) measurement
        function meas = getNextMeasurement(obj)
            if obj.msgidx < obj.dataparser.getNumMeasurements()
                % get new tentative meas
                raw = obj.dataparser.getMeasurement(obj.msgidx);
                obj.msgidx = obj.msgidx + 1;
                meas = Measurement_R(raw);
                srcIndex = meas.nodeIndex;
                
                if (obj.attack_nodes(srcIndex) ==1 )
                    %positive total value
                    meas.R_ij=abs(meas.R_ij+obj.attack_values(srcIndex));
                    meas.attackValue = obj.attack_values(srcIndex);
                end
            else
                % no more measurements
                meas = [];
            end
            
        end
        
        function killLastMeasurement(obj)
            obj.meascnt = obj.meascnt - 1;
        end
        
        % reset measurement index
        function resetMeasurements(obj)
            obj.msgidx = 1;
            obj.measurement_history = cell(obj.MAXMEASUREMENTS,1);
            obj.meascnt = 0;
        end
        
        % =============================================
        %                  BOOTSTRAPPING
        % =============================================
        
        % skip some time
        function skipTime(obj, tskip)
            % enable type 3 messages for this
            type3Enabled = obj.filter_type3;
            typeREnabled = obj.filter_meas_R;
            obj.enableMessageType(obj.MSGTYPE3, true);
            obj.enableMeasurementType(obj.MEASTYPE_R, true);
            
            % get the first measurement
            meas1 = obj.getNextMeasurement();
            t1 = meas1.getTime();
            t_now = t1;
            while t_now - t1 < tskip
                meas = obj.getNextMeasurement();
                if isempty(meas)
                    break;
                end
                t_now = meas.getTime();
            end
            
            % reset the measurement history but not the dp index
            obj.measurement_history = cell(obj.MAXMEASUREMENTS,1);
            obj.meascnt = 0;
            
            % set type 3 filtering to whatever it was before bootstrap
            obj.enableMessageType(obj.MSGTYPE3, type3Enabled);
            obj.enableMeasurementType(obj.MEASTYPE_R, typeREnabled);
        end
        
        % bootstrap node positions using ranges averaged over a duration
        function bootstrapNodeStates(obj, T_boot)
            % enable type 3 messages for this
            type3Enabled = obj.filter_type3;
            typeREnabled = obj.filter_meas_R;
            obj.enableMessageType(obj.MSGTYPE3, true);
            obj.enableMeasurementType(obj.MEASTYPE_R, true);
            
            % get range estimates (distance matrix)
            R_sum = zeros( obj.getNumNodes(), obj.getNumNodes() );
            % get offset estimates
            D_sum = zeros( obj.getNumNodes(), 1 );
            % distance measurement counts
            D_cnt = zeros( obj.getNumNodes(), obj.getNumNodes() );
            % timing measurements
            d_ij_arrays = cell( obj.getNumNodes(), 1 );
            d_ij_times  = cell( obj.getNumNodes(), 1 );
            % get the first measurement
            meas1 = obj.getNextMeasurement();
            t1 = meas1.getTime();
            t_now = t1;
            while t_now - t1 < T_boot
                meas = obj.getNextMeasurement();
                if isempty(meas)
                    break;
                end
                t_now = meas.getTime();
                srcIdx = obj.getNodeIdx( meas.getSourceId() );
                dstIdx = obj.getNodeIdx( meas.getDestId() );
                
                R_sum(srcIdx,dstIdx) = R_sum(srcIdx,dstIdx) + meas.R_ij;
                R_sum(dstIdx,srcIdx) = R_sum(dstIdx,srcIdx) + meas.R_ij;
                D_cnt(srcIdx,dstIdx) = D_cnt(srcIdx,dstIdx) + 1;
                D_cnt(dstIdx,srcIdx) = D_cnt(dstIdx,srcIdx) + 1;
                
                % look for messages from the reference node to bootstrap
                % clock offset and bias
                if ~isempty(obj.refNode) && meas.getSourceId() == obj.refNode
                    % append d_ij and time
                    d_ij_arrays{dstIdx} = [d_ij_arrays{dstIdx} meas.d_ij];
                    d_ij_times{dstIdx} = [d_ij_times{dstIdx} meas.getTime()];
                end
            end
            
            % average range estimates
            D = zeros( obj.getNumNodes(), obj.getNumNodes() );
            W = ones( obj.getNumNodes(), obj.getNumNodes() );
            for i=1:obj.getNumNodes()
                for j=1:obj.getNumNodes()
                    % ignore diagonals
                    if i == j
                        continue;
                    end
                    % assign default or average measurements
                    if D_cnt(i,j) == 0
                        % if we have no measurement, assume it's a bit far
                        % away and give it very low weight
                        D(i,j) = 10;
                        W(i,j) = 0.01;
                    else
                        D(i,j) = R_sum(i,j) / D_cnt(i,j);
                    end
                end
            end
            
            % use MDS to get an estimate of the node positions
            Y = mdscale(D, 3, 'Weights', W);
            
            % assign the estimated positions to each node
            for i=1:obj.getNumNodes()
                obj.nodes{i}.setStatePosition( Y(i,:) );
            end
            
            % perform linear regression over d_ij and time to get estimated
            % clock offset and bias w.r.t. reference node
            for i=1:size(d_ij_arrays,1)
                if length(d_ij_arrays{i}) >= 3
                    % wall time = x
                    x = d_ij_times{i} - d_ij_times{i}(1);
                    % measured d_ij = y
                    y = d_ij_arrays{i};
                    p = polyfit(x, y, 1);
                    est_bias = p(1);
                    est_ofst = polyval(p, x(end));
                    obj.nodes{i}.setStateClockOfst(est_ofst);
                    obj.nodes{i}.setStateClockBias(est_bias*1e9);
                    fprintf('BOOTSTRAP: node %d: offset= %.6f ms, bias = %d ppb\n', obj.nodes{i}.getId(), est_ofst*1e3, est_bias*1e9);
                elseif obj.nodes{i}.getId() ~= obj.refNode
                    fprintf('WARN: No timing bootstrap for %d-->%d (%d measurements)\n', obj.refNode, obj.nodes{i}.getId(), length(d_ij_arrays{i}));
                end
            end
            
            % reset the measurement history but not the dp index
            obj.measurement_history = cell(obj.MAXMEASUREMENTS,1);
            obj.meascnt = 0;
            
            % set type 3 filtering to whatever it was before bootstrap
            obj.enableMessageType(obj.MSGTYPE3, type3Enabled);
            obj.enableMeasurementType(obj.MEASTYPE_R, typeREnabled);
        end
        
        % set all static nodes to their true positions
        function setStaticNodesToTruePosition(obj)
            for i=1:length(obj.nodes)
                if ~obj.nodes{i}.isMobile()
                    obj.nodes{i}.setPositionToTrue();
                end
            end
        end
        
        % =============================================
        %              MEASUREMENT HISTORY
        % =============================================
        
        % get pairwise TWR range measurements
        function r = getMeasurementTWR(obj, srcId, dstId)
            r = [];
            for i=1:obj.meascnt
                meas = obj.measurement_history{i};
                if meas.getSourceId() == srcId && meas.getDestId() == dstId
                    r = [r; meas.R_ij];
                end
            end
        end
        
        % get pairwise OWR range measurements
        function r = getMeasurementOWR(obj, srcId, dstId)
            r = [];
            for i=1:obj.meascnt
                meas = obj.measurement_history{i};
                if meas.getSourceId() == srcId && meas.getDestId() == dstId
                    r = [r; meas.r_ij];
                end
            end
        end
        
        % get pairwise offset measurements
        function [o,t] = getMeasurementOffsets(obj, srcId, dstId)
            o = [];
            for i=1:obj.meascnt
                meas = obj.measurement_history{i};
                if meas.getSourceId() == srcId && meas.getDestId() == dstId
                    o = [o; meas.d_ij];
                end
            end
        end
        
        % get all measurements between two devices
        function m = getMeasurements(obj, srcId, dstId)
            m = [];
            for i=1:obj.meascnt
                meas = obj.measurement_history{i};
                if meas.getSourceId() == srcId && meas.getDestId() == dstId
                    m = [m; meas];
                end
            end
        end
        
        % get pairwise measurement times
        function t = getMeasurementTimes(obj, srcId, dstId)
            t = [];
            for i=1:obj.meascnt
                meas = obj.measurement_history{i};
                if meas.getSourceId() == srcId && meas.getDestId() == dstId
                    t = [t; meas.getTime()];
                end
            end
        end
        
        % get all measurement times
        function t = getAllMeasurementTimes(obj)
            t = zeros(1,obj.meascnt);
            for i=1:obj.meascnt
                meas = obj.measurement_history{i};
                t(i) = meas.getTime();
            end
        end
        
        % get allan deviation between two nodes
        function [ad,tau] = getAllanDev(obj, srcId, dstId, tau)
            % get non-zero aligned data
            data = obj.dataparser.aligned_logs;
            data = data(data(:,2) == srcId & data(:,3) == dstId, :);
            % get tx and rx
            tx = data(:,5);
            rx = data(:,6);
            % tx time (ignore 1st b/c y is diffed)
            x = tx(2:end);
            % rx dFreq in seconds
            y = (diff(rx) ./ diff(tx) - 1);
            [ad,~,~,tau] = allan(struct('freq',y,'time',x),tau,'',0);
        end
        
        % get all time offsets between two nodes
        function [o,t] = getAllOffsets(obj, srcId, dstId)
            data = obj.dataparser.aligned_logs;
            idxs = find(data(:,2) == srcId & data(:,3) == dstId);
            t = data(idxs,1);
            o = data(idxs,10) - data(idxs,9);
        end
        
        % get all time biases between two nodes
        function [b,t] = getAllBiases(obj, srcId, dstId)
            data = obj.dataparser.aligned_logs;
            idxs = find(data(:,2) == srcId & data(:,3) == dstId);
            t = data(idxs,1);
            o = data(idxs,10) - data(idxs,9);
            b = diff(o)./diff(t);
            t = t(2:end);
        end
        
        % =============================================
        %              POSITION ACCESSORS
        % =============================================
        function xyz = getTruePosition(obj, nodeId, tarray)
            xyz = [];
            for i=1:length(tarray)
                t = tarray(i);
                idx = obj.getNodeIdx( nodeId );
                if obj.nodes{idx}.isMobile()
                    rbid = obj.nodes{idx}.getRigidBodyId();
                    p = obj.dataparser.getMocapPos( rbid, t );
                else
                    p = obj.nodes{ idx }.getTruePosition()';
                end
                xyz = [xyz; p];
            end
        end
        
        function setRigidBodyId(obj, nodeId, rbId)
            nodeIdx = obj.getNodeIdx( nodeId );
            obj.nodes{nodeIdx}.setRigidBodyId( rbId );
        end
        
        function setSizebuffer_forall(obj,size_R)
            for i=1:length(obj.nodes)
                obj.nodes{i}.setSizebuffer(size_R);
            end
        end
        
        function init_x_P_forall(obj,x,P,attackedEitaNodes)
            for i=1:length(obj.nodes)
                obj.nodes{i}.init_x_P(x,P,length(obj.network),attackedEitaNodes)
            end
        end

        function init_x_P_forall_reach(obj,x,P)
            for i=1:length(obj.nodes)
                obj.nodes{i}.init_x_P(x,P,length(obj.network))
            end
        end
        
        function publishmeas_forall(obj,meas,h)
            for i=1:length(obj.nodes)
                obj.nodes{i}.set_meas(meas,h);
            end
        end
    
        function mineign=checkest_forall_condMsg(obj)
            for i=1:length(obj.nodes)
                mineign(i)=obj.nodes{i}.checkest_condMsg();
            end
        end
       function checkEkfP1(obj,diffEnable,fstate,algorithm,method,Q)
            for i=1:length(obj.nodes)
                obj.nodes{i}.checkEkfP1(diffEnable,fstate,algorithm,method,Q);
            end
        end
       function checkInterconn(obj,fstate,method,Q)
            for i=1:length(obj.nodes)
                obj.nodes{i}.checkInterconn(fstate,method,Q);
            end
        end

        function RunMeasFlag= checkekf_p1_forallUnconn(obj)
            for i=1:length(obj.nodes)
                obj.nodes{i}.checkekf_p1();
            end
            mobileIndex =9;
            RunMeasFlag = obj.nodes{mobileIndex}.measUpdateFlag;
            obj.nodes{mobileIndex}.measUpdateFlag = 0;
        end
        
        
        function RunMeasFlag= checkekf_p1_forall_fake(obj,dt_ref,meas)
            for i=1:length(obj.nodes)

                obj.nodes{i}.checkekf_p1_fake();
            end

            RunMeasFlag =0;
            %RunMeasFlag = obj.MeasurementPCovUpdate(dt_ref,meas);
        end        
%         function checkukf_p1_forall(obj,M,alpha,bita,kk,delta)
%             for i=1:length(obj.nodes)
%                 obj.nodes{i}.checkukf_p1(M,alpha,bita,kk,delta);
%             end
%         end
        
        function checkEkfP2(obj,fstate,Q,diffEnable,algorithm)
            for i=1:length(obj.nodes)
                obj.nodes{i}.checkEkfP2(fstate,Q,diffEnable,algorithm);
            end
        end
        
        
        function [RunDiffFlag,RunTimeFlag]= checkekf_p2_forallUnconn(obj,fstate,Q,diffEnable,dt_ref)
            for i=1:length(obj.nodes)

                obj.nodes{i}.checkekf_p2(fstate,Q,diffEnable);
            end
            mobileIndex =9;
             RunDiffFlag =obj.nodes{mobileIndex}.diffUpdateFlag ;
            RunTimeFlag = obj.nodes{mobileIndex}.timeUpdateFlag;
        end

        function Q=getQ_red(obj,i)
            Q=[];
            for j=sort(obj.network{i})
                Q = [Q; obj.nodes{j}.getProcessVar()];
            end
            Q = diag(Q);
        end
        
        function publisheita_forall(obj)
            for i=1:length(obj.nodes)
                if(obj.nodes{i}.ready_to_ekf_p1 == 1) %eita is ready to publish
                    for j=1:length(obj.nodes)
                        obj.nodes{j}.seteital(i,obj.nodes{i}.eita,obj.nodes{i}.P);
                    end
                end
            end
        end
        
        function reseteita(obj)
            for i=1:length(obj.nodes)
                obj.nodes{i}.reseteital();
            end
        end
        %%%%%%%%%%%%%% network topology
        
        
        %set the neighbours for all
        function setneigh_forall(obj)
            for i=1:length(obj.nodes)
                obj.nodes{i}.setmyneigh(sort(obj.network{i}),obj.network);
                obj.nodes{i}.setSizebuffer(length(obj.network{i}));
            end
        end
        
        function publishMeasForNeigh(obj,meas,h)
            srcIndex=meas.getNodeIdx() ;
            for i =unique(obj.network{srcIndex}) %obj.network{srcIndex}
                %send the need msg only to the nei
                obj.nodes{i}.set_meas(meas,h);
            end
        end

        

        function addmeas(obj,meas,pi,oi,bi,pj,oj,bj)
            srcIndex=meas.getSourceId() +1;
            desIndex=meas.getDestId() +1;
            
            if srcIndex ==11
                srcIndex =9;
            end
            if desIndex ==11
                desIndex=9;
            end
            %check if the measuremnt is between neighbours
            % if not just discard it
            if unique([obj.network{srcIndex},obj.network{desIndex}])   %( any(obj.network{srcIndex}==desIndex) )
                %                 if (any(obj.measTable(:,1)== srcIndex) && any(obj.measTable(:,2)== desIndex))
                %                     obj.resetTable=1;
                %                 else
                obj.measTable(obj.indexTable,1)=srcIndex;
                obj.measTable(obj.indexTable,2)=desIndex;
                obj.measTable(obj.indexTable,3)=meas.R_ij;
                obj.measTable(obj.indexTable,4)=meas.attackValue;
                obj.indexTable = obj.indexTable +1;
                %                 end
            end
        end
        
        function runEstimation(obj)
            if(length(obj.measTable) >=obj.numberofTabRows)
                [obj.pl_list,obj.att,obj.attGndTruth]=  estimate_sec_center(obj.measTable,obj.pl_list,obj.numberofTabRows);

          end
        end
        
        function checkreset(obj)
            if obj.resetTable==1
                obj.measTable=[0,0,0,0];
                obj.indexTable =1;
            end
        end
     
        function publishEitaForNeigh(obj)
            for i=1:length(obj.nodes)
                if(obj.nodes{i}.ready_to_ekf_p1 == 1 && obj.nodes{i}.eitaIsSent ==0) %eita is ready to publish
                    obj.nodes{i}.eitaIsSent =1;
                    for j = sort(obj.network{i})
                        %if(obj.nodes{j}.ready_to_ekf_p1==1)
                        obj.nodes{j}.setEital(i,obj.nodes{i}.eita,obj.nodes{i}.eita_zonotope);
                       
                        if(j ~= i)% if it is to my self, do not count!
                            obj.sentmsgsD = obj.sentmsgsD +1;
                        end
                        %end
                    end
                end
            end
        end
        
        function publishZono(obj,meas,h)
            srcIndex=meas.getNodeIdx() ;
            %only the node gets its measurement
            obj.nodes{srcIndex}.yl{1} = meas.vectorize();
            obj.nodes{srcIndex}.Rl{1} = meas.getCovariance();
            obj.nodes{srcIndex}.hl{1} = h;           
            for i = 1:length(obj.network{srcIndex})
                % get the zonotopes from the node neighbors 
                % do not consider the node zonotope
                if i~=srcIndex
                    obj.nodes{srcIndex}.setZonol(i,obj.nodes{i}.x_zonotope);
                end
            end            
        end
        function calcPropDelaytoAlpha(obj)
            for i=1:length(obj.nodes)
                obj.propdelay2alpha(i)= norm( obj.nodes{1}.true_p- obj.nodes{i}.true_p )/obj.LIGHTSPEED;
            end
        end

        % get process variance
        function Q = getProcessVar_red(obj)
            Q = [];
            for i=1:length(obj.nodes)
                Q = [Q; obj.nodes{i}.getProcessVar()];
            end
            Q= diag(Q);
        end
        
        % get initial variance
        function P = getInitialVar_red(obj)
            p = [];
            for i=1:length(obj.nodes)
                p = [p; obj.nodes{i}.getInitialVar()];
            end
            P = diag(p);
        end
        
        function setAttackedNodes(obj,nodesArr,valuesArr)
            obj.attack_nodes=nodesArr;
            obj.attack_values=valuesArr;
            %obj.attack_offsets=valuesOffsetArr;
        end
        
        function  checkest_forall_dopt(obj)
            for i=1:length(obj.nodes)
                 obj.nodes{i}.checkest_dopt(obj.numberofTabRows,length(obj.network));
            end
        end        
        
        function publishmeas_forneigh_dopt(obj,meas)
            srcIndex=meas.getSourceId() +1;
            desIndex=meas.getDestId() +1;
            
            if srcIndex ==11
                srcIndex =9;
            end
            if desIndex ==11
                desIndex=9;
            end
            
            ,pi,oi,bi,pj,oj,bj
            %check if the measuremnt is between neighbours
            % if not just discard it
            if( any(obj.network{srcIndex}==desIndex) )
                %                 obj.nodes{srcIndex}.set_meas_dopt(meas);
                %                 obj.nodes{desIndex}.set_meas_dopt(meas);
                %for i =obj.network{srcIndex}
                %for i =1:length(obj.network)
                for i =unique([obj.network{srcIndex},obj.network{desIndex}])
                    %the distance is between me and my neighbour
                  %  if( i==srcIndex || i==desIndex )
                        obj.nodes{i}.set_meas_dopt(meas);
                  %  end
                end
            end
            
        end
        


    end
end