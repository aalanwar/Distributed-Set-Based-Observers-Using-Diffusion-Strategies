classdef DataParserR < handle
    %DATAPARSER parse tcp data logs from NTB nodes
    %   obj = DataParser( config_file, log_folder )
    
    
    properties
        fpath = '';
        raw_logs = {};
        nodeinfo = {};
        timinglogs = [];
        mocaplogs = [];
        
        % for speed, map of id to idx and name to idx
        id2idx = zeros(1,128);
        name2idx = containers.Map('KeyType', 'char', 'ValueType', 'int32');
    end
    
    methods
        % Constructor
        function obj = DataParserR( config, fpath )
            
            % open and parse config data
            fid = fopen(config);
            obj.nodeinfo = textscan(fid,'%s %f %f %f %f %s', 'Delimiter', ',');
            fclose(fid);
            
            % load hash tables for speed
            for i=1:length( obj.nodeinfo{1} )
                name = obj.nodeinfo{1}{i};
                id = obj.nodeinfo{2}(i);
                obj.id2idx(id+1) = i;
                obj.name2idx(name) = i;
            end
            
            % initialize log cell arrays
            obj.timinglogs = cell(1,length(obj.nodeinfo{1}));
            
            % read NTB timing measurements
            obj.fpath = fpath;
            d = importdata([obj.fpath '/rotatingTarget.csv']);
            if size(d,1) < 0
                error('NTB log file empty');
            end
    
            obj.timinglogs = d;
            
        end
        
        % convert hostname to id
        function id = hostnameToId(obj, name)
            idx = find(strcmp(obj.nodeinfo{1}, name));
            id = obj.nodeinfo{2}(idx);
        end
        
        % convert id to hostname
        function name = idToHostname(obj, id)
            hostidx = find(obj.nodeinfo{2} == id);
            name = obj.nodeinfo{1}(hostidx);
        end
                
        % get node index from alpha or numeric ID
        function idx = getNodeIdx(obj, node)
            if isnumeric(node)
                idx = obj.id2idx( node+1 );
            else
                idx = obj.name2idx( node );
            end
        end
       
        % get the node info
        function info = getNodeInfo(obj)
            info = obj.nodeinfo;
        end
        
        % get the node position x,y,z
        function xyz = getNodePos(obj, id)
            idx = obj.getNodeIdx(id);
            xyz = [obj.nodeinfo{3}(idx) obj.nodeinfo{4}(idx) obj.nodeinfo{5}(idx)];
        end
        
        % number of aligned measurements
        function n = getNumMeasurements(obj)
            n = size(obj.timinglogs,1);
        end
        
        % get a specific measurement
        function meas = getMeasurement(obj, idx)
            meas = obj.timinglogs(idx,:);
        end
        
            
        % deprecated ... don't use
        function t = getTotalTime(obj)
            t = obj.timinglogs(end,1) - obj.timinglogs(1,1);
        end     
    end
    
end











































