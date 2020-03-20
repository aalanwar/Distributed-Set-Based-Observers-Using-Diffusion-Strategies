classdef Node_R < handle
    
    properties
        
        %Zonotopes
        x0_z;
        eita_zonotope;
        x_zonotope;
        myY;
        myh;
        attackedEitaNodes;
        debugEnable;
        logFileName;
        zonoforDisSave;
        readytotakeDis=0;
        algdone =0;
        %% ------------
        id = 0;
        name = '';
        eitaIsSent =0;
        % true state variables
        true_p = [0;0;0];
        mobile = false;
        %% ------------
        measUpdateFlag=0;
        diffUpdateFlag=0;
        timeUpdateFlag=0;
        readyTochangeCovFlag=0;
        % corresponding rigid body id, if needed
        rigidbody_id = [];
        
        % state estimation variables
        %         state_p = [0;0;0];
        state_clkofst = 0;
        state_clkbias = 0;
        
        %
        
        state_p = [0,0,0];
        state_att = 0;
        state_att_offset=0;
        %att=0;
        mineig=-1.5e-15;
        
        % --- process variances --- Q
        % by default a node is static with no var on position
        var_p = [0.0; 0.0; 0.0];
        var_co = 1e-12;%1%10e-12; % was 1e-15
        %var_co = 1e-20;
        % clock bias experimentally drifts as much as +/- 2.0 ns / sec
        %         var_cb = 1.00; % ??????
        %         vari_cb = 1;
        var_cb = 1e-20; % ??????
        vari_cb = 1e-20;
        var_att = 1;
        var_att_offset =1e-20;%1e-13;
        
        % eita = [ p o b ad ao]
        % xdiff = [ p o b ad ao pE oE bE adE aoE]
        
        
        %% ------ Eita
        % Meas Cov
        Measvar_Eita_P ;
        Measvar_Eita_O ;
        Measvar_Eita_B ;
        Measvar_Eita_Ad ;
        Measvar_Eita_Ao ;
        % Process Cov
        var_Eita_P ;
        var_Eita_O ;
        var_Eita_B ;
        var_Eita_Ad ;
        var_Eita_Ao ;
        var_Eita_PE ;
        var_Eita_OE ;
        var_Eita_BE ;
        var_Eita_AdE ;
        var_Eita_AoE ;
        
        
        vari_att_offset=1e-2;
        %       var_att_offset =0.5e-20;%1e-13;
        %       vari_att_offset=1e-20;
        vari_att=1;%initial var
        
        x_order_red=[];
        
        
        % --- initial variances --- initial P
        % initial position variance, dependent on area size
        vari_p = [5; 5; 5];
        
        
        
        vari_p_red = [5; 5; 5];
        vari_p_node1_red = [1e-14; 1e-14; 1e-14 ];
        % clock offset is uniform [0,17] roughly, or 24 var
        vari_co = 1e-12;
        vari_co_red = 1e-3;
        vari_co_node1_red = 1e-14;
        
        
        
        % initial bias could be as high as +/- 2ppm - +/- 2ppm = +/-4ppm (uniform)
        
        %         vari_cb_red = 1;
        %         vari_cb_node1_red = 1;
        % message transmission times
        tlast_type1 = 0;
        tlast_type2 = 0;
        tlast_type3 = 0;
        
        % reference node
        is_reference = false;
        R;
        %%%%%%% for 2 hop
        M_hop=false;
        dl_Mhop ;
        R_errl_Mhop ;
        measl_Mhop ={};
        pl_Mhop  ;
        ofstl_Mhop ;
        biasl_Mhop ;
        Rx_Mhop ;
        x_order_red_Mhop =[];
        fill_index_Mhop=1;
        %%%%%%%%%%%%
        dl;
        pl;
        ofstl;
        biasl;
        R_errl;
        measl;
        yl={};
        Rl={};
        hl={};
        measts={};
        eita={};
        x;
        x_i_i;%after the diffusion step
        P;
        P_minus;
        P_i_i;
        eital;
        zonol;%zonotopes for neighbours
        nodeIndex;
        Pl;
        eita_red;
        Q_red;
        c=[0.1;0.3;0.1;0.3;0.1;0.1];
        P_next;
        x_next;
        G=1;
        Rx ;
        ready_to_est =0;
        ready_to_ekf_p1 =0;
        ready_to_ekf_p2 =0;
        
        % eitaIsSent=0;
        
        state_ready =0;
        fill_index=1;
        fill_index_est=2;
        eita_index=1;
        zono_index =1;
        size_R;
        myneigh=[];
        ekf_p1_done =0;
        ekf_p2_done =0;
        ready_to_ukf_p1 =0;
        ready_to_ukf_p2 =0;
        ukf_p1_done =0;
        X_min;
        %%%%%%%%%%%%%%%
        %% sec d opt
        measTableDis=[0,0,0,0];
        measTableAll=[0,0,0,0,0,0];
        %measl={}
        resetTable = 0;
        indexTable=1;
        pl_list;
        off_list;
        bias_list;
        attDis;
        attOff;
        attGndTruth;
        network;
        %%%%%%%%
        %
        xDiff;
        pDiff;
        QDiff;
        RDiff;
    end
    
    methods
        % Constructor
        function obj = Node_R(id, name)
            obj.id = id;
            obj.name = name;
        end
        
        % make this a mobile node
        function setToMobile(obj)
            obj.mobile = true;
            %obj.var_p = [0.2; 0.2; 0.2];
            obj.var_p = [1; 1; 1]       
        end
        
        % make this a static node
        function setToStatic(obj)
            obj.mobile = false;
        end
        
        % assign a rigid body id
        function setRigidBodyId(obj, rb)
            obj.rigidbody_id = rb;
        end
        
        % get rigid body id
        function rb = getRigidBodyId(obj)
            rb = obj.rigidbody_id;
        end
        
        % fix static location to true location
        function setPositionToTrue(obj)
            obj.state_p = obj.true_p';
            obj.vari_p = [0; 0; 0];
        end
        
        % is this node mobile?
        function m = isMobile(obj)
            m = obj.mobile;
        end
        
        % set true position
        function setTruePosition(obj, pos)
            if ~iscolumn(pos)
                pos = pos';
            end
            obj.true_p = pos;
        end
        
        % set the time a measurement of a given type was last sent
        function setLastMeasTime(obj, type, time)
            switch type
                case 1
                    obj.tlast_type1 = time;
                case 2
                    obj.tlast_type2 = time;
                case 3
                    obj.tlast_type3 = time;
                otherwise
                    error('Unrecognized measurement type');
            end
        end
        
        % get the time a measurement of a given type was last sent
        function t = getLastMeasTime(obj, type)
            switch type
                case 1
                    t = obj.tlast_type1;
                case 2
                    t = obj.tlast_type2;
                case 3
                    t = obj.tlast_type3;
                otherwise
                    error('Unrecognized measurement type');
            end
        end
        
        % get true position
        function p = getTruePosition(obj)
            p = obj.true_p;
        end
        
        function att=getStateAttack(obj)
            att=obj.state_att;
        end
        
        function att=getStateAttackOffset(obj)
            att=obj.state_att_offset;
        end
        % set the estimated state position
        function setStatePosition(obj, pos)
            %  if ~iscolumn(pos)
            %      pos = pos';
            % end
            obj.state_p = pos;
        end
        
        % set the clock offset
        function setStateClockOfst(obj, ofst)
            obj.state_clkofst = ofst;
        end
        
        % set the clock bias
        function setStateClockBias(obj, bias)
            obj.state_clkbias = bias;
        end
        
        % get the estimated state position
        function p = getStatePosition(obj)
            p = obj.state_p;
        end
        
        % get the node id
        function id = getId(obj)
            id = obj.id;
        end
        
        % get the node name (string)
        function name = getName(obj)
            name = obj.name;
        end
        
        % get the state vector
        function s = getState(obj)
            if size(obj.state_p,1) ==1
                s = [obj.state_p'];
            else
                s = [obj.state_p];
            end
            
        end
        
        
        % set the state vector
        function setState(obj, si)
            obj.state_p = si(1:3);
           % obj.state_att = si(4);
        end
        
        % get the process variance vector
        function q = getProcessVar(obj)
            q = [obj.var_p];
        end
        
        % get the process variance vector
        function q = getEitaProcessVar(obj)
            q = [obj.var_Eita_P ;
                obj.var_Eita_O ;
                obj.var_Eita_B ;
                obj.var_Eita_Ad ;
                obj.var_Eita_Ao ;
                obj.var_Eita_PE ;
                obj.var_Eita_OE ;
                obj.var_Eita_BE ;
                obj.var_Eita_AdE ;
                obj.var_Eita_AoE ];
        end
        
        function q = getEitaMeasVar(obj)
            q = [
                obj.Measvar_Eita_P ;
                obj.Measvar_Eita_O ;
                obj.Measvar_Eita_B ;
                obj.Measvar_Eita_Ad ;
                obj.Measvar_Eita_Ao ;
                ];
        end
        
        % get the process variance vector
        function q = getEitaProcessVarImpState(obj)
            q = [obj.var_Eita_P ;
                obj.var_Eita_O ;
                obj.var_Eita_B ;
                %obj.var_Eita_Ad ;
                %obj.var_Eita_Ao ;
                obj.var_Eita_PE ;
                obj.var_Eita_OE ;
                obj.var_Eita_BE
                %obj.var_Eita_AdE ;
                %obj.var_Eita_AoE
                ];
        end
        
        function q = getEitaMeasVarImpState(obj)
            q = [
                obj.Measvar_Eita_P ;
                obj.Measvar_Eita_O ;
                obj.Measvar_Eita_B %;
                %obj.Measvar_Eita_Ad ;
                %obj.Measvar_Eita_Ao ;
                ];
        end
        % get the initial variance vector
        function q = getInitialVar(obj)
            q = [obj.vari_p];
        end
        
        % set this as the reference node
        function setAsReference(obj)
%             obj.state_clkofst = 0;
%             obj.state_clkbias = 0 ;
%             obj.state_att_offset = 0;
%             
%             obj.vari_p = [1e-10; 1e-10; 1e-10 ];
%             obj.var_co = 1e-10 ;
%             obj.vari_co = 1e-10 ;
%             obj.var_cb = 1e-10 ;
%             obj.vari_cb = 1e-10 ;
%             obj.vari_att_offset= 1e-10;
%             obj.var_att_offset= 1e-10;
%             obj.var_att= 1e-10;
%             obj.vari_att = 1e-10;
            %             obj.vari_att = 1e-14 ;
            %             obj.vari_att_offset= 1e-14 ;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %             obj.state_clkofst = 0;
            %             obj.state_clkbias = 0 ;
            %             obj.vari_p = [0; 0;0 ];
            %             obj.var_co = 0;
            %             obj.vari_co = 0 ;
            %             obj.var_cb = 0 ;
            %             obj.vari_cb = 0 ;
            %             %obj.vari_att = 0;
            %             obj.vari_att_offset= 0;
            %             obj.var_att_offset= 0;
            
            obj.is_reference = true;
            
        end
        
        % is this node a reference
        function val = isReference(obj)
            val = obj.is_reference;
        end
        
        % set covariances
        function setPositionCovar(obj, v)
            obj.var_p = [v; v; v];
        end
        
        function setOffsetCovar(obj, v)
            obj.var_co = v;
        end
        
        function setBiasCovar(obj, v)
            obj.var_cb = v;
        end
        
        function setAttOffCovar(obj, v)
            obj.var_att_offset = v;
        end
        
        function setAttDisCovar(obj, v)
            obj.var_att = v;
        end
        % %
        %xdiff = [ P O B Ad Ao PE OE BE AdE AoE]
        %%%% -------------- EIta MicroKF Process Cov
        function setEita_P_Covar(obj, v)
            obj.var_Eita_P = [v; v; v];
        end
        function setEita_O_Covar(obj, v)
            obj.var_Eita_O = v;
        end
        function setEita_B_Covar(obj, v)
            obj.var_Eita_B = v;
        end
        function setEita_Ad_Covar(obj, v)
            obj.var_Eita_Ad = v;
        end
        function setEita_Ao_Covar(obj, v)
            obj.var_Eita_Ao = v;
        end
        function setEita_PE_Covar(obj, v)
            obj.var_Eita_PE = [v; v; v];
        end
        function setEita_OE_Covar(obj, v)
            obj.var_Eita_OE = [v];
        end
        function setEita_BE_Covar(obj, v)
            obj.var_Eita_BE = [v];
        end
        function setEita_AdE_Covar(obj, v)
            obj.var_Eita_AdE = [v];
        end
        function setEita_AoE_Covar(obj, v)
            obj.var_Eita_AoE = [v];
        end
        %% --------------
        %%%% -------------- EIta MicroKF Meas Cov
        function setEita_P_MeasCovar(obj, v)
            obj.Measvar_Eita_P = [v; v; v];
        end
        function setEita_O_MeasCovar(obj, v)
            obj.Measvar_Eita_O = v;
        end
        function setEita_B_MeasCovar(obj, v)
            obj.Measvar_Eita_B = v;
        end
        function setEita_Ad_MeasCovar(obj, v)
            obj.Measvar_Eita_Ad = v;
        end
        function setEita_Ao_MeasCovar(obj, v)
            obj.Measvar_Eita_Ao = v;
        end
        %%---------------
        function setSizebuffer(obj, size_R )
            obj.size_R = size_R;
            %             if (obj.id ==2 || obj.id ==7) % charli is not reciveing from hotel
            %                 obj.size_R = obj.size_R -1;
            %             end
            obj.Rx = zeros(1,obj.size_R);
        end
        
        function set_meas(obj,meas,h)
            
            srcIdx = meas.getNodeIdx() ;
            if(srcIdx == obj.id +1 )
                obj.myY = meas.vectorize();
                obj.myh = h;
            end
            obj.yl{obj.fill_index} = meas.vectorize();
            obj.Rl{obj.fill_index} = meas.getCovariance();
            obj.hl{obj.fill_index} = h;
            obj.Rx(obj.fill_index) = 1; %available y's and Rl
            obj.fill_index = obj.fill_index +1;
            for i =1:length(obj.Rx)
                if( obj.Rx(i) ==0 )
                    break; %not ready yet
                end
                if(i == length(obj.Rx))
                    obj.ready_to_ekf_p1 =1;      
                end
            end
            
        end
        
        function set_meas_dopt(obj,meas)
            
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
            %  if( any(obj.myneigh==desIndex) )
            %                 if (any(obj.measTable(:,1)== srcIndex) && any(obj.measTable(:,2)== desIndex))
            %                     obj.resetTable=1;
            %                 else
            obj.measTableAll(obj.indexTable,1)=srcIndex;
            obj.measTableAll(obj.indexTable,2)=desIndex;
            obj.measTableAll(obj.indexTable,3)=meas.R_ij;
            obj.measTableAll(obj.indexTable,4)=meas.attackValue;
            obj.measTableAll(obj.indexTable,5)=meas.d_ij;
            obj.measTableAll(obj.indexTable,6)=meas.attackOffset;
            obj.measl{obj.indexTable}=meas;
            obj.indexTable = obj.indexTable +1;
            
            %   end
            
        end
        function checkest_dopt(obj,numberofTabRows,numOfNodes)
            if(length(obj.measTableAll) > numberofTabRows)
                
                [obj.bias_list,obj.off_list,obj.attOff,obj.pl_list,obj.attDis,obj.attGndTruth]=  estimate_sec_Dopt(obj.id+1, obj.network,  obj.measTableAll(end-numberofTabRows:end,:),obj.pl_list,numberofTabRows,obj.attDis,obj.attOff,obj.off_list,obj.bias_list,numOfNodes);%,obj.numberofattacks); %,obj.measl
                obj.state_p = obj.pl_list{obj.id+1};
                obj.state_clkofst = obj.off_list{obj.id+1};
                obj.state_clkbias = obj.bias_list{obj.id+1};
                obj.state_att = obj.attDis(obj.id+1);
                obj.state_att_offset=obj.attOff(obj.id+1);
            end
        end
        
        
        function checkEkfP1(obj,diffEnable,fstate,algorithm,method,Q)
            obj.P_minus = obj.P;
            if(obj.ready_to_ekf_p1 == 1 && obj.ekf_p1_done ==0)
                obj.efk_part1(fstate,algorithm,method,Q);
                obj.ekf_p1_done=1;
                obj.measUpdateFlag=1;
                if(~diffEnable && strcmp(algorithm,'set-membership'))
                    obj.x= obj.eita;
                    obj.x_zonotope = obj.eita_zonotope;
                    ekf_part3(obj,Q,fstate);
                    %obj.x_zonotope = deleteAligned(obj.x_zonotope);
                    
                    obj.zonoforDisSave=obj.x_zonotope;
                    obj.readytotakeDis=1;
                    obj.algdone =1;
                    obj.reseteital();
                    obj.x_zonotope = reduce(obj.x_zonotope,'girard',40);
                elseif(~diffEnable && strcmp(algorithm,'luenberger'))
                    obj.x= obj.eita;
                    obj.x_zonotope = obj.eita_zonotope;
                    %obj.x_zonotope = deleteAligned(obj.x_zonotope);
                    obj.x_zonotope = reduce(obj.x_zonotope,'girard',40);
                    obj.zonoforDisSave=obj.x_zonotope;
                    obj.readytotakeDis=1;
                    obj.algdone =1;
                    obj.reseteital();
                end
            end
        end


        function init_x_P(obj,x,P,numOfNodes)
            obj.x=x;
            obj.x_i_i=x;
            obj.eita=x;
            obj.P=P;
            obj.P_minus=P;
            obj.P_i_i=P;
        end
        
        
        
        function init_x(obj,x)
            obj.state_p=x;
        end
        
        
        
        function checkEkfP2(obj,fstate,Q,diffEnable,algorithm)
            
            if(obj.ready_to_ekf_p2 == 1 && obj.ready_to_ekf_p1 == 1)
                
                %obj.efk_part2(fstate,Q,diffEnable);%,typeDisAtt,typeTimeAtt,currentTime,t_stop);
                obj.efk_part2_argmin(fstate,Q,diffEnable);
                %obj.efkPart2Bundle(fstate,Q,diffEnable);
                %obj.efkPart2BundleInterval(fstate,Q,diffEnable);
                %obj.efkPart2BundleInterval2ndApp(fstate,Q,diffEnable);
                obj.diffUpdateFlag = 1;
                obj.reseteital();
                % obj.ekf_p2_done =1;;
                if obj.debugEnable==1
                    logMsg(obj.logFileName,'Node Id = %d',obj.id);
                    logVar(obj.logFileName,'x',obj.x);
                    logVar(obj.logFileName,'x_zonotope.center',obj.x_zonotope.center);
                    logVar(obj.logFileName,'x_zonotope.generators',obj.x_zonotope.generators);
                    nfro = norm2center(obj.x_zonotope,'fro');
                    logMsg(obj.logFileName,'norm2center = %f',nfro);
                end
                if strcmp(algorithm,'set-membership')
                    ekf_part3(obj,Q,fstate);
                end
                
                obj.zonoforDisSave=obj.x_zonotope;
                obj.readytotakeDis=1;
                obj.algdone =1;
            end

            %if(~diffEnable)
            %    ekf_part3(obj,Q,fstate);
                %obj.reseteital();
            %end
            %ekf_part3(obj,Q,fstate);
        end
        
        
        
        
        function setEital(obj,index,eita,eita_zonotope)
            %new eita and it is not my eita as well
            if (~any(obj.nodeIndex == index) && index ~= obj.id+1 )
                obj.nodeIndex(obj.eita_index)=index;
                obj.eital{obj.eita_index}= eita;
                %get eita zonotope from neighbours
                obj.zonol{obj.eita_index}= eita_zonotope;
                obj.eita_index = obj.eita_index +1;
            end
            
            if obj.eita_index > length(obj.myneigh) -1
                obj.ready_to_ekf_p2 =1;
                %efk_part2(obj,Q,fstate)
            end
            
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   % related work     
        function setZonol(obj,index,x_zonotope)
                obj.nodeIndex(obj.zono_index)=index;
                %get zonotope from neighbours
                obj.zonol{obj.zono_index}= x_zonotope;
                obj.zono_index = obj.zono_index +1;
            if obj.zono_index > length(obj.myneigh) -1
                obj.ready_to_ekf_p1 =1;
            end           
        end
        function checkInterconn(obj,fstate,method,Q)
            if(obj.ready_to_ekf_p1 == 1 && obj.ekf_p1_done ==0)
                %[obj.x_zonotope] = interconnRWork(fstate,method,Q,obj.hl,obj.Rl,obj.yl,obj.x_zonotope,obj.zonol);
                [obj.x_zonotope] = kalmanInspired(fstate,method,Q,obj.hl,obj.Rl,obj.yl,obj.x_zonotope,obj.zonol);
                obj.zonoforDisSave=obj.x_zonotope;
                obj.readytotakeDis=1;
                %obj.x_zonotope = deleteAligned(obj.x_zonotope);
                obj.x_zonotope = reduce(obj.x_zonotope,'girard',10);%40
                obj.x = obj.x_zonotope.center;
                obj.ekf_p1_done=1;
                obj.algdone=1;
                obj.reseteital();
            end
        end
        
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     
        function reseteital(obj)
            obj.eital={};  
            obj.zonol={};
            obj.eita_red=[];
            obj.yl={};
            obj.Rl={};
            obj.Rx=zeros(1,obj.size_R);
            obj.x_order_red=[];
            obj.ready_to_est =0;
            obj.ready_to_ekf_p1 =0;
            obj.ready_to_ekf_p2 =0;
            obj.fill_index =1;
            obj.hl ={};
            obj.eita_index = 1;
            obj.zono_index=1;
            obj.ekf_p1_done =0;
            obj.ekf_p2_done =0;
            obj.nodeIndex=[];
            obj.eitaIsSent =0;
        end
        
        function efk_part1(obj,fstate,algorithm,method,Q)
            %obj.eita_zonotope = zonotope([0 1 0;0 0 1]);

            if strcmp(algorithm,'set-membership')
                [obj.eita,obj.eita_zonotope] = set_mem_p1(method,obj.x,obj.P,obj.hl,obj.Rl,obj.yl,obj.x_zonotope);
            elseif strcmp(algorithm,'luenberger')
                [obj.eita,obj.eita_zonotope] = inter_p1_berger(method,obj.x,obj.hl,obj.Rl,obj.yl,obj.x_zonotope,obj.myY,obj.myh,fstate,Q);     
            end
            if obj.debugEnable==1
                logMsg(obj.logFileName,'Node Id = %d',obj.id);
                logVar(obj.logFileName,'eita',obj.eita);
                logVar(obj.logFileName,'eita_zonotope.center',obj.eita_zonotope.center);
                logVar(obj.logFileName,'eita_zonotope.generators',obj.eita_zonotope.generators);
                nfro = norm2center(obj.eita_zonotope,'fro');
                logMsg(obj.logFileName,'norm2center = %f',nfro);
            end
            obj.x = obj.eita;
            obj.P_i_i= obj.P;
        end
        
        function efk_part2_argmin(obj,fstate,Q,diffEnable)
                %obj.x_zonotope = reduce(obj.x_zonotope,'girard',3);
                obj.x_zonotope = diffusionArgmin(obj.c,obj.x_zonotope,obj.zonol,obj.eita);
                %newlist =  obj.zonol;
                %newlist{length(obj.zonol)+1} = obj.x_zonotope;
                %obj.x_zonotope  = andAveraging(newlist);
                %obj.x_zonotope = diffusionArgmin_test(obj.c,obj.x_zonotope,obj.zonol,obj.eita);
                %obj.x_zonotope = deleteAligned(obj.x_zonotope);
                obj.x_zonotope = reduce(obj.x_zonotope,'girard',20);%20
                obj.x = obj.x_zonotope.center;
        end
        
        
        function efk_part2(obj,fstate,Q,diffEnable)
            if(diffEnable)
                obj.x_zonotope = reduce(obj.x_zonotope,'girard',3);
                for i =1:length(obj.c)-1
                    obj.zonol{i}=reduce(obj.zonol{i},'girard',3);
                end
                includedResults= zeros(1,length(obj.c)-1);
                %x_zonotope_interval = interval(obj.x_zonotope);
                obj.x =  obj.eita;
                
                for i=1:length(obj.c)-1
                    obj.x = obj.x + obj.eital{i};
                    includedResults(i)=1;
                end
                
                obj.x=obj.x/(sum(includedResults)+1);%1 for myself for correct averaging
                % ekf_part3(obj,Q,fstate);
                
                obj.x_zonotope = obj.eita_zonotope;
                %   obj.P=0;
                for i =1:length(obj.c)-1
                    %minkowski sum of zonotopes between neighbours
                    obj.x_zonotope = obj.x_zonotope + obj.zonol{i};
                end
                %divide by n to get the average
                obj.x_zonotope = ( 1/(sum(includedResults)+1) )*obj.x_zonotope;
                %obj.x_zonotope = reduce(obj.x_zonotope,'girard',3);
            else
                %rm diff
                obj.x= obj.eita;
                obj.x_zonotope = obj.eita_zonotope;
            end
        end        
        
        
        function efkPart2Bundle(obj,fstate,Q,diffEnable)
            if(diffEnable)
                zIntersect=conZonotope(obj.eita_zonotope);
                for i =1:length(obj.zonol)
                   zIntersect = and(zIntersect,conZonotope(obj.zonol{i})); 
                end
                obj.x_zonotope = zonotope(interval(zIntersect));
                obj.x = obj.x_zonotope.center;
                
                obj.x_zonotope = reduce(obj.x_zonotope,'girard',2);
%                 %bundle Intersection between zonotopes
%                 tempList = obj.zonol;
%                 listLength = length(obj.zonol)+1;
%                 tempList{listLength} = obj.eita_zonotope;
%                 zB   = zonotopeBundle(tempList); % instantiate zonotope bundle from Z1, Z2
%                 zIntersect = zonotope(conZonotope(zB));
%                 obj.x_zonotope = zIntersect;
%                 obj.x = zIntersect.center;
                
                
            else
                %rm diff
                obj.x= obj.eita;
                obj.x_zonotope = obj.eita_zonotope;
            end
        end
        
        function efkPart2BundleInterval(obj,fstate,Q,diffEnable)
            if(diffEnable)
                %obj.x_zonotope = reduce(obj.x_zonotope,'girard',3);
                %bundle Intersection between zonotopes
                tempList = obj.zonol;
                listLength = length(obj.zonol)+1;
                tempList{listLength} = obj.eita_zonotope;
                zB   = zonotopeBundle(tempList); % instantiate zonotope bundle from Z1, Z2
                %zIntersect = zonotope(conZonotope(zB));
                zIntersect = zonotope(interval(zB));
                obj.x_zonotope = zIntersect;
                obj.x = zIntersect.center;
                
                %obj.x_zonotope = reduce(obj.x_zonotope,'girard',2);
            else
                %rm diff
                obj.x= obj.eita;
                obj.x_zonotope = obj.eita_zonotope;
            end
        end
        
        function efkPart2BundleInterval2ndApp(obj,fstate,Q,diffEnable)
            if(diffEnable)
                %obj.x_zonotope = reduce(obj.x_zonotope,'girard',3);
                %bundle Intersection between zonotopes
                templist{1} = obj.eita_zonotope;
                zonoBund = zonotopeBundle(templist);
                for i =1:length(obj.zonol)
                   templist{1} = obj.zonol{i};
                   zonoBund = zonoBund &  zonotopeBundle(templist);
                end
                %zIntersect = zonotope(conZonotope(zB));
                zIntersect = zonotope(interval(zonoBund));
                obj.x_zonotope = zIntersect;
                obj.x = zIntersect.center;
                
                %obj.x_zonotope = reduce(obj.x_zonotope,'girard',2);
            else
                %rm diff
                obj.x= obj.eita;
                obj.x_zonotope = obj.eita_zonotope;
            end
        end
        
        function efk_part2_oldzono(obj,fstate,Q,diffEnable)
            if(diffEnable)
                %obj.x=obj.eita*obj.c(1); %%start with my eita
                obj.x=obj.eita*obj.c(1);%+3*rand(length(obj.x),1); %%start with my eita
                obj.x_zonotope = obj.eita_zonotope;
                temp = obj.x_zonotope.Z;
                temp(:,1) = temp(:,1) * obj.c(1);
                obj.x_zonotope = zonotope(temp);
                %   obj.P=0;
                for i =1:length(obj.c)-1
                    obj.x = obj.x+ obj.eital{i}*obj.c(i) ;% +  3*rand(length(obj.x),1) ;
                    obj.x_zonotope = obj.x_zonotope + obj.eital{i}*obj.c(i);
                    %      obj.P = obj.P+ obj.Pl{i}*obj.c(i) ;
                end
                obj.x_zonotope = reduce(obj.x_zonotope,'girard',30);
                
                % ekf_part3(obj,Q,fstate);
            else
                %rm diff
                %obj.x= obj.eita;
            end
        end
        
        
     
        function ekf_part3(obj,Q,fstate)
            obj.timeUpdateFlag = 1;
            obj.readyTochangeCovFlag =1;
            obj.x_i_i = obj.x;
            [obj.x,obj.x_zonotope]=set_mem_p3(fstate ,Q,obj.G,obj.x,obj.x_zonotope);
            obj.state_ready =1;
            
            % obj.reseteital();
        end
        
        
        function ekf_part3_Only(obj,Q,fstate)
            obj.timeUpdateFlag = 1;
            obj.x_i_i = obj.x;
            obj.P_i_i = obj.P;
            obj.P_minus = obj.P;
            
            
            [obj.P,obj.x]=dif_ekf_p3(fstate ,obj.P,Q,obj.G,obj.x);
            obj.state_ready =1;
            %if obj.id == 1
            obj.x(4) = 0;
            obj.x(5) = 0;
            %end
            
            % obj.reseteital();
        end
        
        %%%% network topology
        function setmyneigh(obj,neigh,network)
            obj.myneigh=neigh;
            obj.network= network;
            obj.c = zeros(1,length(neigh));
            for i = 1:length(neigh)
                obj.c(i)=1/length(neigh);
            end
        end
        

        
        function initReach(obj,xlength,debugEnable,logFileName)
            %center
            center  =zeros(xlength,1);
            width   =150;%16
            obj.x_zonotope=0.5*zonotope([center,width*eye(xlength,xlength)]); %input for reachability analysis
           % obj.x_zonotope=0.5*zonotope([center,width*eye(xlength,xlength),2*eye(2,2),4*eye(2,2),5*eye(2,2),6*ones(2,2),7*ones(2,2),4*eye(2,2),5*eye(2,2),6*ones(2,2),7*ones(2,2),4*eye(2,2),5*eye(2,2),6*ones(2,2),7*ones(2,2),4*eye(2,2),5*eye(2,2),6*ones(2,2),7*ones(2,2)]); %input for reachability analysis
           
            obj.debugEnable = debugEnable;
            obj.logFileName = logFileName;
        end
    end
end


