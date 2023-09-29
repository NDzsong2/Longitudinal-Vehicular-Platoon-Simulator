classdef Network < handle
    %UNTITLED4 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        networkIndex
        numOfPlatoons % N
        numOfVehicles = [] % [n_1,n_2,...,n_N] 

        platoons = [] % all the platoons

        % State variables
        time
        error
        cost

        % For plotting
        graphics = [];

    end

    methods

        function obj = Network(indexVal,numOfPlatoons,numOfVehicles,parameters,states,desiredSeparation,noiseMean,noiseStd)

            obj.networkIndex = indexVal;
            obj.numOfPlatoons = numOfPlatoons;
            obj.numOfVehicles = numOfVehicles;

            % create the platoons
            platoons = [];
            for k = 1:1:numOfPlatoons
                platoon = Platoon(k,numOfVehicles(k),parameters{k},states{k},desiredSeparation{k},noiseMean{k},noiseStd{k});
                platoons = [platoons, platoon];
            end
            obj.platoons = platoons;
                        
            obj.time = 0;
            obj.error = [0;0;0];
            obj.cost = 0;

        end

           
        function outputArg = drawNetwork(obj,figNum)
            figure(figNum); hold on;
            
            % Draw platoons
            for k = 1:1:obj.numOfPlatoons
                obj.platoons(k).drawPlatoon(figNum);
            end
            
            % Coordinate of the boundary of the bounding box
            posY1 = -5;
            posY2 = 20;
            lastPlatoon = obj.platoons(obj.numOfPlatoons);
            posX1 = lastPlatoon.vehicles(lastPlatoon.numOfVehicles).states(1)-10;
            posX2 = obj.platoons(1).vehicles(1).states(1)+10;


            % Plot the 3 topics, i.e., time,error,cost, on the top of the simulator
            obj.graphics(1) = text(posX1+5,posY2-5,['Time: ',num2str(obj.time)],'FontSize',12);
            obj.graphics(2) = text(posX1+30,posY2-5,['Error: ',num2str(round(norm(obj.error),1))],'FontSize',12);
            obj.graphics(3) = text(posX1+55,posY2-5,['Cost: ',num2str(round(obj.cost,1))],'FontSize',12); 

            obj.graphics(4) = text(posX1+5,posY2-8,['P-Error: ',num2str(round(obj.error(1),1))],'FontSize',12);
            obj.graphics(5) = text(posX1+30,posY2-8,['V-Error: ',num2str(round(obj.error(2),1))],'FontSize',12);
            obj.graphics(6) = text(posX1+55,posY2-8,['A-Error: ',num2str(round(obj.error(3),1))],'FontSize',12);


            axis([posX1,posX2,posY1,posY2])
        end


        function outputArg = update(obj,t,dt)
            
            % Do the necessary computations/generations to generate the signals to initiate the program
            totalError = zeros(3,1);

            for k = 1:1:obj.numOfPlatoons

                % Generate the noises
                obj.platoons(k).generateNoises();

                % Error Dynamics - I : Computing Platooning Errors and Controls
%                 obj.platoons(k).computePlatooningErrors1();
%                 obj.platoons(k).computeControlInputs1(t);

                % Error Dynamics - II : Computing Platooning Errors and Controls
                obj.platoons(k).computePlatooningErrors2();
                obj.platoons(k).computeControlInputs2(t); 

                % Update the states
                platoonError = obj.platoons(k).update(t,dt);
                totalError = totalError + platoonError;

            end


            % Update the time, error and cost
            costSoFar = obj.cost^2*obj.time;
            obj.time = obj.time + dt;

            
            obj.error = totalError;
            costSoFar = costSoFar + (norm(obj.error))^2*dt;
            obj.cost = sqrt(costSoFar/obj.time);
            
        end


        function outputArg = redrawNetwork(obj,figNum)
            
            % Redraw platoons
            for k = 1:1:obj.numOfPlatoons
                obj.platoons(k).redrawPlatoon(figNum); % Redraw the platoons by calling the redrawPlatoon method in platoon class
            end

            % Update the 3 topics of the figure
            if ~isempty(obj.graphics)
                delete(obj.graphics(1));
                delete(obj.graphics(2)); 
                delete(obj.graphics(3)); 
                delete(obj.graphics(4));
                delete(obj.graphics(5)); 
                delete(obj.graphics(6));

                % Coordinate of the boundary of the bounding box
                posY1 = -5;
                posY2 = 20;
                lastPlatoon = obj.platoons(obj.numOfPlatoons);
                posX1 = lastPlatoon.vehicles(lastPlatoon.numOfVehicles).states(1)-10;
                posX2 = obj.platoons(1).vehicles(1).states(1)+10;
                
                obj.graphics(1) = text(posX1+5,posY2-5,['Time: ',num2str(obj.time)],'FontSize',12);
                obj.graphics(2) = text(posX1+30,posY2-5,['Error: ',num2str(round(norm(obj.error),1))],'FontSize',12);
                obj.graphics(3) = text(posX1+55,posY2-5,['Cost: ',num2str(round(obj.cost,1))],'FontSize',12);           
                
                obj.graphics(4) = text(posX1+5,posY2-8,['P-Error: ',num2str(round(obj.error(1),1))],'FontSize',12);
                obj.graphics(5) = text(posX1+30,posY2-8,['V-Error: ',num2str(round(obj.error(2),1))],'FontSize',12);
                obj.graphics(6) = text(posX1+55,posY2-8,['A-Error: ',num2str(round(obj.error(3),1))],'FontSize',12);  

                axis([posX1,posX2,posY1,posY2])
            end

        end

        
        function outputArg = generateLeadersControlProfiles(obj,dt,tVals,vVals)
            % Here we basically have to derive each leader's control trajectory
            
            % aVals computation
            aVals = []; 
            for k = 1:1:(length(tVals)-1)
                v_1 = vVals(k);     
                t_1 = tVals(k);
                v_2 = vVals(k+1);   
                t_2 = tVals(k+1);
                a_1 = (v_2-v_1)/(t_2-t_1);
                aVals = [aVals, a_1];
            end
            aVals = [aVals, 0];

            % uVals computation
            uVals= [];
            a_0 = 0; 
            for k = 1:1:length(tVals)
                a_1 = aVals(k);     
                u_1 = (a_1-a_0)/dt;
                a_0 = a_1;
                uVals = [uVals, u_1];
            end

            % Setting initial velocities
            for k = 1:1:obj.numOfPlatoons
                obj.platoons(k).vehicles(1).states(2) = vVals(1);
                obj.platoons(k).vehicles(1).plannedControls = [tVals',uVals'];
            end
           
        
        end



        function output = loadPlatoonControllers(obj,errorDynamicsType,isCentralized,isOnlyStabilizing,gammaSqBar,nuBar,rhoBar)
            for k = 1:1:obj.numOfPlatoons

                %% Controller Types:
                % There can be three factors that determines the controller
                % type: (i) Centralized/Decentralized, (ii)
                % Stabilizing/Robust, and (iii) Error Dynamics Type
                
                if errorDynamicsType == 1           % Error dynamics formulation I
                    if isCentralized == 1           % Centralized
                        if isOnlyStabilizing == 1   % Only Stabilizing
                            status = obj.platoons(k).centralizedStabilizingControllerSynthesis1(nuBar,rhoBar);
                        else                        % Robust
                            status = obj.platoons(k).centralizedRobustControllerSynthesis1(nuBar,rhoBar,gammaSqBar);
                        end
                    else                            % Decentralized
                        if isOnlyStabilizing == 1   % Only Stabilizing
                            status = obj.platoons(k).decentralizedStabilizingControllerSynthesis1(nuBar,rhoBar);
                        else                        % Robust
                            status = obj.platoons(k).decentralizedRobustControllerSynthesis1(nuBar,rhoBar,gammaSqBar);
                        end
                    end
                else                                % Error dynamics formulation II
                    if isCentralized == 1           % Centralized
                        if isOnlyStabilizing == 1   % Only Stabilizing
                            status = obj.platoons(k).centralizedStabilizingControllerSynthesis2(nuBar,rhoBar);
                        else                        % Robust
                            status = obj.platoons(k).centralizedRobustControllerSynthesis2(nuBar,rhoBar,gammaSqBar);
                        end
                    else                            % Decentralized
                        if isOnlyStabilizing == 1   % Only Stabilizing
                            status = obj.platoons(k).decentralizedStabilizingControllerSynthesis2(nuBar,rhoBar);
                        else                        % Robust
                            status = obj.platoons(k).decentralizedRobustControllerSynthesis2(nuBar,rhoBar,gammaSqBar);
                        end
                    end
                end
                
                % Success or Failure
                if status == 1
                    disp(['Global Controller ',num2str(k),' Synthesis Success.']);
                else
                    disp(['Global Controller ',num2str(k),' Synthesis Failed.']);
                end
                
            end
        end
        


        %% From Past Projects:

        % Decentralized FSF stabilization
        % % At Network Level
        % function [K, isStabilizable] = decentralizedFSFStabilization(obj,indexing,solverOptions)
        % 
        %     if isempty(indexing)
        %         indexing = [1:1:length(obj.subsystems)];
        %     end
        % 
        %     for i = 1:1:length(indexing)
        %         iInd = indexing(i);
        %         previousSubsystems = indexing(1:i-1);                
        %         [isStabilizable,K_ii,K_ijVals,K_jiVals] = obj.subsystems(iInd).FSFStabilization(previousSubsystems, obj.subsystems, solverOptions);
        % 
        %         K{iInd,iInd} = K_ii;
        %         obj.subsystems(iInd).controllerGains.decenFSFStabCont{iInd} = K_ii;
        %         for j = 1:1:length(previousSubsystems)
        %             jInd = previousSubsystems(j);
        %             obj.subsystems(jInd).controllerGains.decenFSFStabCont{iInd} = K_jiVals{jInd};
        %             obj.subsystems(iInd).controllerGains.decenFSFStabCont{jInd} = K_ijVals{jInd};
        %             K{iInd,jInd} = K_ijVals{jInd};
        %             K{jInd,iInd} = K_jiVals{jInd};
        %         end
        % 
        %         if ~isStabilizable
        %             break
        %         end
        % 
        %     end
        % 
        %     if isStabilizable
        %         Kmat = [];
        %         for i = 1:1:length(obj.subsystems)
        %             Karray = [];
        %             for j = 1:1:length(obj.subsystems)
        %                 Karray = [Karray, K{i,j}];
        %             end
        %             Kmat = [Kmat; Karray];
        %         end
        %         K = Kmat;
        %     end
        % 
        %     % Collect all the coefficients
        % end

        
        % % At Subsystem Level
        % function [isStabilizable,K_ii,K_ijVals,K_jiVals] = FSFStabilization(obj,previousSubsystems, subsystems, solverOptions)
        %     i = length(previousSubsystems)+1;
        %     iInd = obj.index;
        %     disp(['Stabilizing at: ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
        %     % W_ij = - M_ii A_ji^T - A_ij M_jj - L_ji^T B_jj^T - B_ii L_ij
        %     % L_ij is p_i x n_j  and  K_ij = L_ij M_jj^{-1}
        % 
        %     A_ii = obj.A{iInd};
        %     B_ii = obj.B{iInd};
        %     n_i = obj.dim_n;
        %     p_i = obj.dim_p; 
        % 
        %     if isempty(previousSubsystems)
        %         % The subsystem only need to test W_ii > 0  
        % 
        %         M_ii = sdpvar(n_i,n_i);
        %         L_ii = sdpvar(p_i,n_i,'full');
        % 
        %         W_ii = - M_ii*A_ii' - A_ii*M_ii - L_ii'*B_ii' - B_ii*L_ii;
        %         con1 = M_ii >= 0.000000001*eye(n_i);
        %         con2 = W_ii >= 0;
        %         sol = optimize([con1,con2],[],solverOptions);
        %         isStabilizable = sol.problem==0;
        % 
        %         M_iiVal = value(M_ii);
        %         L_iiVal = value(L_ii);
        %         W_iiVal = value(W_ii);
        %         tildeW_i = W_iiVal; % Note that, here, \tilde{W}_ii = W_ii = \tilde{W}_i. This also needs to be stored
        % 
        %         if abs(det(tildeW_i))<0.000000001
        %             disp("Error: det(tildeW_ii) low");
        %             isStabilizable = 0;
        %         end
        % 
        %         K_ii = L_iiVal/M_iiVal; % This needs to be stored
        %         K_jiVals = [];
        %         K_ijVals = [];
        % 
        %         obj.dataToBeDistributed.tildeW = tildeW_i; % Storing
        %         obj.dataToBeDistributed.M = M_iiVal; % Storing
        %         obj.controllerGains.decenFSFStabCont{iInd} = K_ii;
        % 
        %         disp(['Data saved ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
        %         if ~isStabilizable
        %             disp(['Not stabilizable at: ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
        %         end
        % 
        %     else
        %         % This subsystem has to talk with all the previosSubsystems
        %         % tildeW_ii > 0 iff [M_i, W_i'; W_i, W_ii] > 0 is required where 
        %         % M_i =
        %         % inv((scriptD_i*scriptA_i^T)^{-1}*(scriptD_i)*(scriptD_i*scriptA_i^T)^{-1}') = scriptA_i*scriptD_i*scriptA_i'
        % 
        % 
        %         % M_i term
        %         blockSize = obj.dim_n; 
        %         scriptA_i = [];
        %         scriptD_i = [];
        %         for j = 1:1:length(previousSubsystems)
        %             jInd = previousSubsystems(j);
        % 
        %             % Getting stored info from jInd to create \mathcal{A}_i and \mathcal{D}_i matrices (their j-th columns)
        %             tildeW_j = subsystems(jInd).dataToBeDistributed.tildeW;
        % 
        %             Z = zeros(blockSize, blockSize*(i-1-j)); % (i-1)-j blocks of blockSize X blockSize zero matrices
        %             z = zeros(blockSize, blockSize*(j-1));
        %             if j==1
        %                 tildeW_jj = tildeW_j;                    
        %                 scriptA_i = [tildeW_jj, Z];         % The first row of \mathcal{A}_i.
        %                 scriptD_i = [inv(tildeW_jj), Z];    % The first row of \mathcal{D}_i.
        %             else
        %                 tildeW_jj = tildeW_j(:,blockSize*(j-1)+1:blockSize*j);   % last blockSizeXblockSize block in the row block vector
        %                 tildeW_j  = tildeW_j(:,1:blockSize*(j-1));                % first (j-1) blocks in the row block vector
        %                 scriptA_i = [scriptA_i; [tildeW_j, tildeW_jj, Z]];    % The j-th row of \mathcal{A}_i.
        %                 scriptD_i = [scriptD_i; [z, inv(tildeW_jj), Z]];         % The j-th row of \mathcal{D}_i.
        %             end                    
        %         end
        %         disp(['Data at ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
        %         scriptA_i;
        %         scriptD_i;                
        % 
        %         M1_i = inv(scriptD_i*scriptA_i');
        %         % M_i = inv(M1_i*scriptD_i*M1_i') % THis fills (i-1)x(i-1) blocks in the LMI
        %         M_i = scriptA_i*scriptD_i*scriptA_i';
        % 
        %         if issymmetric(scriptD_i) & issymmetric(scriptA_i) & ~issymmetric(M_i)
        %             tf = norm(M_i-M_i.',inf);
        %             disp(['Symmetry Error !!! Magnitude:',num2str(tf)]);
        %             % M_i
        %             M_i = 0.5*(M_i + M_i');
        %         end
        % 
        % 
        %         % W_ii and W_i terms
        %         M_ii = sdpvar(n_i,n_i);
        %         L_ii = sdpvar(p_i,n_i,'full');
        %         W_ii = - M_ii*A_ii' - A_ii*M_ii - L_ii'*B_ii' - B_ii*L_ii;
        %         W_i = [];
        %         for j = 1:1:length(previousSubsystems)
        %             jInd = previousSubsystems(j);
        %             n_j = subsystems(jInd).dim_n;
        %             p_j = subsystems(jInd).dim_p;
        % 
        %             if any(subsystems(iInd).neighbors==jInd)
        %                 L_ij{j} = sdpvar(p_i,n_j,'full');
        %             else
        %                 L_ij{j} = zeros(p_i,n_j);
        %             end
        %             if any(subsystems(jInd).neighbors==iInd) 
        %                 L_ji{j} = sdpvar(p_j,n_i,'full');
        %             else
        %                 L_ji{j} = zeros(p_j,n_i);
        %             end
        % 
        %             A_ij = obj.A{jInd};
        %             A_ji = subsystems(jInd).A{iInd};
        %             B_jj  = subsystems(jInd).B{jInd};
        %             M_jj = subsystems(jInd).dataToBeDistributed.M;
        % 
        %             % W_ij = - M_ii A_ji^T - A_ij M_jj - L_ji^T B_jj^T - B_ii L_ij
        %             W_ij = - M_ii*A_ji' - A_ij*M_jj - L_ji{j}'*B_jj' - B_ii*L_ij{j};
        %             W_i = [W_i, W_ij];
        %         end
        % 
        %         con1 = M_ii >= 0.000000001*eye(n_i);
        %         con2 = [M_i, W_i';W_i, W_ii] >= 0;
        %         sol = optimize([con1,con2],[],solverOptions);
        %         isStabilizable = sol.problem==0;
        %         M_iiVal = value(M_ii); % This needs to be stored
        %         L_iiVal = value(L_ii);
        %         W_iVal = value(W_i);
        %         W_iiVal = value(W_ii);
        % 
        %         K_ii = L_iiVal/M_iiVal;
        %         obj.controllerGains.decenFSFStabCont{iInd} = K_ii;
        %         for j = 1:1:length(previousSubsystems)
        %             jInd = previousSubsystems(j);
        %             M_jj = subsystems(jInd).dataToBeDistributed.M;
        % 
        %             K_ij = value(L_ij{j})/M_jj;
        %             K_ji = value(L_ji{j})/M_iiVal;
        %             K_ijVals{jInd} = K_ij;
        %             obj.controllerGains.decenFSFStabCont{jInd} = K_ij;
        %             K_jiVals{jInd} = K_ji; % these values will be loaded outside the function
        %         end  
        % 
        %         % Need to compute \tilede{W}_i and \tilde{W}_{ii} for storage
        %         tildeW_i = W_iVal*M1_i;
        %         tildeW_ii = W_iiVal - tildeW_i*scriptD_i*tildeW_i'; % Note that here, \tilde{W}_ii, W_ii, \tilde{W}_i are different.
        % 
        %         disp(['Data saved ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
        %         tildeW_i = [tildeW_i, tildeW_ii];
        % 
        %         if abs(det(tildeW_ii))<0.000000001
        %             disp("Error: det(tildeW_ii) low");
        %             isStabilizable = 0;
        %         end
        % 
        %         obj.dataToBeDistributed.tildeW = tildeW_i; % Storing
        %         obj.dataToBeDistributed.M = M_iiVal; % Storing
        %         if ~isStabilizable
        %             disp(['LMI is not feasible at: ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
        %         end
        % 
        %     end
        % end


        % % % Dissipating state-feedback controller design
        % % At Network level
        % function [K, isDissipative] = decentralizedFSFDissipativation(obj,dissFrom,dissTo,indexing,solverOptions)
        % 
        %     if isempty(indexing)
        %         indexing = [1:1:length(obj.subsystems)];
        %     end
        % 
        %     K = [];
        %     for i = 1:1:length(indexing)
        %         iInd = indexing(i);
        %         previousSubsystems = indexing(1:i-1);                
        %         [isDissipative,K_ii,K_ijVals,K_jiVals] = obj.subsystems(iInd).FSFDissipativation(dissFrom,dissTo,previousSubsystems, obj.subsystems, solverOptions);
        % 
        %         K{iInd,iInd} = K_ii;
        %         obj.subsystems(iInd).controllerGains.decenFSFDissCont{iInd} = K_ii;
        %         for j = 1:1:length(previousSubsystems)
        %             jInd = previousSubsystems(j);
        %             obj.subsystems(jInd).controllerGains.decenFSFDissCont{iInd} = K_jiVals{jInd};
        %             obj.subsystems(iInd).controllerGains.decenFSFDissCont{jInd} = K_ijVals{jInd};
        % 
        %             K{iInd,jInd} = K_ijVals{jInd};
        %             K{jInd,iInd} = K_jiVals{jInd};
        %         end
        % 
        %         if ~isDissipative
        %             break
        %         end
        %     end
        % 
        %     if isDissipative
        %         Kmat = [];
        %         for i = 1:1:length(obj.subsystems)
        %             Karray = [];
        %             for j = 1:1:length(obj.subsystems)
        %                 Karray = [Karray, K{i,j}];
        %             end
        %             Kmat = [Kmat; Karray];
        %         end            
        %         K = Kmat;
        % 
        %     end
        % 
        %     if ~isDissipative
        %         K = zeros(size(obj.networkMatrices.C,1),size(obj.networkMatrices.C,2));
        %     end
        % 
        % end

        % % At subsystem level
        % function [isStabilizable,K_ii,K_ijVals,K_jiVals] = FSFDissipativation(obj, dissFrom, dissTo, previousSubsystems, subsystems, solverOptions)
        %     i = length(previousSubsystems)+1;
        %     iInd = obj.index;
        %     disp(['Dissipativating at: ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
        %     % W_ij = [W_1_ij; W_2_ij; W_3_ij]
        %     % W_1_ij = [A_ij*M_jj+B_ii*L_ij+M_ii*A_ji'+L_ji'*B_jj',  -E_ij+M_ii*C_ii'*S_ij,  M_ii*C_ii'*e_ij]  
        %     % W_2_ij = [-E_ji'+S_ji'*C_jj*M_jj,   F_ii'*S_ij+S_ji'*F_jj,   F_ii'*e_ij]
        %     % W_3_ij = [C_jj*M_jj*e_ij, F_jj*e_ij,  -inv(Q_ii)*E_ij]
        %     % L_ij is p_i x n_j and K_ij = L_ij M_jj^{-1}
        % 
        %     Q_ii = obj.dataToBeDistributed.Q{iInd};
        %     S_ii = obj.dataToBeDistributed.S{iInd};
        %     R_ii = obj.dataToBeDistributed.R{iInd};
        % 
        %     A_ii = obj.A{iInd};
        %     B_ii = obj.B{iInd};
        %     E_ii = obj.E{iInd};
        %     if isequal(dissFrom,'w') % the only possibility under FSF
        %         if isequal(dissTo,'y')
        %             C_ii = obj.C{iInd};
        %             F_ii = obj.F{iInd};
        %         elseif isequal(dissTo,'z')
        %             C_ii = obj.G{iInd};
        %             F_ii = obj.J{iInd};
        %         end
        %     end
        %     n_i = obj.dim_n;
        %     p_i = obj.dim_p; 
        % 
        % 
        %     if isempty(previousSubsystems)
        %         % The subsystem only need to test W_ii > 0  
        % 
        %         M_ii = sdpvar(n_i,n_i);
        %         L_ii = sdpvar(p_i,n_i,'full');
        % 
        %         if ~all(Q_ii(:)==0)
        %             % W_1_ij = [A_ij*M_jj+B_ii*L_ij+M_ii*A_ji'+L_ji'*B_jj',  -E_ij+M_ii*C_ii'*S_ij,  M_ii*C_ii'*e_ij]  
        %             W_1_ii = [A_ii*M_ii+B_ii*L_ii+M_ii*A_ii'+L_ii'*B_ii',  -E_ii+M_ii*C_ii'*S_ii,  M_ii*C_ii'];  
        %             % W_2_ij = [-E_ji'+S_ji'*C_jj*M_jj,   F_ii'*S_ij+S_ji'*F_jj+R_ij,   F_ii'*e_ij]
        %             W_2_ii = [-E_ii'+S_ii'*C_ii*M_ii,   F_ii'*S_ii+S_ii'*F_ii+R_ii,   F_ii'];
        %             % W_3_ij = [C_jj*M_jj*e_ij, F_jj*e_ij,  -inv(Q_ii)*E_ij]
        %             W_3_ii = [C_ii*M_ii, F_ii,  -inv(Q_ii)];
        %             % W_ij = [W_1_ij; W_2_ij; W_3_ij]
        %             W_ii = [W_1_ii; W_2_ii; W_3_ii];
        %         else
        %             % W_1_ij = [A_ij*M_jj+B_ii*L_ij+M_ii*A_ji'+L_ji'*B_jj',  -E_ij+M_ii*C_ii'*S_ij,  M_ii*C_ii'*e_ij]  
        %             W_1_ii = [A_ii*M_ii+B_ii*L_ii+M_ii*A_ii'+L_ii'*B_ii',  -E_ii+M_ii*C_ii'*S_ii];  
        %             % W_2_ij = [-E_ji'+S_ji'*C_jj*M_jj,   F_ii'*S_ij+S_ji'*F_jj+R_ij,   F_ii'*e_ij]
        %             W_2_ii = [-E_ii'+S_ii'*C_ii*M_ii,   F_ii'*S_ii+S_ii'*F_ii+R_ii];
        %             % W_ij = [W_1_ij; W_2_ij; W_3_ij]
        %             W_ii = [W_1_ii; W_2_ii];
        %         end
        % 
        %         con1 = M_ii >= 0;
        %         con2 = W_ii >= 0;
        %         sol = optimize([con1,con2],[],solverOptions);
        %         isStabilizable = sol.problem==0;
        % 
        %         M_iiVal = value(M_ii);
        %         L_iiVal = value(L_ii);
        %         W_iiVal = value(W_ii);
        %         tildeW_i = W_iiVal; % Note that, here, \tilde{W}_ii = W_ii = \tilde{W}_i. This also needs to be stored
        % 
        %         K_ii = L_iiVal/M_iiVal; % This needs to be stored
        %         K_jiVals = [];
        %         K_ijVals = [];
        % 
        %         obj.dataToBeDistributed.tildeW = tildeW_i; % Storing
        %         obj.dataToBeDistributed.M = M_iiVal; % Storing
        %         obj.controllerGains.decenFSFDissCont{iInd} = K_ii;
        % 
        %         disp(['Data saved ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
        %         if ~isStabilizable
        %             disp(['Not stabilizable at: ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
        %         end
        % 
        %     else
        %         % This subsystem has to talk with all the previosSubsystems
        %         % tildeW_ii > 0 iff [M_i, W_i'; W_i, W_ii] > 0 is required where 
        %         % M_i = scriptA1_i*scriptD1_i*scriptA1_i'; Also: 
        %         % M1_i = inv(scriptD1_i*scriptA1_i') and tildeW_i = W_i*M1_i;
        % 
        %         % M_i term
        %         if ~all(Q_ii(:)==0)
        %             blockSize = obj.dim_n + obj.dim_q + obj.dim_m; 
        %         else
        %             blockSize = obj.dim_n + obj.dim_q; 
        %         end
        %         scriptA_i = [];
        %         scriptD_i = [];
        %         for j = 1:1:length(previousSubsystems)
        %             jInd = previousSubsystems(j);
        % 
        %             % Getting stored info from jInd to create \mathcal{A}_i and \mathcal{D}_i matrices (their j-th columns)
        %             tildeW_j = subsystems(jInd).dataToBeDistributed.tildeW;
        % 
        %             Z = zeros(blockSize, blockSize*(i-1-j)); % (i-1)-j blocks of blockSize X blockSize zero matrices
        %             z = zeros(blockSize, blockSize*(j-1));
        %             if j==1
        %                 tildeW_jj = tildeW_j;                    
        %                 scriptA_i = [tildeW_jj, Z];         % The first row of \mathcal{A}_i.
        %                 scriptD_i = [inv(tildeW_jj), Z];    % The first row of \mathcal{D}_i.
        %             else
        %                 tildeW_jj = tildeW_j(:,blockSize*(j-1)+1:blockSize*j);   % last blockSizeXblockSize block in the row block vector
        %                 tildeW_j  = tildeW_j(:,1:blockSize*(j-1));                % first (j-1) blocks in the row block vector
        %                 scriptA_i = [scriptA_i; [tildeW_j, tildeW_jj, Z]];    % The j-th row of \mathcal{A}_i.
        %                 scriptD_i = [scriptD_i; [z, inv(tildeW_jj), Z]];         % The j-th row of \mathcal{D}_i.
        %             end                    
        %         end
        %         disp(['Data at ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
        %         scriptA_i;
        %         scriptD_i;                
        % 
        %         M1_i = inv(scriptD_i*scriptA_i');
        %         % M_i = inv(M1_i*scriptD_i*M1_i') % THis fills (i-1)x(i-1) blocks in the LMI
        %         M_i = scriptA_i*scriptD_i*scriptA_i';
        % 
        %         if issymmetric(scriptD_i) & issymmetric(scriptA_i) & ~issymmetric(M_i)
        %             tf = norm(M_i-M_i.',inf);
        %             disp(['Symmetry Error !!! Magnitude:',num2str(tf)]);
        %             % M_i
        %             M_i = 0.5*(M_i + M_i');
        %         end
        % 
        % 
        %         M_ii = sdpvar(n_i,n_i);
        %         L_ii = sdpvar(p_i,n_i,'full');
        %         % W_ii and W_i terms
        %         if ~all(Q_ii(:)==0)
        %             % W_1_ij = [A_ij*M_jj+B_ii*L_ij+M_ii*A_ji'+L_ji'*B_jj',  -E_ij+M_ii*C_ii'*S_ij,  M_ii*C_ii'*e_ij]  
        %             W_1_ii = [A_ii*M_ii+B_ii*L_ii+M_ii*A_ii'+L_ii'*B_ii',  -E_ii+M_ii*C_ii'*S_ii,  M_ii*C_ii'];  
        %             % W_2_ij = [-E_ji'+S_ji'*C_jj*M_jj,   F_ii'*S_ij+S_ji'*F_jj+R_ij,   F_ii'*e_ij]
        %             W_2_ii = [-E_ii'+S_ii'*C_ii*M_ii,   F_ii'*S_ii+S_ii'*F_ii+R_ii,   F_ii'];
        %             % W_3_ij = [C_jj*M_jj*e_ij, F_jj*e_ij,  -inv(Q_ii)*E_ij]
        %             W_3_ii = [C_ii*M_ii, F_ii,  -inv(Q_ii)];
        %             % W_ij = [W_1_ij; W_2_ij; W_3_ij]
        %             W_ii = [W_1_ii; W_2_ii; W_3_ii];
        %         else
        %             % W_1_ij = [A_ij*M_jj+B_ii*L_ij+M_ii*A_ji'+L_ji'*B_jj',  -E_ij+M_ii*C_ii'*S_ij,  M_ii*C_ii'*e_ij]  
        %             W_1_ii = [A_ii*M_ii+B_ii*L_ii+M_ii*A_ii'+L_ii'*B_ii',  -E_ii+M_ii*C_ii'*S_ii];  
        %             % W_2_ij = [-E_ji'+S_ji'*C_jj*M_jj,   F_ii'*S_ij+S_ji'*F_jj+R_ij,   F_ii'*e_ij]
        %             W_2_ii = [-E_ii'+S_ii'*C_ii*M_ii,   F_ii'*S_ii+S_ii'*F_ii+R_ii];
        %             % W_ij = [W_1_ij; W_2_ij; W_3_ij]
        %             W_ii = [W_1_ii; W_2_ii];
        %         end
        %         W_i = [];
        %         for j = 1:1:length(previousSubsystems)
        %             jInd = previousSubsystems(j);
        % 
        %             % Q_ij = obj.dataToBeDistributed.Q{jInd}; % not needed
        %             % Q_ji = subsystems(jInd).dataToBeDistributed.Q{iInd};
        %             S_ij = obj.dataToBeDistributed.S{jInd};
        %             S_ji = subsystems(jInd).dataToBeDistributed.S{iInd};
        %             R_ij = obj.dataToBeDistributed.R{jInd};
        %             % R_ji = subsystems(jInd).dataToBeDistributed.R{iInd}; % not needed
        % 
        %             % W_ij term
        %             n_j = subsystems(jInd).dim_n;
        %             p_j = subsystems(jInd).dim_p;
        %             if any(subsystems(iInd).neighbors==jInd)
        %                 L_ij{j} = sdpvar(p_i,n_j,'full');
        %             else
        %                 L_ij{j} = zeros(p_i,n_j);
        %             end
        %             if any(subsystems(jInd).neighbors==iInd) 
        %                 L_ji{j} = sdpvar(p_j,n_i,'full');
        %             else
        %                 L_ji{j} = zeros(p_j,n_i);
        %             end
        %             M_jj = subsystems(jInd).dataToBeDistributed.M;
        % 
        %             A_ij = obj.A{jInd};
        %             A_ji = subsystems(jInd).A{iInd};
        %             B_jj = subsystems(jInd).B{jInd};
        %             E_ij = obj.E{jInd};
        %             E_ji = subsystems(jInd).E{iInd};
        %             if isequal(dissFrom,'w') % the only possibility under FSF
        %                 if isequal(dissTo,'y')
        %                     C_jj = subsystems(jInd).C{jInd};
        %                     F_jj = subsystems(jInd).F{jInd};
        %                 elseif isequal(dissTo,'z')
        %                     C_jj = subsystems(jInd).G{jInd};
        %                     F_jj = subsystems(jInd).J{jInd};
        %                 end
        %             end
        % 
        %             if ~all(Q_ii(:)==0)
        %                 % W_1_ij = [A_ij*M_jj+B_ii*L_ij+M_ii*A_ji'+L_ji'*B_jj',  -E_ij+M_ii*C_ii'*S_ij,  M_ii*C_ii'*e_ij]  
        %                 W_1_ij = [A_ij*M_jj+B_ii*L_ij{j}+M_ii*A_ji'+L_ji{j}'*B_jj',  -E_ij+M_ii*C_ii'*S_ij,  M_ii*C_ii'*(i==j)];  
        %                 % W_2_ij = [-E_ji'+S_ji'*C_jj*M_jj,   F_ii'*S_ij+S_ji'*F_jj+R_ij,   F_ii'*e_ij]
        %                 W_2_ij = [-E_ji'+S_ji'*C_jj*M_jj,   F_ii'*S_ij+S_ji'*F_jj+R_ij,   F_ii'*(i==j)];
        %                 % W_3_ij = [C_jj*M_jj*e_ij, F_jj*e_ij,  -inv(Q_ii)*E_ij]
        %                 W_3_ij = [C_jj*M_jj*(i==j), F_jj*(i==j),  -inv(Q_ii)*(i==j)]
        %                 % W_ij = [W_1_ij; W_2_ij; W_3_ij]
        %                 W_ij = [W_1_ij; W_2_ij; W_3_ij];
        %             else
        %                 % W_1_ij = [A_ij*M_jj+B_ii*L_ij+M_ii*A_ji'+L_ji'*B_jj',  -E_ij+M_ii*C_ii'*S_ij,  M_ii*C_ii'*e_ij]  
        %                 W_1_ij = [A_ij*M_jj+B_ii*L_ij{j}+M_ii*A_ji'+L_ji{j}'*B_jj',  -E_ij+M_ii*C_ii'*S_ij];  
        %                 % W_2_ij = [-E_ji'+S_ji'*C_jj*M_jj,   F_ii'*S_ij+S_ji'*F_jj+R_ij,   F_ii'*e_ij]
        %                 W_2_ij = [-E_ji'+S_ji'*C_jj*M_jj,   F_ii'*S_ij+S_ji'*F_jj+R_ij];
        %                 % W_ij = [W_1_ij; W_2_ij; W_3_ij]
        %                 W_ij = [W_1_ij; W_2_ij];
        %             end
        % 
        %             W_i = [W_i, W_ij];
        %         end
        % 
        %         con1 = M_ii >= 0;
        %         con2 = [M_i, W_i';W_i, W_ii] >= 0;
        %         sol = optimize([con1,con2],[],solverOptions);
        %         isStabilizable = sol.problem==0;
        %         M_iiVal = value(M_ii); % This needs to be stored
        %         L_iiVal = value(L_ii);
        %         W_iVal = value(W_i);
        %         W_iiVal = value(W_ii);
        % 
        %         K_ii = L_iiVal/M_iiVal;
        %         obj.controllerGains.decenFSFDissCont{iInd} = K_ii;
        % 
        %         for j = 1:1:length(previousSubsystems)
        %             jInd = previousSubsystems(j);
        %             M_jj = subsystems(jInd).dataToBeDistributed.M;
        % 
        %             K_ij = value(L_ij{j})/M_jj;
        %             K_ji = value(L_ji{j})/M_iiVal;
        %             K_ijVals{jInd} = K_ij;
        %             obj.controllerGains.decenFSFStabCont{jInd} = K_ij;
        %             K_jiVals{jInd} = K_ji; % these values will be loaded outside the function
        %         end  
        % 
        %         % Need to compute \tilede{W}_i and \tilde{W}_{ii} for storage
        %         tildeW_i = W_iVal*M1_i;
        %         tildeW_ii = W_iiVal - tildeW_i*scriptD_i*tildeW_i'; % Note that here, \tilde{W}_ii, W_ii, \tilde{W}_i are different.
        % 
        %         disp(['Data saved ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
        %         tildeW_i = [tildeW_i, tildeW_ii];
        % 
        %         obj.dataToBeDistributed.tildeW = tildeW_i; % Storing
        %         obj.dataToBeDistributed.M = M_iiVal; % Storing
        %         if ~isStabilizable
        %             disp(['LMI is not feasible at: ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
        %         end
        % 
        %     end
        % end

    end
end