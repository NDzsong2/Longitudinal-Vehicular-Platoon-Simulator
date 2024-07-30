classdef Platoon < handle
    %CHAIN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Indices
        platoonIndex            % k
        
        % parameters
        numOfVehicles           % n
        
        vehicles = []           % this array holds all the vehicle class objects created. 
        topology
       
        graphics1 = []
        graphics2 = []

        K % controller gains obtained 
        R

        % K sequence
        KSequence
    end
    
    methods

        function obj = Platoon(k,n_k,parameters,states,desiredSeparation,noiseMean,noiseStd)

            obj.platoonIndex = k;
            obj.numOfVehicles = n_k;
            
            % Create the vehicles
            vehicles = [];
            for i = 1:1:n_k
                % create an object from the Vehicle class
                vehicle = Vehicle(k,i,parameters(:,i),states(:,i),desiredSeparation(:,i),noiseMean(:,i),noiseStd(:,i));
                vehicles = [vehicles, vehicle];
            end
            obj.vehicles = vehicles;

            obj.topology = Topology(n_k); % Generate a topology
            obj.updateNeighbors();   % update the neighbor information of each vehicle object inside obj.vehicles based on obj.topology
            obj.loadDefaultControllerGains(); % based on the neighbor connections, load some controller gains

        end
        

        function outputArg = updateNeighbors(obj)
            for i = 1:1:obj.numOfVehicles
                obj.vehicles(i).inNeighbors = obj.topology.inNeighbors{i};
                obj.vehicles(i).outNeighbors = obj.topology.outNeighbors{i};

                desiredSeperations = []; % to store d_ij values
                for j = 1:1:obj.numOfVehicles
                    d_ij = obj.vehicles(i).desiredSeparation - obj.vehicles(j).desiredSeparation;
                    desiredSeperations = [desiredSeperations, d_ij];
                end
                obj.vehicles(i).desiredSeparations = desiredSeperations;
            end
        end

        
        function outputArg = loadDefaultControllerGains(obj)

            for i = 2:1:obj.numOfVehicles


                % In error dynamics formulation - I
                L_ii = -[5,5,5]; 
                obj.vehicles(i).localControllerGains1 = L_ii/2;
                obj.vehicles(i).controllerGains1{i} = L_ii/2;
                    

                % In error dynamics formulation - II
                L_ii = -[5,5,5];
                obj.vehicles(i).localControllerGains2 = L_ii/2;
                obj.vehicles(i).controllerGains2{i} = L_ii/2;

                for jInd = 1:1:length(obj.vehicles(i).inNeighbors)
                    j = obj.vehicles(i).inNeighbors(jInd);

                    % In error dynamics formulation - I, these are \bar{k}_{ij} values
                    % \bar{k}_{ij}\in\R and exist for all j\in\bar{\N}_N/{i}
                    k_ijBar = -1;
                    obj.vehicles(i).controllerGains1{j} = k_ijBar;
                    
                    % In error dynamics formulation - II, these are L_{ij} values
                    % L_{ij}\in\R^3 and exist for all j\in\N_N/{i}
                    if j ~= 1 
                        L_ij = -[1,1,1];
                        obj.vehicles(i).controllerGains2{j} = L_ij;
                    end                   

                end
            end
        end


        function outputArg = loadPassivityIndices(obj,nuVals,rhoVals)
            for i = 2:1:obj.numOfVehicles
                obj.vehicles(i).loadPassivityIndices(nuVals(i-1),rhoVals(i-1));
            end
        end


        function outputArg = drawPlatoon(obj,figNum)
            figure(figNum); hold on; 

            % Draw the vehicles
            for i = 1:1:obj.numOfVehicles
                obj.vehicles(i).drawVehicle(figNum);
            end
            
            % use the obj.function() to add the topology (Because this is the layer that we can get access to the vehicle class)
            obj.drawTopology(figNum) 
        end


        % Draw the topology for a fixed graph
        function outputArg = drawTopology(obj,figNum)
            figure(figNum); hold on;

            if ~isempty(obj.graphics1)
                delete(obj.graphics1);
                delete(obj.graphics2);
            end

            numOfLinks = length(obj.topology.startNodes);
            for i = 1:1:numOfLinks
                % Draw a link
                startVehicleIndex = obj.topology.startNodes(i);
                endVehicleIndex = obj.topology.endNodes(i);

                startPos = obj.vehicles(startVehicleIndex).states(1) - obj.vehicles(startVehicleIndex).vehicleParameters(2)/2;
                endPos = obj.vehicles(endVehicleIndex).states(1) - obj.vehicles(endVehicleIndex).vehicleParameters(2)/2;
                midPos = (startPos + endPos)/2;
                midPointHeight = -5*sign(startPos-endPos)+0.05*abs(startPos-endPos)-0.5*(startPos<endPos); % 4

                startPosY = obj.vehicles(startVehicleIndex).vehicleParameters(2)*3/8;
                endPosY = obj.vehicles(endVehicleIndex).vehicleParameters(2)*3/8;
                %obj.graphics(i) = plot([startPos,midPos,endPos],[startPosY,midPointHeight,endPosY],'-b');

                % Plotting the Spline
                x = [startPos,midPos,endPos];
                y = [startPosY,midPointHeight,endPosY];
                stepSize = (endPos-startPos)/20; 
                xx = startPos:stepSize:endPos;
                yy = spline(x,y,xx);
                obj.graphics1(i) = plot(xx,yy,'-b');
            
                % Plotting the arrowHead (polyshape)
                polyPosX = midPos;
                polyPosY = midPointHeight;
                polySize = 0.3;
                polyVertX = [-0.5,1,-0.5];
                polyVertY = [0.5,0,-0.5];
                if polyPosY < 0
                    polyVertX = -polyVertX;
                end
                arrowHead = polyshape(polyPosX+polySize*polyVertX,polyPosY+polySize*polyVertY);
                obj.graphics2(i) = plot(arrowHead,'EdgeColor','k','FaceColor','b');
            end            
        end


        function outputArg = redrawPlatoon(obj,figNum)
            figure(figNum); hold on; 
                        
            % Redraw the vehicles
            for i = 1:1:obj.numOfVehicles
                obj.vehicles(i).redrawVehicle(figNum);
            end
            
            % Redraw the topology by calling redrawTopology method, based on the newly updated states of the vehicles
            obj.redrawTopology(figNum) 
        end


        % Redraw the topology for a fixed graph
        function outputArg = redrawTopology(obj,figNum)
            figure(figNum); hold on;
                 
            numOfLinks = length(obj.topology.startNodes);
            
            for i = 1:1:numOfLinks
                if ~isempty(obj.graphics1(i))
                    delete(obj.graphics1(i));
                    delete(obj.graphics2(i));

                    % Redraw a link
                    startVehicleIndex = obj.topology.startNodes(i);
                    endVehicleIndex = obj.topology.endNodes(i);
                    
                    startPos = obj.vehicles(startVehicleIndex).states(1) - obj.vehicles(startVehicleIndex).vehicleParameters(2)/2;
                    endPos = obj.vehicles(endVehicleIndex).states(1) - obj.vehicles(endVehicleIndex).vehicleParameters(2)/2;
                    midPos = (startPos + endPos)/2;
                    midPointHeight = -5*sign(startPos-endPos)+0.05*abs(startPos-endPos)-0.5*(startPos<endPos); % 4
                    
                    startPosY = obj.vehicles(startVehicleIndex).vehicleParameters(2)*3/8;
                    endPosY = obj.vehicles(endVehicleIndex).vehicleParameters(2)*3/8;
                    % obj.graphics1(i) = plot([startPos,midPos,endPos],[0,4,0]+[startPosY,0,endPosY],'-b');
                    % obj.graphics1(i) = plot([startPos,midPos,endPos],[startPosY,midPointHeight,endPosY],'-b');

                    % Plotting the Spline
                    x = [startPos,midPos,endPos];
                    y = [startPosY,midPointHeight,endPosY];
                    stepSize = (endPos-startPos)/20; 
                    xx = startPos:stepSize:endPos;
                    yy = spline(x,y,xx);
                    obj.graphics1(i) = plot(xx,yy,'-b');
                
                    % Plotting the arrowHead (polyshape)
                    polyPosX = midPos;
                    polyPosY = midPointHeight;
                    polySize = 0.3;
                    polyVertX = [-0.5,1,-0.5];
                    polyVertY = [0.5,0,-0.5];
                    if polyPosY < 0
                        polyVertX = -polyVertX;
                    end
                    arrowHead = polyshape(polyPosX+polySize*polyVertX,polyPosY+polySize*polyVertY);
                    obj.graphics2(i) = plot(arrowHead,'EdgeColor','k','FaceColor','b');
                end
            end            
        end


        function outputArg = generateNoises(obj)
            for i = 1:1:obj.numOfVehicles
                obj.vehicles(i).generateNoise();
            end
        end


        function outputArg = computePlatooningErrors1(obj)
            leaderStates = obj.vehicles(1).states;  % x_0,v_0,a_0
            for i = 2:1:obj.numOfVehicles
                neighborInformation = [];
                for jInd = 1:1:length(obj.vehicles(i).inNeighbors)
                    j = obj.vehicles(i).inNeighbors(jInd);
                    neighborInformation{j} = obj.vehicles(j).states;
                end 
                obj.vehicles(i).computePlatooningErrors1(leaderStates,neighborInformation);
            end
        end


        function outputArg = computePlatooningErrors2(obj)
            leaderStates = obj.vehicles(1).states;  % x_0,v_0,a_0
            for i = 2:1:obj.numOfVehicles
                obj.vehicles(i).computePlatooningErrors2(leaderStates);
            end
        end


        function outputArg = computeControlInputs1(obj,t)
            for i = 1:1:obj.numOfVehicles
                obj.vehicles(i).computeControlInputs1(t);
            end
        end


        function outputArg = computeControlInputs2(obj,t)
            for i = 1:1:obj.numOfVehicles
                neighborInformation = [];
                for jInd = 1:1:length(obj.vehicles(i).inNeighbors)
                    j = obj.vehicles(i).inNeighbors(jInd);
                    neighborInformation{j} = obj.vehicles(j).errors;
                end 
                obj.vehicles(i).computeControlInputs2(t,neighborInformation);
            end
        end


        function totalError = update(obj,t,dt)
            totalError = zeros(3,1);
            for i = 1:1:obj.numOfVehicles
                vehicleError = obj.vehicles(i).update(t,dt);
                totalError = totalError + vehicleError;
            end
        end

        
        %% Stabilizing Controller Synthesis Using Error Dynamics Formulation I

        % Centralized Stabilizing Controller Synthesis (Error Dynamics I) 
        function status =  centralizedStabilizingControllerSynthesis1(obj,nuBar,rhoBar)
            % This is a very simple stabilizing controller synthesized
            % based on the error dynamics formulation 1.

            % Number of follower vehicles
            N = obj.numOfVehicles - 1; 
            
            % Whether to use a soft or hard graph constraint
            isSoft = 1;
            normType = 2;
            
            % Load local controllers/passivity indices
            for i = 1:1:N
                status = obj.vehicles(i+1).synthesizeLocalControllers(1,nuBar,rhoBar);
            end            
            
            % Creating the adgacency matrix, null matrix and cost matrix
            G = obj.topology.graph;
            A = adjacency(G);
            for i = 1:1:N
                for j = 1:1:N
                    % Structure of K_ij (which is a 3x3 matrix) should be embedded here
                    if i~=j
                        if A(j+1,i+1)==1
                            adjMatBlock{i,j} = [0,0,0; 0,0,1; 0,0,0];
                            nullMatBlock{i,j} = [1,1,1; 1,1,0; 1,1,1];
                            costMatBlock{i,j} = 0.01*[0,0,0; 0,0,1; 0,0,0];
                        else
                            adjMatBlock{i,j} = [0,0,0; 0,0,0; 0,0,0];
                            nullMatBlock{i,j} = [1,1,1; 1,1,0; 1,1,1];
                            costMatBlock{i,j} = 1*[0,0,0; 0,0,1; 0,0,0];
                        end
                    else
                        adjMatBlock{i,j} = [0,0,0; 0,0,1; 1,1,1];
                        nullMatBlock{i,j} = [1,1,1; 1,1,0; 0,0,0];
                        costMatBlock{i,j} = 1*[0,0,0; 0,0,1; 1,1,1];
                    end
                end 
            end
            adjMatBlock = cell2mat(adjMatBlock);
            nullMatBlock = cell2mat(nullMatBlock);
            costMatBlock = cell2mat(costMatBlock);
            
            % Set up the LMI problem
            solverOptions = sdpsettings('solver','mosek','verbose',0);
            I_n = eye(3);
            O = zeros(3*N);
            
            Q = sdpvar(3*N,3*N,'full'); 
            P = sdpvar(N,N,'diagonal');
            
            X_p_11 = [];
            X_p_12 = [];
            X_12 = [];
            X_p_22 = [];
            for i = 1:1:N
                nu_i = obj.vehicles(i+1).nu;
                rho_i = obj.vehicles(i+1).rho;
            
                X_p_11 = blkdiag(X_p_11,-nu_i*P(i,i)*I_n);
                X_p_12 = blkdiag(X_p_12,0.5*P(i,i)*I_n);
                X_12 = blkdiag(X_12,(-1/(2*nu_i))*I_n);
                X_p_22 = blkdiag(X_p_22,-rho_i*P(i,i)*I_n);
            end
            X_p_21 = X_p_12';
            X_21 = X_12';
            
            % Objective Function
            costFun = norm(Q.*costMatBlock,normType) + 0*norm(Q.*nullMatBlock,normType);
            
            % Budget Constraints
            % con01 = costFun <= 1;
            
            % Basic Constraints
            con1 = P >= 0;
            %%con2 = trace(P) == 1;
            
            DMat = [X_p_11];
            MMat = [Q];
            ThetaMat = [-X_21*Q-Q'*X_12-X_p_22];
            con3 = [DMat, MMat; MMat', ThetaMat] >= 0;
            
            % Structural constraints
            con4 = Q.*(nullMatBlock==1)==O;  % Structural limitations (due to the format of the control law)
            con5 = Q.*(adjMatBlock==0)==O;   % Graph structure : hard constraint
            
            
            % Total Cost and Constraints
            if isSoft
                cons = [con1,con3,con4]; % Without the hard graph constraint con7
                costFun = 1*costFun; % soft 
            else
                cons = [con1,con3,con4,con5]; % With the hard graph constraint con7
                costFun = 1*costFun; % hard (same as soft)
            end
            
            sol = optimize(cons,[costFun],solverOptions);
            status = sol.problem == 0; %sol.info;
            
            costFunVal = value(costFun);
            PVal = value(P);
            QVal = value(Q);

            % What we had originally: (perhaps because \nu = 0 was assumed ? - check)
%             X_p_21Val = value(X_p_21);
%             X_p_22Val = value(X_p_22);
%             M_neVal = X_p_21Val\QVal;

            % What I guess we need to have hear (when nu\neq 0) 
            X_p_11Val = value(X_p_11);
            X_p_21Val = value(X_p_21);
            M_neVal = X_p_11Val\QVal;
            
            % Obtaining K_ij blocks (Blocking and Filtering)
            M_neVal(nullMatBlock==1) = 0;
            maxNorm = 0;
            for i = 1:1:N
                for j = 1:1:N
                    K{i,j} = M_neVal(3*(i-1)+1:3*i , 3*(j-1)+1:3*j); % (i,j)-th (3 x 3) block
                    normVal = max(max(abs(K{i,j})));
                    if normVal>maxNorm 
                        maxNorm = normVal;
                    end
                end
            end
            
            % filtering out extremely small interconnections
            for i=1:1:N
                for j=1:1:N
                    if i~=j
                        if isSoft
                            K{i,j}(abs(K{i,j})<0.000001*maxNorm) = 0;                       
                        else
                            if A(j+1,i+1)==0
                                K{i,j} = zeros(3);
                            end
                        end
                    end        
                end
            end
      
            % Updating the stored variables in the Platoon class and 
            % Vehicle class objects based on the synthesized K
            % (i.e., the interconnection matrix)
            K
            obj.loadTopologyFromK1(K);
            obj.loadControllerGains1(K);

        end


        % Decentralized Stabilizing Controller Synthesis (Error Dynamics I) - Lets start with this as it is the simplest form!
        function status = decentralizedStabilizingControllerSynthesis1(obj,indexing,nuBar,rhoBar)

            if isempty(indexing)
                indexing = [1:1:(length(obj.vehicles)-1)];
            end
        
            for i = 1:1:length(indexing)
                iInd = indexing(i);
                previousSubsystems = indexing(1:i-1);                
                [isStabilizable,K_ii,K_ijVals,K_jiVals] = obj.vehicles(iInd).stabilizingControllerSynthesis1(previousSubsystems, obj.vehicles, nuBar, rhoBar);

                K{iInd,iInd} = K_ii;
                for j = 1:1:length(previousSubsystems)
                    jInd = previousSubsystems(j);
                    K{iInd,jInd} = K_ijVals{jInd};
                    K{jInd,iInd} = K_jiVals{jInd};

                    %% We need to update K_{jj} values here!!! K_{j0} = k_{j0} + k_{ji}
                    disp([' Local controller updated at j = ',num2str(j)])
                    obj.vehicles(jInd+1).localControllerGains1 = obj.vehicles(jInd+1).localControllerGains1 + K{jInd,iInd}(3,:);
                end
                obj.KSequence{iInd} = K;
                disp(['Current K at i = ',num2str(iInd)])
                disp(cell2mat(K))
        
                if ~isStabilizable
                    break
                end
            end
        
            if isStabilizable
                status = 1;
                obj.loadTopologyFromK1(K); 
                obj.loadControllerGains1(K);
            else
                status = 0;
            end

        end


        %% Robust Controller Synthesis Using Error Dynamics Formulation I

        % Centralized Robust Controller Synthesis (Error Dynamics Formulation I)
        function status = centralizedRobustControllerSynthesis1(obj,nuBar,rhoBar,gammaSqBar)
            
            % Number of follower vehicles
            N = obj.numOfVehicles-1; 

            % Load local controllers and resulting passivity indices
            for i = 1:1:N
                status = obj.vehicles(i+1).synthesizeLocalControllers(1,nuBar,rhoBar);
            end

%             %% Passivity indices type
%             passivityInfoType = 2;
%             
%             if passivityInfoType==1
%                 % Load Passivity Indices - I
%                 tau = -sqrt(2)/4;
%                 nuVals = tau*ones(N,1);
%                 rhoVals = tau*ones(N,1);
%                 obj.loadPassivityIndices(nuVals,rhoVals);
%             else
%                 % Load Passivity Indices - II
%                 rho = -1/2;
%                 nuVals = 0*ones(N,1);
%                 rhoVals = rho*ones(N,1);
%                 obj.loadPassivityIndices(nuVals,rhoVals);
%             end

            % Creating the adgacency matrix, null matrix and cost matrix
            G = obj.topology.graph;
            A = adjacency(G);
            for i = 1:1:N
                for j = 1:1:N
                    % Structure of K_ij (which is a 3x3 matrix) should be embedded here
                    if i~=j
                        if A(j+1,i+1)==1
                            adjMatBlock{i,j} = [0,0,0; 0,0,1; 0,0,0];
                            nullMatBlock{i,j} = [1,1,1; 1,1,0; 1,1,1];
                            costMatBlock{i,j} = 0.01*[0,0,0; 0,0,1; 0,0,0];
                        else
                            adjMatBlock{i,j} = [0,0,0; 0,0,0; 0,0,0];
                            nullMatBlock{i,j} = [1,1,1; 1,1,0; 1,1,1];
                            costMatBlock{i,j} = 1*[0,0,0; 0,0,1; 0,0,0];
                        end
                    else
                        adjMatBlock{i,j} = [0,0,0; 0,0,1; 1,1,1];
                        nullMatBlock{i,j} = [1,1,1; 1,1,0; 0,0,0];
                        costMatBlock{i,j} = 1*[0,0,0; 0,0,1; 1,1,1];
                    end
                end 
            end
            adjMatBlock = cell2mat(adjMatBlock);
            nullMatBlock = cell2mat(nullMatBlock);
            costMatBlock = cell2mat(costMatBlock);

            % Set up the LMI problem
            solverOptions = sdpsettings('solver','mosek');
            I = eye(3*N);
            I_n = eye(3);
            O = zeros(3*N);
            
            % Whether to use a soft or hard graph constraint
            isSoft = 1;
            normType = 2;

            Q = sdpvar(3*N,3*N,'full'); 
            P = sdpvar(N,N,'diagonal');
            gammaSq = sdpvar(1);
            
            X_p_11 = [];
            X_p_12 = [];
            X_12 = [];
            X_p_22 = [];
            for i = 1:1:N
                nu_i = obj.vehicles(i+1).nu;
                rho_i = obj.vehicles(i+1).rho;

                X_p_11 = blkdiag(X_p_11,-nu_i*P(i,i)*I_n);
                X_p_12 = blkdiag(X_p_12,0.5*P(i,i)*I_n);
                X_12 = blkdiag(X_12,(-1/(2*nu_i))*I_n);
                X_p_22 = blkdiag(X_p_22,-rho_i*P(i,i)*I_n);
            end
            X_p_21 = X_p_12';
            X_21 = X_12';
            
            % Objective Function
            costFun = norm(Q.*costMatBlock,normType) + 0*norm(Q.*nullMatBlock,normType);

            % Budget Constraints (Not at this stage)
            % con01 = costFun <= 1;

            % Basic Constraints
            con1 = P >= 0;
            %%con2 = trace(P) == 1;
            con3 = gammaSq >= 0;
            
            DMat = [X_p_11, O; O, I];
            MMat = [Q, X_p_11; I, O];
            ThetaMat = [-X_21*Q-Q'*X_12-X_p_22, -X_p_21; -X_p_12, gammaSq*I];
            con4 = [DMat, MMat; MMat', ThetaMat] >= 0; % The real one
            
            % LMI Conditions
%             if passivityInfoType==1
%                 % LMI Condition - I
%                 DMat = [X_p_11, O; O, I];
%                 MMat = [Q, X_p_11; I, O];
%                 ThetaMat = [-X_21*Q-Q'*X_12-X_p_22, -X_21*X_p_11; -X_p_11'*X_12, gammaSq*I];
%                 con4 = [DMat, MMat; MMat', ThetaMat] >= 0;
%             else    
%                 % LMI Condition - II (when nu_i = 0, for all i)
%                 DMat = [I];
%                 MMat = [I, O];
%                 ThetaMat = [-Q-Q'-X_p_22, -X_p_21; -X_p_12, gammaSq*I];
%                 con4 = [DMat, MMat; MMat', ThetaMat] >= 0;
%             end

            % Structural constraints
            con5 = Q.*(nullMatBlock==1)==O;  % Structural limitations (due to the format of the control law)
            con6 = Q.*(adjMatBlock==0)==O;  % Graph structure : hard constraint

            con7 = gammaSq <= gammaSqBar;

            % Total Cost and Constraints
            if isSoft
                cons = [con1,con3,con4,con5,con7]; % Without the hard graph constraint con7
                costFun = 1*costFun + 1*gammaSq; % soft 
            else
                cons = [con1,con3,con4,con5,con6,con7]; % With the hard graph constraint con7
                costFun = 1*costFun + 1*gammaSq; % hard (same as soft)
            end
            
            sol = optimize(cons,[costFun],solverOptions);
            status = sol.problem == 0; %sol.info;
            
            costFunVal = value(costFun);
            PVal = value(P)
            QVal = value(Q);
            X_p_11Val = value(X_p_11);
            X_p_21Val = value(X_p_21);

            gammaSqVal = value(gammaSq)

            M_neVal = X_p_11Val\QVal;
            
%             if passivityInfoType==1
%                 M_neVal = X_p_11Val\QVal
%             else
%                 M_neVal = X_p_21Val\QVal
%             end
            
            % Obtaining K_ij blocks
            M_neVal(nullMatBlock==1) = 0;
            maxNorm = 0;
            for i = 1:1:N
                for j = 1:1:N
                    K{i,j} = M_neVal(3*(i-1)+1:3*i , 3*(j-1)+1:3*j); % (i,j)-th (3 x 3) block
                    normVal = max(max(abs(K{i,j})));
                    if normVal>maxNorm
                        maxNorm = normVal;
                    end
                end
            end
            
            % filtering out small interconnections
            for i=1:1:N
                for j=1:1:N
                    if i~=j
                        if isSoft
                            K{i,j}(abs(K{i,j})<0.000001*maxNorm) = 0;                       
                        else
                            if A(j+1,i+1)==0
                                K{i,j} = zeros(3);
                            end
                        end
                    end
                    
                end
            end
            
            K
            obj.loadTopologyFromK1(K);
            obj.loadControllerGains1(K);

        end

        % Decentralized Robust Controller Synthesis (Error Dynamics Formulation I)
        function status = decentralizedRobustControllerSynthesis1(obj,indexing,nuBar,rhoBar,gammaSqBar)
            
            if isempty(indexing)
                indexing = [1:1:(length(obj.vehicles)-1)];
            end
        
            for i = 1:1:length(indexing)
                iInd = indexing(i);
                previousSubsystems = indexing(1:i-1);                
                [isRobustStabilizable,K_ii,K_ijVals,K_jiVals] = obj.vehicles(iInd).robustControllerSynthesis1(previousSubsystems, obj.vehicles, nuBar, rhoBar, gammaSqBar);

                K{iInd,iInd} = K_ii;
                for j = 1:1:length(previousSubsystems)
                    jInd = previousSubsystems(j);
                    K{iInd,jInd} = K_ijVals{jInd};
                    K{jInd,iInd} = K_jiVals{jInd};

                    %% We need to update K_{jj} values here!!! K_{j0} = k_{j0} + k_{ji}
                    disp([' Local controller updated at j = ',num2str(j)])
                    obj.vehicles(jInd+1).localControllerGains1 = obj.vehicles(jInd+1).localControllerGains1 + K{jInd,iInd}(3,:);
                end
                obj.KSequence{iInd} = K;
                disp(['Current K at i = ',num2str(iInd)])
                disp(cell2mat(K))
        
                if ~isRobustStabilizable
                    break
                end
            end
        
            if isRobustStabilizable
                status = 1;
                obj.loadTopologyFromK1(K); 
                obj.loadControllerGains1(K);
            else
                status = 0;
            end
            
        end 


        %% Stabilizing Controller Synthesis Using Error Dynamics Formulation II

        % Centralized Stabilizing Controller Synthesis (Error Dynamics II) 
        function status = centralizedStabilizingControllerSynthesis2(obj,nuBar,rhoBar)
            % This is a very simple stabilizing controller synthesized
            % based on the error dynamics formulation 2.

            % Number of follower vehicles
            N = obj.numOfVehicles-1; 
            
            % Whether to use a soft or hard graph constraint
            isSoft = 1;
            normType = 2;
            
            % Load local controllers/passivity indices
            for i = 1:1:N
                status = obj.vehicles(i+1).synthesizeLocalControllers(2,nuBar,rhoBar);
            end            
            
            % Creating the adgacency matrix, null matrix and cost matrix
            G = obj.topology.graph;
            A = adjacency(G);
            for i = 1:1:N
                for j = 1:1:N
                    % Structure of K_ij (which is a 3x3 matrix) should be embedded here
                    if i~=j
                        if A(j+1,i+1)==1
                            adjMatBlock{i,j} = [0,0,0; 0,0,0; 1,1,1];
                            nullMatBlock{i,j} = [1,1,1; 1,1,1; 0,0,0];
                            costMatBlock{i,j} = 0.01*[0,0,0; 0,0,0; 1,1,1];
                        else
                            adjMatBlock{i,j} = [0,0,0; 0,0,0; 0,0,0];
                            nullMatBlock{i,j} = [1,1,1; 1,1,1; 0,0,0];
                            costMatBlock{i,j} = 1*[0,0,0; 0,0,0; 1,1,1];
                        end
                    else
                        adjMatBlock{i,j} = [0,0,0; 0,0,0; 1,1,1];
                        nullMatBlock{i,j} = [1,1,1; 1,1,1; 0,0,0];
                        costMatBlock{i,j} = 1*[0,0,0; 0,0,0; 1,1,1];
                    end
                end 
            end
            adjMatBlock = cell2mat(adjMatBlock);
            nullMatBlock = cell2mat(nullMatBlock);
            costMatBlock = cell2mat(costMatBlock);
            
            % Set up the LMI problem
            solverOptions = sdpsettings('solver','mosek','verbose',0);
            I_n = eye(3);
            O = zeros(3*N);
            
            Q = sdpvar(3*N,3*N,'full'); 
            P = sdpvar(N,N,'diagonal');
            
            X_p_11 = [];
            X_p_12 = [];
            X_12 = [];
            X_p_22 = [];
            for i = 1:1:N
                nu_i = obj.vehicles(i+1).nu;
                rho_i = obj.vehicles(i+1).rho;
            
                X_p_11 = blkdiag(X_p_11,-nu_i*P(i,i)*I_n);
                X_p_12 = blkdiag(X_p_12,0.5*P(i,i)*I_n);
                X_12 = blkdiag(X_12,(-1/(2*nu_i))*I_n);
                X_p_22 = blkdiag(X_p_22,-rho_i*P(i,i)*I_n);
            end
            X_p_21 = X_p_12';
            X_21 = X_12';
            
            % Objective Function
            costFun = norm(Q.*costMatBlock,normType) + 0*norm(Q.*nullMatBlock,normType);
            
            % Budget Constraints
            % con01 = costFun <= 1;
            
            % Basic Constraints
            con1 = P >= 0;
            %%con2 = trace(P) == 1;
            
            DMat = [X_p_11];
            MMat = [Q];
            ThetaMat = [-X_21*Q-Q'*X_12-X_p_22];
            con3 = [DMat, MMat; MMat', ThetaMat] >= 0;
            
            % Structural constraints
            con4 = Q.*(nullMatBlock==1)==O;  % Structural limitations (due to the format of the control law)
            con5 = Q.*(adjMatBlock==0)==O;  % Graph structure : hard constraint
            
            
            % Total Cost and Constraints
            if isSoft
                cons = [con1,con3,con4]; % Without the hard graph constraint con7
                costFun = 1*costFun; % soft 
            else
                cons = [con1,con3,con4,con5]; % With the hard graph constraint con7
                costFun = 1*costFun; % hard (same as soft)
            end
            
            sol = optimize(cons,[costFun],solverOptions);
            status = sol.problem == 0; %sol.info;
            
            costFunVal = value(costFun);
            PVal = value(P);
            QVal = value(Q);

            % What we had originally: (perhaps because \nu = 0 was assumed ? - check)
%             X_p_21Val = value(X_p_21);
%             X_p_22Val = value(X_p_22);
%             M_neVal = X_p_21Val\QVal;

            % What I guess we need to have hear (when nu\neq 0) 
            X_p_11Val = value(X_p_11);
            X_p_21Val = value(X_p_21);
            M_neVal = X_p_11Val\QVal;
            
            % Obtaining K_ij blocks
            M_neVal(nullMatBlock==1) = 0;
            maxNorm = 0;
            for i = 1:1:N
                for j = 1:1:N
                    K{i,j} = M_neVal(3*(i-1)+1:3*i , 3*(j-1)+1:3*j); % (i,j)-th (3 x 3) block
                    normVal = max(max(abs(K{i,j})));
                    if normVal>maxNorm 
                        maxNorm = normVal;
                    end
                end
            end
            
            % filtering out extremely small interconnections
            for i=1:1:N
                for j=1:1:N
                    if i~=j
                        if isSoft
                            K{i,j}(abs(K{i,j})<0.000001*maxNorm) = 0;                       
                        else
                            if A(j+1,i+1)==0
                                K{i,j} = zeros(3);
                            end
                        end
                    end        
                end
            end
      
            K
            obj.loadTopologyFromK2(K);
            obj.loadControllerGains2(K);

        end
        

        % Decentralized Stabilizing Controller Synthesis (Error Dynamics II)
        function status = decentralizedStabilizingControllerSynthesis2(obj,nuBar,rhoBar)

            if isempty(indexing)
                indexing = [1:1:(length(obj.vehicles)-1)];
            end
        
            for i = 1:1:length(indexing)
                iInd = indexing(i);
                previousSubsystems = indexing(1:i-1);                
                [isStabilizable,K_ii,K_ijVals,K_jiVals] = obj.vehicles(iInd).stabilizingControllerSynthesis2(previousSubsystems, obj.vehicles, nuBar, rhoBar);

                K{iInd,iInd} = K_ii;
                for j = 1:1:length(previousSubsystems)
                    jInd = previousSubsystems(j);
                    K{iInd,jInd} = K_ijVals{jInd};
                    K{jInd,iInd} = K_jiVals{jInd};

                    %% We need to update K_{jj} values here!!! K_{j0} = k_{j0} + k_{ji}
                    disp([' Local controller updated at j = ',num2str(j)])
                    obj.vehicles(jInd+1).localControllerGains2 = obj.vehicles(jInd+1).localControllerGains2 + K{jInd,iInd}(3,:);
                end
                obj.KSequence{iInd} = K;
                disp(['Current K at i = ',num2str(iInd)])
                disp(cell2mat(K))
        
                if ~isStabilizable
                    break
                end
            end
        
            if isStabilizable
                status = 1;
                obj.loadTopologyFromK2(K); 
                obj.loadControllerGains2(K);
            else
                status = 0;
            end
        end


        %% Robust Controller Synthesis Using Error Dynamics Formulation II

        % Centralized Robust Controller Synthesis (Error Dynamics II) 
        function status = centralizedRobustControllerSynthesis2(obj,pVals)
            
            % Number of follower vehicles
            N = obj.numOfVehicles-1; 
            
            % Load local controllers and resulting passivity indices
            for i = 1:1:N
                p_i = pVals(i);
                obj.vehicles(i+1).synthesizeLocalControllersParameterized(2,p_i);
            end
            
            % Creating the adgacency matrix, null matrix and cost matrix
            G = obj.topology.graph;
            A = adjacency(G);
            for i = 1:1:N
                for j = 1:1:N
                    % Structure of K_ij (which is a 3x3 matrix) should be embedded here
                    if i~=j
                        if A(j+1,i+1)==1
                            adjMatBlock{i,j} = [0,0,0; 0,0,0; 1,1,1];
                            nullMatBlock{i,j} = [1,1,1; 1,1,1; 0,0,0];
                            costMatBlock{i,j} = 1*[0,0,0; 0,0,0; 1,1,1];
                        else
                            adjMatBlock{i,j} = [0,0,0; 0,0,0; 0,0,0];
                            nullMatBlock{i,j} = [1,1,1; 1,1,1; 0,0,0];
                            costMatBlock{i,j} = (20/N)*abs(i-j)*[0,0,0; 0,0,0; 1,1,1];
                        end
                    else
                        adjMatBlock{i,j} = [0,0,0; 0,0,0; 1,1,1];
                        nullMatBlock{i,j} = [1,1,1; 1,1,1; 0,0,0];
                        costMatBlock{i,j} = 0*[0,0,0; 0,0,0; 1,1,1];
                    end
                end 
            end
            adjMatBlock = cell2mat(adjMatBlock);
            nullMatBlock = cell2mat(nullMatBlock);
            costMatBlock = cell2mat(costMatBlock);
            
            % Set up the LMI problem
            solverOptions = sdpsettings('solver','mosek','verbose',0);
            I = eye(3*N);
            I_n = eye(3);
            O = zeros(3*N);

            % Whether to use a soft or hard graph constraint
            isSoft = 1;
            % normType = 2;

            Q = sdpvar(3*N,3*N,'full'); 
            P = sdpvar(N,N,'diagonal');
            gammaSq = sdpvar(1,1,'full');

            X_p_11 = [];
            X_p_12 = [];
            X_12 = [];
            X_p_22 = [];
            for i = 1:1:N
                nu_i = obj.vehicles(i+1).nu;
                rho_i = obj.vehicles(i+1).rho;
            
                X_p_11 = blkdiag(X_p_11,-nu_i*P(i,i)*I_n);
                X_p_12 = blkdiag(X_p_12,0.5*P(i,i)*I_n);
                X_12 = blkdiag(X_12,(-1/(2*nu_i))*I_n);
                X_p_22 = blkdiag(X_p_22,-rho_i*P(i,i)*I_n);
            end
            X_p_21 = X_p_12';
            X_21 = X_12';
            
            % Objective Function
            % costFun = 1*norm(Q.*costMatBlock,normType);
            costFun0 = sum(sum(Q.*costMatBlock));
    
            % Minimum Budget Constraints
            con0 = costFun0 >= 0.002;
                        
            % Basic Constraints
            con1 = P >= 0;

            DMat = [X_p_11, O; O, I];
            MMat = [Q, X_p_11; I, O];
            ThetaMat = [-X_21*Q-Q'*X_12-X_p_22, -X_p_21; -X_p_12, gammaSq*I];
            con2 = [DMat, MMat; MMat', ThetaMat] >= 0; % The real one
            
            % Structural constraints
            con3 = Q.*(nullMatBlock==1)==O;  % Structural limitations (due to the format of the control law)
            con4 = Q.*(adjMatBlock==0)==O;  % Graph structure : hard constraint
            
            
            % Total Cost and Constraints
            if isSoft
                cons = [con0,con1,con2,con3]; % Without the hard graph constraint con7
                costFun = 1*costFun0 + 1*gammaSq; % soft 
            else
                cons = [con0,con1,con2,con3,con4]; % With the hard graph constraint con7
                costFun = 1*costFun0 + 1*gammaSq; % hard (same as soft)
            end
            
            sol = optimize(cons,[costFun],solverOptions);
            status = sol.problem == 0; %sol.info;
            
            costFun0Val = value(costFun0)
            costFunVal = value(costFun)
            PVal = value(P)
            QVal = value(Q)


%             con2Val = value([DMat, MMat; MMat', ThetaMat]);
%             eigVals = eig(con2Val);   
%             minEigVal = min(eigVals)
%             con2Val = value(con2)

            % tempMat = value([DMat, MMat; MMat', ThetaMat])
            % detTempMat = det(tempMat)
            
            X_p_11Val = value(X_p_11);
            X_p_21Val = value(X_p_21);

            gammaSqVal = value(gammaSq)

            M_neVal = X_p_11Val\QVal;
            
            % Obtaining K_ij blocks
            M_neVal(nullMatBlock==1) = 0;
            maxNorm = 0;
            for i = 1:1:N
                for j = 1:1:N
                    K{i,j} = M_neVal(3*(i-1)+1:3*i , 3*(j-1)+1:3*j); % (i,j)-th (3 x 3) block
                    normVal = max(max(abs(K{i,j})));
                    if normVal>maxNorm 
                        maxNorm = normVal;
                    end
                end
            end
            
            % filtering out extremely small interconnections
            for i=1:1:N
                for j=1:1:N
                    if i~=j
                        if isSoft
                            K{i,j}(abs(K{i,j})<0.0001*maxNorm) = 0;                       
                        else
                            if A(j+1,i+1)==0
                                K{i,j} = zeros(3);
                            end
                        end
                    end
                    
                    K_ijMax = max(abs(K{i,j}(:)));
                    K{i,j}(abs(K{i,j})<0.01*K_ijMax) = 0;

                end
            end
      
            obj.K = K
            obj.loadTopologyFromK2(K);
            obj.loadControllerGains2(K);

        end       

        

        % Decentralized Robust Controller Synthesis (Error Dynamics Formulation II)
        function status = decentralizedRobustControllerSynthesis2(obj,pVals)
            displayMasseges = 1;
            N = length(obj.vehicles)-1;
            indexing = 1:1:N; % Lets use the default indexin scheme
            isSoft = 1;
            
            G = obj.topology.graph;
            A = adjacency(G);

            gammaSqVal = 0;
            for i = 1:1:length(indexing)
                iInd = indexing(i);
                previousSubsystems = indexing(1:i-1);  

                tic
                [isRobustStabilizable,K_ii,K_ijVals,K_jiVals,gammaSq_iVal,statusL,LVal] = obj.vehicles(iInd+1).robustControllerSynthesis2(previousSubsystems, obj.vehicles, pVals(iInd), displayMasseges, isSoft);
                toc

                K{iInd,iInd} = K_ii;
                for j = 1:1:length(previousSubsystems)
                    jInd = previousSubsystems(j);
                    K{iInd,jInd} = K_ijVals{jInd};
                    K{jInd,iInd} = K_jiVals{jInd};

                    %% We need to update K_{j0} values here!!! K_{j0} = k_{j0} + k_{ji}
                    disp([' Local controller updated at j = ',num2str(j)])
                    obj.vehicles(jInd+1).localControllerGains2 = obj.vehicles(jInd+1).localControllerGains2 + K{jInd,iInd}(3,:);
                end
                obj.KSequence{iInd} = K;
                disp(['Current K at i = ',num2str(iInd)])
                disp(cell2mat(K))

                if gammaSq_iVal > gammaSqVal
                    gammaSqVal = gammaSq_iVal;
                end
        
                if ~isRobustStabilizable
                    break
                end
            end
            
            if isRobustStabilizable
                disp(['Global Synthesis Success with gammaSq=',num2str(value(gammaSqVal))])
                status = 1;

                maxNorm = 0;
                for i = 1:1:N
                    for j = 1:1:N
                        normVal = max(max(abs(K{i,j})));
                        if normVal>maxNorm 
                            maxNorm = normVal;
                        end
                    end
                end
                
                % filtering out extremely small interconnections
                for i=1:1:N
                    for j=1:1:N
                        if i~=j
                            if isSoft
                                K{i,j}(abs(K{i,j})<0.0001*maxNorm) = 0;                       
                            else
                                if A(j+1,i+1)==0
                                    K{i,j} = zeros(3);
                                end
                            end
                        end
                        K_ijMax = max(abs(K{i,j}(:)));
                        K{i,j}(abs(K{i,j})<0.01*K_ijMax) = 0;
                    end
                end
                

                obj.K = K
                % Loading topology based on K
                obj.loadTopologyFromK2(K); 
                obj.loadControllerGains2(K);
            else
                disp(['Global Synthesis Failed'])
                status = 0;
            end

        end 



        %% Robust Controller Synthesis Using Error Dynamics Formulation II With DSS Constraints

        % Centralized Robust Controller Synthesis With DSS Constraints (not finished) (Error Dynamics II) 
        function status = centralizedRobustControllerSynthesisDSS2(obj,pVals)
            
            % Number of follower vehicles
            N = obj.numOfVehicles-1; 
            
            % Load local controllers and resulting passivity indices
            for i = 1:1:N
                p_i = pVals(i);
                obj.vehicles(i+1).synthesizeLocalControllersParameterized(2,p_i);
            end
            
            % Creating the adjacency matrix, null matrix and cost matrix
            G = obj.topology.graph;
            A = adjacency(G);
            for i = 1:1:N
                for j = 1:1:N
                    % Structure of K_ij (which is a 3x3 matrix) should be embedded here
                    if i~=j
                        if A(j+1,i+1)==1
                            adjMatBlock{i,j} = [0,0,0; 0,0,0; 1,1,1];
                            nullMatBlock{i,j} = [1,1,1; 1,1,1; 0,0,0];
                            costMatBlock{i,j} = 1*[0,0,0; 0,0,0; 1,1,1];
                        else
                            adjMatBlock{i,j} = [0,0,0; 0,0,0; 0,0,0];
                            nullMatBlock{i,j} = [1,1,1; 1,1,1; 0,0,0];
                            costMatBlock{i,j} = (20/N)*abs(i-j)*[0,0,0; 0,0,0; 1,1,1];
                        end
                    else
                        adjMatBlock{i,j} = [0,0,0; 0,0,0; 1,1,1];
                        nullMatBlock{i,j} = [1,1,1; 1,1,1; 0,0,0];
                        costMatBlock{i,j} = 0*[0,0,0; 0,0,0; 1,1,1];
                    end
                end 
            end
            adjMatBlock = cell2mat(adjMatBlock);
            nullMatBlock = cell2mat(nullMatBlock);
            costMatBlock = cell2mat(costMatBlock);
            
            % Set up the centralized LMI problem
            solverOptions = sdpsettings('solver','mosek','verbose',0);
            I = eye(3*N);
            I_n = eye(3);
            O = zeros(3*N);

            % Whether to use a soft or hard graph constraint
            isSoft = 1;
            % normType = 2;

            Q = sdpvar(3*N,3*N,'full'); 
            P = sdpvar(N,N,'diagonal');
            gammaSq = sdpvar(1,1,'full');

            X_p_11 = [];
            X_p_12 = [];
            X_12 = [];
            X_p_22 = [];
            for i = 1:1:N
                nu_i = obj.vehicles(i+1).nu;
                rho_i = obj.vehicles(i+1).rho;
            
                X_p_11 = blkdiag(X_p_11,-nu_i*P(i,i)*I_n);
                X_p_12 = blkdiag(X_p_12,0.5*P(i,i)*I_n);
                X_12 = blkdiag(X_12,(-1/(2*nu_i))*I_n);
                X_p_22 = blkdiag(X_p_22,-rho_i*P(i,i)*I_n);
            end
            X_p_21 = X_p_12';
            X_21 = X_12';
            
            % Objective Function
            % costFun = 1*norm(Q.*costMatBlock,normType);
            costFun0 = sum(sum(Q.*costMatBlock));
    
            % Minimum Budget Constraints
            con0 = costFun0 >= 0.0001;
                        
            % Basic Constraints
            con1 = P >= 0;

            DMat = [X_p_11, O; O, I];
            MMat = [Q, X_p_11; I, O];
            ThetaMat = [-X_21*Q-Q'*X_12-X_p_22, -X_p_21; -X_p_12, gammaSq*I];
            con2 = [DMat, MMat; MMat', ThetaMat] >= 0; % The real one
            
            % Structural constraints
            con3 = Q.*(nullMatBlock==1)==O;  % Structural limitations (due to the format of the control law)
            con4 = Q.*(adjMatBlock==0)==O;  % Graph structure : hard constraint (can be ignored if we allow changes for the initial graph)
            
            
            % Total Cost and Constraints
            if isSoft
                cons = [con0,con1,con2,con3]; % Without the hard graph constraint con7
                costFun = 1*costFun0 + 1*gammaSq; % soft 
            else
                cons = [con0,con1,con2,con3,con4]; % With the hard graph constraint con7
                costFun = 1*costFun0 + 1*gammaSq; % hard (same as soft)
            end
            
            sol = optimize(cons,[costFun],solverOptions);
            status = sol.problem == 0; %sol.info;
            
            costFun0Val = value(costFun0)
            costFunVal = value(costFun)
            PVal = value(P)
            QVal = value(Q)

            
            X_p_11Val = value(X_p_11);
            X_p_21Val = value(X_p_21);

            gammaSqVal = value(gammaSq)
            
            % This is the M_{\eta e} matrix block of the M matrix
            M_neVal = X_p_11Val\QVal;
            
            % Obtaining K_ij blocks
            M_neVal(nullMatBlock==1) = 0;
            maxNorm = 0;
            for i = 1:1:N
                for j = 1:1:N
                    K{i,j} = M_neVal(3*(i-1)+1:3*i , 3*(j-1)+1:3*j); % (i,j)-th (3 x 3) block
                    normVal = max(max(abs(K{i,j})));
                    if normVal>maxNorm 
                        maxNorm = normVal;
                    end
                end
            end
            
            % filtering out extremely small interconnections
            for i=1:1:N
                for j=1:1:N
                    if i~=j
                        if isSoft
                            K{i,j}(abs(K{i,j})<0.0001*maxNorm) = 0;                       
                        else
                            if A(j+1,i+1)==0
                                K{i,j} = zeros(3);
                            end
                        end
                    end
                    
                    K_ijMax = max(abs(K{i,j}(:)));
                    K{i,j}(abs(K{i,j})<0.01*K_ijMax) = 0;

                end
            end
      
            K
            obj.loadTopologyFromK2(K);
            obj.loadControllerGains2(K);

        end       

        

        % Decentralized Robust Controller Synthesis With DSS Constraints (Error Dynamics Formulation II)
        function status = decentralizedRobustControllerSynthesisDSS2(obj,pVals)
            displayMasseges = 1;
            N = length(obj.vehicles)-1;
            indexing = 1:1:N; % Lets use the default indexin scheme
            isSoft = 1;

            gammaSqVal = 0;
            for i = 1:1:length(indexing)
                iInd = indexing(i);
                previousSubsystems = indexing(1:i-1);                
                [isRobustStabilizable,K_ii,K_ijVals,K_jiVals,gammaSq_iVal,statusL,LVal] = obj.vehicles(iInd+1).robustControllerSynthesisDSS2(previousSubsystems, obj.vehicles, pVals(iInd), displayMasseges, isSoft);
                
                % Collect the K_{ii} value into the K matrix
                K{iInd,iInd} = K_ii;

                % Collect the K_{ij} and K_{ji} values into the K matrix
                for j = 1:1:length(previousSubsystems)
                    jInd = previousSubsystems(j);
                    K{iInd,jInd} = K_ijVals{jInd};
                    K{jInd,iInd} = K_jiVals{jInd};

                    %% We need to update K_{jj} values here!!! K_{j0} = k_{j0} + k_{ji}
                    disp([' Local controller updated at j = ',num2str(j)])
                    obj.vehicles(jInd+1).localControllerGains2 = obj.vehicles(jInd+1).localControllerGains2 + K{jInd,iInd}(3,:);
                end
                obj.KSequence{iInd} = K;
                disp(['Current K at i = ',num2str(iInd)])
                disp(cell2mat(K))
                
                % Update the gammaSq value
                if gammaSq_iVal > gammaSqVal
                    gammaSqVal = gammaSq_iVal;
                end
        
                if ~isRobustStabilizable
                    break
                end
            end
            
            if isRobustStabilizable
                disp(['Global Synthesis Success with gammaSq=',num2str(value(gammaSqVal))])
                status = 1;

                maxNorm = 0;
                for i = 1:1:N
                    for j = 1:1:N
                        normVal = max(max(abs(K{i,j})));
                        if normVal>maxNorm 
                            maxNorm = normVal;
                        end
                    end
                end
                
                % filtering out extremely small interconnections
                for i=1:1:N
                    for j=1:1:N
                        if i~=j
                            if isSoft
                                K{i,j}(abs(K{i,j})<0.0001*maxNorm) = 0;                       
                            else
                                if A(j+1,i+1)==0
                                    K{i,j} = zeros(3);
                                end
                            end
                        end
                        K_ijMax = max(abs(K{i,j}(:)));
                        K{i,j}(abs(K{i,j})<0.01*K_ijMax) = 0;
                    end
                end

                obj.K = K
                % Loading topology based on K
                obj.loadTopologyFromK2(K); 
                obj.loadControllerGains2(K);
            else
                disp(['Global Synthesis Failed'])
                status = 0;
            end

        end



        
        %% Co-design functions

        % Objective function for co-design : Centralized Robust Controller Synthesis (Error Dynamics II) 
        function gammaSqVal = centralizedRobustControllerSynthesis2Codesign(obj,pVals)
            
            % Number of follower vehicles
            N = obj.numOfVehicles-1; 
            
            % Load local controllers and resulting passivity indices
            LVals = [];
            nuVals = [];
            rhoVals = [];
            gammaSqVals = [];
            for i = 1:1:N
                p_i = pVals(i);
                [statusL_i,LVal,nuVal,rhoVal,gammaSqVal] = obj.vehicles(i+1).synthesizeLocalControllersParameterized(2,p_i);
                if statusL_i == 0
                    gammaSqVal = 1000000;
                    return;
                else 
                    LVals = [LVals;LVal];
                    nuVals = [nuVals,nuVal];
                    rhoVals = [rhoVals,rhoVal];
                    gammaSqVals = [gammaSqVals,gammaSqVal];
                end
            end
%             disp(['Local Synthesis Success at Vehicles with mean nu=',num2str(mean(nuVals)),'; mean rho=',num2str(mean(rhoVals)),'.'])

            % Creating the adgacency matrix, null matrix and cost matrix
            G = obj.topology.graph;
            A = adjacency(G);
            for i = 1:1:N
                for j = 1:1:N
                    % Structure of K_ij (which is a 3x3 matrix) should be embedded here
                    if i~=j
                        if A(j+1,i+1)==1
                            adjMatBlock{i,j} = [0,0,0; 0,0,0; 1,1,1];
                            nullMatBlock{i,j} = [1,1,1; 1,1,1; 0,0,0];
                            costMatBlock{i,j} = 1*[0,0,0; 0,0,0; 1,1,1];
                        else
                            adjMatBlock{i,j} = [0,0,0; 0,0,0; 0,0,0];
                            nullMatBlock{i,j} = [1,1,1; 1,1,1; 0,0,0];
                            costMatBlock{i,j} = (20/N)*abs(i-j)*[0,0,0; 0,0,0; 1,1,1];
                        end
                    else
                        adjMatBlock{i,j} = [0,0,0; 0,0,0; 1,1,1];
                        nullMatBlock{i,j} = [1,1,1; 1,1,1; 0,0,0];
                        costMatBlock{i,j} = 0*[0,0,0; 0,0,0; 1,1,1];
                    end
                end 
            end
            adjMatBlock = cell2mat(adjMatBlock);
            nullMatBlock = cell2mat(nullMatBlock);
            costMatBlock = cell2mat(costMatBlock);
            
            % Set up the LMI problem
            solverOptions = sdpsettings('solver','mosek','verbose',0);
            I = eye(3*N);
            I_n = eye(3);
            O = zeros(3*N);

            % Whether to use a soft or hard graph constraint
            isSoft = 1;
            normType = 2;

            Q = sdpvar(3*N,3*N,'full'); 
            P = sdpvar(N,N,'diagonal');
            gammaSq = sdpvar(1,1,'full');

            X_p_11 = [];
            X_p_12 = [];
            X_12 = [];
            X_p_22 = [];
            for i = 1:1:N
%                 nu_i = obj.vehicles(i+1).nu;
%                 rho_i = obj.vehicles(i+1).rho;
                nu_i = nuVals(i);
                rho_i = rhoVals(i);
                X_p_11 = blkdiag(X_p_11,-nu_i*P(i,i)*I_n);
                X_p_12 = blkdiag(X_p_12,0.5*P(i,i)*I_n);
                X_12 = blkdiag(X_12,(-1/(2*nu_i))*I_n);
                X_p_22 = blkdiag(X_p_22,-rho_i*P(i,i)*I_n);
            end
            X_p_21 = X_p_12';
            X_21 = X_12';
            
            % Objective Function
            % costFun0 = 1*norm(Q.*costMatBlock,normType);
            costFun0 = sum(sum(Q.*costMatBlock));
    
            % Minimum Budget Constraints
            con0 = costFun0 >= 0.002;
                        
            % Basic Constraints
            con1 = P >= 0;

            DMat = [X_p_11, O; O, I];
            MMat = [Q, X_p_11; I, O];
            ThetaMat = [-X_21*Q-Q'*X_12-X_p_22, -X_p_21; -X_p_12, gammaSq*I];
            con2 = [DMat, MMat; MMat', ThetaMat] >= 0; % The real one
            
            % Structural constraints
            con3 = Q.*(nullMatBlock==1)==O;  % Structural limitations (due to the format of the control law)
            con4 = Q.*(adjMatBlock==0)==O;  % Graph structure : hard constraint
            
            
            % Total Cost and Constraints
            if isSoft
                cons = [con0,con1,con2,con3]; % Without the hard graph constraint con7
                costFun = 1*costFun0 + 1*gammaSq; % soft 
            else
                cons = [con0,con1,con2,con3,con4]; % With the hard graph constraint con7
                costFun = 1*costFun0 + 1*gammaSq; % hard (same as soft)
            end
            
            sol = optimize(cons,[costFun],solverOptions);
            status = sol.problem == 0; %sol.info;
            
%             costFun0Val = value(costFun0);
%             costFunVal = value(costFun);
%             PVal = value(P);
%             QVal = value(Q);
%             X_p_11Val = value(X_p_11);
%             X_p_21Val = value(X_p_21);
% 
            gammaSqVal = value(gammaSq);
% 
%             M_neVal = X_p_11Val\QVal;
%             
%             % Obtaining K_ij blocks
%             M_neVal(nullMatBlock==1) = 0;
%             maxNorm = 0;
%             for i = 1:1:N
%                 for j = 1:1:N
%                     K{i,j} = M_neVal(3*(i-1)+1:3*i , 3*(j-1)+1:3*j); % (i,j)-th (3 x 3) block
%                     normVal = max(max(abs(K{i,j})));
%                     if normVal>maxNorm 
%                         maxNorm = normVal;
%                     end
%                 end
%             end
%             
%             % filtering out extremely small interconnections
%             for i=1:1:N
%                 for j=1:1:N
%                     if i~=j
%                         if isSoft
%                             K{i,j}(abs(K{i,j})<0.0001*maxNorm) = 0;                       
%                         else
%                             if A(j+1,i+1)==0
%                                 K{i,j} = zeros(3);
%                             end
%                         end
%                     end
%                     
%                     K_ijMax = max(abs(K{i,j}(:)));
%                     K{i,j}(abs(K{i,j})<0.01*K_ijMax) = 0;
% 
%                 end
%             end
      
%             K
%             obj.loadTopologyFromK2(K);
%             obj.loadControllerGains2(K);

            if status == 1
                disp(['Global Synthesis Success with gammaSq=',num2str(value(gammaSqVal))])
            else
                disp(['Global Synthesis Failed'])
                gammaSqVal = 1000000; 
                return;
            end

        end  



        % Feasibility constraints for Co-Design : Centralized Robust Controller Synthesis (Error Dynamics II) 
        function [C,Ceq] = centralizedRobustControllerSynthesis2CodesignFeasibility(obj,pVals)
            
            % Number of follower vehicles
            N = obj.numOfVehicles-1; 
            
            % Load local controllers and resulting passivity indices
            statusLVals = [];
            LVals = [];
            nuVals = [];
            rhoVals = [];
            gammaSqVals = [];
            for i = 1:1:N
                p_i = pVals(i);
                [statusL_i,LVal,nuVal,rhoVal,gammaSqVal] = obj.vehicles(i+1).synthesizeLocalControllersParameterized(2,p_i);
                
                statusLVals = [statusLVals;statusL_i];
                LVals = [LVals;LVal];
                nuVals = [nuVals,nuVal];
                rhoVals = [rhoVals,rhoVal];
                gammaSqVals = [gammaSqVals,gammaSqVal];
                
            end
            
            if any(statusLVals==0)
                statusG = 0;
                statusK = norm(LVals) <= 1000*sqrt(N);
                C = [statusLVals-ones(N,1); statusG-1; statusK-1];
                Ceq = [];
                return;
            end
            % Else: continue to do the gobal design
            
            % Creating the adgacency matrix, null matrix and cost matrix
            G = obj.topology.graph;
            A = adjacency(G);
            for i = 1:1:N
                for j = 1:1:N
                    % Structure of K_ij (which is a 3x3 matrix) should be embedded here
                    if i~=j
                        if A(j+1,i+1)==1
                            adjMatBlock{i,j} = [0,0,0; 0,0,0; 1,1,1];
                            nullMatBlock{i,j} = [1,1,1; 1,1,1; 0,0,0];
                            costMatBlock{i,j} = 1*[0,0,0; 0,0,0; 1,1,1];
                        else
                            adjMatBlock{i,j} = [0,0,0; 0,0,0; 0,0,0];
                            nullMatBlock{i,j} = [1,1,1; 1,1,1; 0,0,0];
                            costMatBlock{i,j} = (20/N)*abs(i-j)*[0,0,0; 0,0,0; 1,1,1];
                        end
                    else
                        adjMatBlock{i,j} = [0,0,0; 0,0,0; 1,1,1];
                        nullMatBlock{i,j} = [1,1,1; 1,1,1; 0,0,0];
                        costMatBlock{i,j} = 0*[0,0,0; 0,0,0; 1,1,1];
                    end
                end 
            end
            adjMatBlock = cell2mat(adjMatBlock);
            nullMatBlock = cell2mat(nullMatBlock);
            costMatBlock = cell2mat(costMatBlock);
            
            % Set up the LMI problem
            solverOptions = sdpsettings('solver','mosek','verbose',0);
            I = eye(3*N);
            I_n = eye(3);
            O = zeros(3*N);

            % Whether to use a soft or hard graph constraint
            isSoft = 1;
            normType = 2;

            Q = sdpvar(3*N,3*N,'full'); 
            P = sdpvar(N,N,'diagonal');
            gammaSq = sdpvar(1,1,'full');

            X_p_11 = [];
            X_p_12 = [];
            X_12 = [];
            X_p_22 = [];
            for i = 1:1:N
%                 nu_i = obj.vehicles(i+1).nu;
%                 rho_i = obj.vehicles(i+1).rho;
                nu_i = nuVals(i);
                rho_i = rhoVals(i);
                X_p_11 = blkdiag(X_p_11,-nu_i*P(i,i)*I_n);
                X_p_12 = blkdiag(X_p_12,0.5*P(i,i)*I_n);
                X_12 = blkdiag(X_12,(-1/(2*nu_i))*I_n);
                X_p_22 = blkdiag(X_p_22,-rho_i*P(i,i)*I_n);
            end
            X_p_21 = X_p_12';
            X_21 = X_12';
            
            % Objective Function
            % costFun0 = 1*norm(Q.*costMatBlock,normType);
            costFun0 = sum(sum(Q.*costMatBlock));
    
            % Minimum Budget Constraints
            con0 = costFun0 >= 0.002;
                        
            % Basic Constraints
            con1 = P >= 0;

            DMat = [X_p_11, O; O, I];
            MMat = [Q, X_p_11; I, O];
            ThetaMat = [-X_21*Q-Q'*X_12-X_p_22, -X_p_21; -X_p_12, gammaSq*I];
            con2 = [DMat, MMat; MMat', ThetaMat] >= 0; % The real one
            
            % Structural constraints
            con3 = Q.*(nullMatBlock==1)==O;  % Structural limitations (due to the format of the control law)
            con4 = Q.*(adjMatBlock==0)==O;  % Graph structure : hard constraint
            
            
            % Total Cost and Constraints
            if isSoft
                cons = [con0,con1,con2,con3]; % Without the hard graph constraint con7
                costFun = 1*costFun0 + 1*gammaSq; % soft 
            else
                cons = [con0,con1,con2,con3,con4]; % With the hard graph constraint con7
                costFun = 1*costFun0 + 1*gammaSq; % hard (same as soft)
            end
            
            sol = optimize(cons,[costFun],solverOptions);
            statusG = sol.problem == 0; %sol.info;
            
%             costFun0Val = value(costFun0);
%             costFunVal = value(costFun);
%             PVal = value(P);
%             QVal = value(Q);
%             X_p_11Val = value(X_p_11);
%             X_p_21Val = value(X_p_21);
% 
%             gammaSqVal = value(gammaSq);
% 
%             M_neVal = X_p_11Val\QVal;
%             
%             % Obtaining K_ij blocks
%             M_neVal(nullMatBlock==1) = 0;
%             maxNorm = 0;
%             for i = 1:1:N
%                 for j = 1:1:N
%                     K{i,j} = M_neVal(3*(i-1)+1:3*i , 3*(j-1)+1:3*j); % (i,j)-th (3 x 3) block
%                     normVal = max(max(abs(K{i,j})));
%                     if normVal>maxNorm 
%                         maxNorm = normVal;
%                     end
%                 end
%             end
%             
%             % filtering out extremely small interconnections
%             for i=1:1:N
%                 for j=1:1:N
%                     if i~=j
%                         if isSoft
%                             K{i,j}(abs(K{i,j})<0.0001*maxNorm) = 0;                       
%                         else
%                             if A(j+1,i+1)==0
%                                 K{i,j} = zeros(3);
%                             end
%                         end
%                     end
%                     
%                     K_ijMax = max(abs(K{i,j}(:)));
%                     K{i,j}(abs(K{i,j})<0.01*K_ijMax) = 0;
% 
%                 end
%             end
            % K

            statusK = norm(LVals) <= 1000*sqrt(N);
            C = [statusLVals-ones(N,1); statusG-1; statusK-1];
            Ceq = [];

        end  



        function gammaSqVal = decentralizedRobustControllerSynthesis2Codesign(obj,pVals)
            displayMasseges = 0;
            N = length(obj.vehicles)-1;
            indexing = 1:1:N; % Lets use the default indexin scheme
            isSoft = 0;

            gammaSqVal = 0;
            for i = 1:1:length(indexing)
                iInd = indexing(i);
                previousSubsystems = indexing(1:i-1);                
                [isRobustStabilizable,K_ii,K_ijVals,K_jiVals,gammaSq_iVal,statusL_i,L_iVal] = obj.vehicles(iInd+1).robustControllerSynthesis2(previousSubsystems, obj.vehicles, pVals(iInd), displayMasseges, isSoft);
                
                if ~isRobustStabilizable
                    break
                end

                if gammaSq_iVal > gammaSqVal
                    gammaSqVal = gammaSq_iVal;
                end                
            end

            if isRobustStabilizable
                disp(['Global Synthesis Success with gammaSq=',num2str(value(gammaSqVal))])
            else
                gammaSqVal = 1000000-i*(10000);
                disp(['Global Synthesis Failed'])
            end
                  
        end 



        function [C,Ceq] = decentralizedRobustControllerSynthesis2CodesignFeasibility(obj,pVals)
            displayMasseges = 0;
            N = length(obj.vehicles)-1;
            indexing = 1:1:N; % Lets use the default indexin scheme
            isSoft = 0;

            statusLVals = zeros(N,1);
            LVals = zeros(N,3);
            for i = 1:1:length(indexing)
                iInd = indexing(i);
                previousSubsystems = indexing(1:i-1);                
                [isRobustStabilizable,K_ii,K_ijVals,K_jiVals,gammaSq_iVal,statusL_i,L_iVal] = obj.vehicles(iInd+1).robustControllerSynthesis2(previousSubsystems, obj.vehicles, pVals(iInd), displayMasseges, isSoft);
                
                if ~isRobustStabilizable
                    break
                else
                    statusLVals(i) = statusL_i;
                    LVals(i,:) = L_iVal;
                end
            end

            statusG = isRobustStabilizable;
            statusK = norm(LVals) <= 10000*sqrt(N);
            C = [statusLVals-ones(N,1); statusG-1; statusK-1];
            Ceq = [];
        end



        function gammaSqVal = decentralizedRobustControllerSynthesisDSS2Codesign(obj,pVals)
            displayMasseges = 0;
            N = length(obj.vehicles)-1;
            indexing = 1:1:N; % Lets use the default indexin scheme
            isSoft = 1;

            gammaSqVal = 0;
            for i = 1:1:length(indexing)
                iInd = indexing(i);
                previousSubsystems = indexing(1:i-1);                
                [isRobustStabilizable,K_ii,K_ijVals,K_jiVals,gammaSq_iVal,statusL_i,L_iVal] = obj.vehicles(iInd+1).robustControllerSynthesisDSS2(previousSubsystems, obj.vehicles, pVals(iInd), displayMasseges, isSoft);
                
                if ~isRobustStabilizable
                    break
                end

                if gammaSq_iVal > gammaSqVal
                    gammaSqVal = gammaSq_iVal;
                end                
            end

            if isRobustStabilizable
                disp(['Global Synthesis Success with gammaSq=',num2str(value(gammaSqVal))])
            else
                gammaSqVal = 1000000-i*(10000);
                disp(['Global Synthesis Failed'])
            end
                  
        end 

        

        function [C,Ceq] = decentralizedRobustControllerSynthesisDSS2CodesignFeasibility(obj,pVals)
            displayMasseges = 0;
            N = length(obj.vehicles)-1;
            indexing = 1:1:N; % Lets use the default indexin scheme
            isSoft = 1;

            statusLVals = zeros(N,1);
            LVals = zeros(N,3);
            for i = 1:1:length(indexing)
                iInd = indexing(i);
                previousSubsystems = indexing(1:i-1);                
                [isRobustStabilizable,K_ii,K_ijVals,K_jiVals,gammaSq_iVal,statusL_i,L_iVal] = obj.vehicles(iInd+1).robustControllerSynthesisDSS2(previousSubsystems, obj.vehicles, pVals(iInd), displayMasseges, isSoft);
                
                if ~isRobustStabilizable
                    break
                else
                    statusLVals(i) = statusL_i;
                    LVals(i,:) = L_iVal;
                end
            end

            statusG = isRobustStabilizable;
            statusK = norm(LVals) <= 10000*sqrt(N);
            C = [statusLVals-ones(N,1); statusG-1; statusK-1];
            Ceq = [];
        end

        % (Old) Centralized Robust Controller Synthesis (Error Dynamics II) 
%         function status = centralizedRobustControllerSynthesis2Old(obj)
%             
%             % Number of follower vehicles
%             N = obj.numOfVehicles-1; 
% 
%             % Whether to use a soft or hard graph constraint
%             isSoft = 1;
% 
%             % Desired noise attenuation level: L2Gain from w to e
%             gammaSqBar = 9;
% 
%             % Passivity indices type
%             passivityInfoType = 4;
% 
%             if passivityInfoType==1         % Load Passivity Indices - I
%                 tau = -sqrt(3)/4;
%                 nuVals = tau*ones(N,1);
%                 rhoVals = tau*ones(N,1);
%                 obj.loadPassivityIndices(nuVals,rhoVals);
%             elseif passivityInfoType==2     % Load Passivity Indices - II
%                 rho = -sqrt(2)/2;   
%                 nuVals = 0*ones(N,1);
%                 rhoVals = rho*ones(N,1);
%                 obj.loadPassivityIndices(nuVals,rhoVals);
%             elseif passivityInfoType==3 % With local controllers - I: rho = -1/2, nu = 0 (this approach fails to even stabilize)
%                 rho = -1/2;   
%                 nuVals = 0*ones(N,1);
%                 rhoVals = rho*ones(N,1);
%                 obj.loadPassivityIndices(nuVals,rhoVals);
%             elseif passivityInfoType==4 % With local controllers - II: rho = 0, nu = -1/2 (this approach can stabilize)
%                 nu = -1/2;
%                 nuVals = nu*ones(N,1);
%                 rhoVals = 0*ones(N,1);
%                 obj.loadPassivityIndices(nuVals,rhoVals);
%             end
% 
%             % Creating the adgacency matrix, null matrix and cost matrix
%             G = obj.topology.graph
%             A = adjacency(G)
%             for i = 1:1:N
%                 for j = 1:1:N
%                     % Structure of K_ij (which is a 3x3 matrix) should be embedded here
%                     if i~=j
%                         if A(j+1,i+1)==1
%                             adjMatBlock{i,j} = [0,0,0; 0,0,0; 1,1,1];
%                             nullMatBlock{i,j} = [1,1,1; 1,1,1; 0,0,0];
%                             costMatBlock{i,j} = 0.01*[0,0,0; 0,0,0; 1,1,1];
%                         else
%                             adjMatBlock{i,j} = [0,0,0; 0,0,0; 0,0,0];
%                             nullMatBlock{i,j} = [1,1,1; 1,1,1; 0,0,0];
%                             costMatBlock{i,j} = 1*[0,0,0; 0,0,0; 1,1,1];
%                         end
%                     else
%                         adjMatBlock{i,j} = [0,0,0; 0,0,0; 1,1,1];
%                         nullMatBlock{i,j} = [1,1,1; 1,1,1; 0,0,0];
%                         costMatBlock{i,j} = 0*[0,0,0; 0,0,0; 1,1,1];
%                     end
%                 end 
%             end
%             adjMatBlock = cell2mat(adjMatBlock)
%             nullMatBlock = cell2mat(nullMatBlock)
%             costMatBlock = cell2mat(costMatBlock)
%             
% 
%             % Set up the LMI problem
%             solverOptions = sdpsettings('solver','mosek');
%             I = eye(3*N);
%             I_n = eye(3);
%             O = zeros(3*N);
% 
%             Q = sdpvar(3*N,3*N,'full'); 
%             P = sdpvar(N,N,'diagonal');
%             gammaSq = sdpvar(1,1,'full');
%             
%             X_p_11 = [];
%             X_p_12 = [];
%             X_12 = [];
%             X_p_22 = [];
%             for i = 1:1:N
%                 nu_i = obj.vehicles(i+1).nu;
%                 rho_i = obj.vehicles(i+1).rho;
% 
%                 X_p_11 = blkdiag(X_p_11,-nu_i*P(i,i)*I_n);
%                 X_p_12 = blkdiag(X_p_12,0.5*P(i,i)*I_n);
%                 X_12 = blkdiag(X_12,(-1/(2*nu_i))*I_n);
%                 X_p_22 = blkdiag(X_p_22,-rho_i*P(i,i)*I_n);
%             end
%             X_p_21 = X_p_12';
%             X_21 = X_12';
%             
%             % Objective Function
%             costFun = norm(Q.*costMatBlock,1);
% 
%             % Budget Constraints
%             con0 = costFun <= 1;
% 
%             % Basic Constraints
%             con1 = P >= 0;
%             con2 = gammaSq >= 0;
%             con3 = gammaSq <= gammaSqBar;
%             con4 = trace(P)==1;
% 
%             if passivityInfoType==1 || passivityInfoType == 4
%                 % LMI Condition - I
%                 DMat = [X_p_11, O; O, I];
%                 MMat = [Q, X_p_11; I, O];
%                 ThetaMat = [-X_21*Q-Q'*X_12-X_p_22, -X_21*X_p_11; -X_p_11'*X_12, gammaSq*I];
%                 con5 = [DMat, MMat; MMat', ThetaMat] >= 0;
%             else
%                 % LMI Condition - II (when nu_i = 0, for all i)
% %                 DMat = [I];
% %                 MMat = [I, O];
% %                 ThetaMat = [-Q-Q'-X_p_22, -X_p_21; -X_p_12, gammaSq*I];
% %                 con5 = [DMat, MMat; MMat', ThetaMat] >= 0;
% %                 con5 = -Q-Q'-X_p_22 >= 0;
%             end
% 
%             % Structural constraints
%             con6 = Q.*(nullMatBlock==1)==O  % Structural limitations (due to the format of the control law)
%             con7 = Q.*(adjMatBlock==0)==O;  % Graph structure : hard constraint
% 
%             % Total Cost and Constraints
%             if isSoft
%                 cons = [con0,con1,con2,con3,con5,con6]; % Without the hard graph constraint con7
%                 costFun = 1*costFun + 1*gammaSq; % soft 
%             else
%                 cons = [con0,con1,con2,con3,con4,con5,con6,con7]; % With the hard graph constraint con7
%                 costFun = 1*costFun + 1*gammaSq; % hard (same as soft)
%             end
%             
%             sol = optimize(cons,[costFun],solverOptions);
%             status = sol.info
%             
%             PVal = value(P)
%             QVal = value(Q)
% 
%             costFunVal = value(costFun)
%             gammaSqVal = value(gammaSq)
% 
%             
%             X_p_11Val = value(X_p_11);
%             X_p_21Val = value(X_p_21);
% 
%             if passivityInfoType==1 || passivityInfoType == 4
%                 M_neVal = X_p_11Val\QVal
%             else
%                 M_neVal = X_p_21Val\QVal
%             end
%             
%             % Obtaining K_ij blocks
%             M_neVal(nullMatBlock==1) = 0;
%             maxNorm = 0;
%             for i = 1:1:N
%                 for j = 1:1:N
%                     K{i,j} = M_neVal(3*(i-1)+1:3*i , 3*(j-1)+1:3*j); % (i,j)-th (3 x 3) block
%                     normVal = max(max(abs(K{i,j})));
%                     if normVal>maxNorm 
%                         maxNorm = normVal;
%                     end
%                 end
%             end
%             
%             % filtering out small interconnections
%             for i=1:1:N
%                 for j=1:1:N
%                     
%                     if i~=j
%                         if isSoft
%                             K{i,j}(abs(K{i,j})<0.000001*maxNorm) = 0;                       
%                         else
%                             if A(j+1,i+1)==0
%                                 K{i,j} = zeros(3);
%                             end
%                         end
%                     end
%                     
%                 end
%             end
% 
%             K
% %             obj.loadTopologyFromK2(K);
%             obj.loadControllerGains2(K);
% 
%         end

        

        %% Functions for Loading Controller Gains and Topology under Different Error Dynamics Formulations

        % Loading Controller Gains (Error Dynamics Formulation I)
        function outputArg = loadControllerGains1(obj,K)
            N = obj.numOfVehicles-1;

            % Loading L_ij values from K
            for i = 1:1:N
                sumK_ij = zeros(3,3);
                for j = 1:1:N
                    if j~=i
                        sumK_ij = sumK_ij + K{i,j};
                        L{i,j} = -K{i,j}(2,3);   %[0,0,0; 0,0,\bar{k}_{ij}; 0,0,0]
                    end
                end                
                K_i0 = K{i,i} + sumK_ij;        %[0,0,0; 0,0,\bar{k}_{i0}; l_ii^x,l_ii^v,l_ii^a]
                L0{i} = K_i0(2,3);              
                L{i,i} = K_i0(3,:);             
            end
            
            L0
            L

            % Loading controller gains from L_ij values
            for i = 1:1:N
                obj.vehicles(i+1).controllerGains1 = [];
                obj.vehicles(i+1).controllerGains1{1} = L0{i};
                obj.vehicles(i+1).controllerGains1{i+1} = L{i,i};
                for j = 1:1:N
                    if j~=i && norm(L{i,j})> 0
                        obj.vehicles(i+1).controllerGains1{j+1} = L{i,j};
                    end
                end
            end

        end


        % Loading Topology (Error Dynamics Formulation I)
        function outputArg = loadTopologyFromK1(obj,K)
            NBar = obj.numOfVehicles;
            startNodes = [];
            endNodes = [];
            for i = 1:1:NBar
                nodeNames{i} = num2str(i-1);
                for j = 1:1:NBar
                    if j ~= 1 && j~=i 
                        if i == 1
                            startNodes = [startNodes,i];
                            endNodes = [endNodes,j];
                        elseif norm(K{i-1,j-1})> 0
                            startNodes = [startNodes,j];
                            endNodes = [endNodes,i];
                        end
                    end
                end

            end

            obj.topology = Topology(NBar,startNodes,endNodes,nodeNames);
            obj.updateNeighbors();

        end


        % Loading Controller Gains (Error Dynamics Formulation II With/Without DSS Constraints)
        function outputArg = loadControllerGains2(obj,K)

            N = obj.numOfVehicles-1;

            % Loading L_ij values from K
            for i = 1:1:N
                sumK_ij = zeros(3,3);
                for j = 1:1:N
                    if j~=i
                        sumK_ij = sumK_ij + K{i,j};
                        L{i,j} = -K{i,j}(3,:);   %[0,0,0; 0,0,0; l_ij^x,l_ij^v,l_ij^a]
                    end
                end                
                K_i0 = K{i,i} + sumK_ij;
                L{i,i} = K_i0(3,:);             %[0,0,0; 0,0,0; l_ii^x,l_ii^v,l_ii^a] 
            end

            % Loading controller gains from L_ij values
            for i = 1:1:N
                obj.vehicles(i+1).controllerGains2 = [];
                obj.vehicles(i+1).controllerGains2{i+1} = L{i,i};
                for j = 1:1:N
                    if j~=i && norm(L{i,j})> 0
                        obj.vehicles(i+1).controllerGains2{j+1} = L{i,j};
                    end
                end
            end
        
        end


        % Loading Controller Gains (Error Dynamics Formulation II With/Without DSS Constraints)
        function outputArg = loadTopologyFromK2(obj,K)

            NBar = obj.numOfVehicles;

            startNodes = [];
            endNodes = [];
            for i = 1:1:NBar
                nodeNames{i} = num2str(i-1);
                for j = 1:1:NBar
                    if j ~= 1 && j~=i 
                        if i == 1
                            startNodes = [startNodes,i];
                            endNodes = [endNodes,j];
                        elseif norm(K{i-1,j-1})> 0
                            startNodes = [startNodes,j];
                            endNodes = [endNodes,i];
                        end
                    end
                end

            end

            obj.topology = Topology(NBar,startNodes,endNodes,nodeNames);
            obj.updateNeighbors();

        end


        %% Codesign
        function pVals = optimizeCodesignParameters(obj,isCentralized,isDSS)
            
            N = obj.numOfVehicles-1;

            % pARAMETERS
            options = optimoptions('fmincon');
 
            % Set OptimalityTolerance to 1e-3
            options = optimoptions(options, 'OptimalityTolerance', 1e-3); 
     
            % Set the Display option to 'iter' and StepTolerance to 1e-4
            options.Display = 'iter';
            options.StepTolerance = 1e-4;

            A = [];
            b = [];
            Aeq = [];
            beq = [];
            lb = zeros(N,1);
            ub = inf*ones(N,1);
            
            
            if isCentralized && ~isDSS
                p0 = (1/N)*ones(N,1); % initial condition
                % f: Function of p_i parameters (each p_i used for local controller design), which will determine the resulting gamma value from the global design
                % f is what we need to optimize with respect to the used p_i parameters
                f = @(P)obj.centralizedRobustControllerSynthesis2Codesign(P);
                nonlcon = @(P)obj.centralizedRobustControllerSynthesis2CodesignFeasibility(P);
                [pVals,fval] = fmincon(f,p0,A,b,Aeq,beq,lb,ub,nonlcon,options)
            elseif ~isCentralized && ~isDSS
                p0 = (1/N)*ones(N,1);
                f = @(P)obj.decentralizedRobustControllerSynthesis2Codesign(P);
                nonlcon = @(P)obj.decentralizedRobustControllerSynthesis2CodesignFeasibility(P);
%                 [pVals,fval] = fmincon(f,p0,A,b,Aeq,beq,lb,ub)
                [pVals,fval] = fmincon(f,p0,A,b,Aeq,beq,lb,ub,nonlcon,options)
            elseif ~isCentralized && isDSS
                p0 = (1/N)*ones(N,1);
                f = @(P)obj.decentralizedRobustControllerSynthesisDSS2Codesign(P);
                nonlcon = @(P)obj.decentralizedRobustControllerSynthesisDSS2CodesignFeasibility(P);
%                 [pVals,fval] = fmincon(f,p0,A,b,Aeq,beq,lb,ub)
                [pVals,fval] = fmincon(f,p0,A,b,Aeq,beq,lb,ub,nonlcon,options)
            end           
            pVals = pVals';
        end
        
    end
end

