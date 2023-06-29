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

        function obj = Network(indexVal,numOfPlatoons,numOfVehicles,parameters,states,des_states,noiseMean,noiseStd)

            obj.networkIndex = indexVal;
            obj.numOfPlatoons = numOfPlatoons;
            obj.numOfVehicles = numOfVehicles;

            % create the platoons
            platoons = [];
            for k = 1:1:numOfPlatoons
                platoon = Platoon(k,numOfVehicles(k),parameters{k},states{k},des_states{k},noiseMean{k},noiseStd{k});
                platoons = [platoons, platoon];
            end
            obj.platoons = platoons;
                        
            obj.time = 0;
            obj.error = 0;
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
            obj.graphics(2) = text(posX1+30,posY2-5,['Error: ',num2str(obj.error)],'FontSize',12);
            obj.graphics(3) = text(posX1+55,posY2-5,['Cost: ',num2str(obj.cost)],'FontSize',12);

            axis([posX1,posX2,posY1,posY2])
        end


        function outputArg = update(obj,dt)
            
            % Do the necessary computations/generations to generate the signals to initiate the program
            for k = 1:1:obj.numOfPlatoons

                % Generate the noises
                obj.platoons(k).generateNoises();                      % generate the noises associated with the platoons.

                % Compute all the controls 
%                 obj.platoons(k).computeControlInputs(obj.platoons,dt); % compute all the self-controls associated with the platoons.

                % Update the states
                obj.platoons(k).update(dt);
                
            end

%             % Update the cross-chain control outputs based on the computed cross-chain control inputs)
%             for k = 1:1:obj.numOfPlatoons
%                 obj.chains(k).updateControlOutputs(obj.chains);
%             end

            % Update the states
%             costSoFar = obj.cost*obj.time;
            obj.time = obj.time + dt;
%             totalError = 0;          

%             for k = 1:1:obj.numOfPlatoons
%                 platoonError = obj.platoons(k).update(dt);
%                 totalError = totalError + platoonError;
%             end
% 
%             obj.error = totalError;
%             costSoFar = costSoFar + obj.error;
%             obj.cost = costSoFar/obj.time;
            
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

                % Coordinate of the boundary of the bounding box
                posY1 = -5;
                posY2 = 20;
                lastPlatoon = obj.platoons(obj.numOfPlatoons);
                posX1 = lastPlatoon.vehicles(lastPlatoon.numOfVehicles).states(1)-10;
                posX2 = obj.platoons(1).vehicles(1).states(1)+10;
                
                obj.graphics(1) = text(posX1+5,posY2-5,['Time: ',num2str(obj.time)],'FontSize',12);
                obj.graphics(2) = text(posX1+30,posY2-5,['Error: ',num2str(round(obj.error,3))],'FontSize',12);
                obj.graphics(3) = text(posX1+55,posY2-5,['Cost: ',num2str(round(obj.cost,3))],'FontSize',12);           
                
                axis([posX1,posX2,posY1,posY2])
            end

        end


        
%         function K = synthesizeControllers(obj)
%             % n-1 graphs needs to be synthesized, each graph has N nodes
%             n = obj.numOfMaxFaci;
%             N = obj.numOfChains;
% 
%             isSoft = 1;
%             gammaSqLim = 0.01;
%             
%             % create adgacency matrics and cost matrices and nullMatrix
%             for k = 2:1:n
%                 adjMat{k} = adjacency(obj.topologies(k-1).graph); 
%                 % adjMatI{k} = adjMat{k} + eye(N);
%                 % costMat{k} = 10*AdjMat{k} + 1*eye(N) + 100*(AdjMatI{k}==0); % cost matrices
%             end
% 
%             for i = 1:1:N
%                 for j = 1:1:N
%                     % Structure of K_ij (which is an n x n matrix) should be embedded here
%                     if i~=j
%                         % If (i,j) \in G_k, we need to set the (k,k-1)-th term to 1, for all k \in {2,...,n}
%                         % If (j,i) \in G_k, we need to set the (k-1,k)-th term to 1, for all k \in {2,...,n}
%                         % otherwise zero
%                         nullMatrixBlock{i,j} = zeros(n,n);
%                         AdjMatBlock{i,j} = zeros(n,n);
%                         costMatBlock{i,j} = zeros(n,n);
%                         for k = 2:1:n
%                             G_k = obj.topologies(k-1).graph;
%                             nullMatrixBlock{i,j}(k,k-1) = 1;
%                             if adjMat{k}(i,j)==1
%                                 AdjMatBlock{i,j}(k,k-1) = 1;
%                                 costMatBlock{i,j}(k,k-1) = 0.01*G_k.Edges.Weight(findedge(G_k, i, j));
%                             else
%                                 costMatBlock{i,j}(k,k-1) = 10000;
%                             end
%                             nullMatrixBlock{i,j}(k-1,k) = 1;
%                             if adjMat{k}(j,i)==1
%                                 AdjMatBlock{i,j}(k-1,k) = 1;
%                                 costMatBlock{i,j}(k-1,k) = 0.01*G_k.Edges.Weight(findedge(G_k, i, j));
%                             else
%                                 costMatBlock{i,j}(k-1,k) = 10000;
%                             end
%                         end
%                     else
%                         nullMatrixBlock{i,j} = eye(n);
%                         AdjMatBlock{i,j} = eye(n);
%                         costMatBlock{i,j} = 100*eye(n);
%                     end
%                 end 
%             end
%             nullMatrixBlock = cell2mat(nullMatrixBlock);
%             AdjMatBlock = cell2mat(AdjMatBlock)
%             costMatBlock = cell2mat(costMatBlock)
%             
% 
%             % Set up the LMI problem
%             solverOptions = sdpsettings('solver','mosek');
%             I = eye(n);
%             O = zeros(n*N);
% 
%             L_ux = sdpvar(n*N,n*N,'full'); 
%             P = sdpvar(N,N,'diagonal');
%             gammaSq = sdpvar(1);
%             
%             X_p_12 = [];
%             X_p_22 = [];
%             for i = 1:1:N
%                 X_p_12 = blkdiag(X_p_12,0.5*P(i,i)*I);
%                 X_p_22 = blkdiag(X_p_22,-obj.chains(i).rhoBar*P(i,i)*I);
%             end
%             X_p_21 = X_p_12';
%             
%             DMat = [eye(n*N)];
%             MMat = [eye(n*N), O];
%             ThetaMat = [-L_ux-L_ux'-X_p_22, X_p_21; X_p_12, gammaSq*eye(n*N)];
%             
%             costFun = norm(L_ux.*costMatBlock);
%             con1 = P>=0;
%             con2 = gammaSq>=0;
%             con3 = [DMat, MMat; MMat', ThetaMat]>= 0;
%             con4 = gammaSq <= gammaSqLim; % determined by hard constraint result
%             % con5 = sum(L_ux,2)==0*ones(n*N,1);
%             con6 = L_ux.*(AdjMatBlock==0)==O;  % Graph structure : hard constraint
%             con7 = trace(P)==1
%             con8 = L_ux.*(nullMatrixBlock==0)==O; % Structural limitations (due to the control law)
%             con9 = costFun <= 3*25*obj.numOfChains; %Coef = eg#
%             
%              
%             if isSoft
%                 cons = [con1,con2,con3,con4,con7,con8,con9]; % soft add con4 rem con7
%                 costFun = 1*costFun + 1*gammaSq - 0*trace(P); % soft 1*, 1*, -1* 
%             else
%                 cons = [con1,con2,con3,con4,con6,con7,con8,con9]; % hard add con4 rem con7
%                 costFun = 1*costFun + 1*gammaSq - 0*trace(P); % hard (same as soft, 1*, 1*, -1* )
%             end
%             
%             sol = optimize(cons,[costFun],solverOptions);
%             sol.info
%             
%             PVal = value(P)
%             costFunVal = value(costFun)
%             gammaSqVal = value(gammaSq)
%             L_uxVal = value(L_ux);
%             X_p_21Val = value(X_p_21);
%             M_uxVal = X_p_21Val\L_uxVal
%             
% 
%             % Obtaining K_ij blocks
%             M_uxVal(nullMatrixBlock==0) = 0;
%             maxNorm = 0;
%             for i = 1:1:N
%                 for j = 1:1:N
%                     K{i,j} = M_uxVal(n*(i-1)+1:n*i , n*(j-1)+1:n*j); % (i,j)-th (n x n) block
%                     normVal = max(max(abs(K{i,j})));
%                     if normVal>maxNorm & i~=j
%                         maxNorm = normVal;
%                     end
%                 end
%             end
%             
%             % filtering out small interconnections
%             for i=1:1:N
%                 for j=1:1:N
%                     if isSoft & i~=j 
%                         K{i,j}(abs(K{i,j})<0.001*maxNorm) = 0;                       
%                     end
%                     if ~isSoft & i~=j
%                         % If (i,j) \in G_k, we need to set the (k,k-1)-th term to 1, for all k \in {2,...,n}
%                         % If (j,i) \in G_k, we need to set the (k-1,k)-th term to 1, for all k \in {2,...,n}
%                         % otherwise zero
%                         for k = 2:1:n
%                             if adjMat{k}(i,j)==0
%                                 K{i,j}(k,k-1) = 0;
%                             end
%                             if adjMat{k}(j,i)==0
%                                 K{i,j}(k-1,k) = 0;
%                             end
%                         end
%                     else
% 
%                     end
%                     
%                 end
%             end
%             % K = cell2mat(K);
% 
%             % Deriving K_{ij,k} values from K
%             obj.loadControllerGains(K)
%             
%         end
% 
%         function outputArg = loadControllerGains(obj,K)
%             n = obj.numOfMaxFaci;
%             N = obj.numOfChains;
%             for i = 1:1:N
%                 controlGains = zeros(N,n);
%                 sumK_ij = zeros(n,n);
%                 for j = 1:1:N
%                     if j~=i
%                         sumK_ij = sumK_ij + K{i,j};
%                         for k = 2:1:n
%                             controlGains(j,k) = K{i,j}(k,k-1);
%                         end
%                     end
%                 end
% 
%                 for k = 2:1:n
%                     K_ii_k_check = -sumK_ij(k,k-1);
%                     K_ii_k_hat = -sumK_ij(k-1,k);
%                     K_ii_k = K{i,i}(k,k);
%                     K_ii_k_bar = K_ii_k - K_ii_k_check - K_ii_k_hat;
%                     controlGains(i,k) = K_ii_k_bar;
%                 end
% 
%                 controlGains(i,1) = K{i,i}(1,1);
%                 obj.chains(i).controlGains = controlGains;
%             end
% 
%             obj.loadTopologiesFromK(K);
% 
%         end
% 
%         function outputArg = loadTopologiesFromK(obj,K)
%             N = obj.numOfChains;
%             n = obj.numOfMaxFaci;
% 
%             for i = 1:1:N
%                 for k = 1:1:n
%                     obj.chains(i).facilities(k).outNeighbors = [];
%                     obj.chains(i).facilities(k).inNeighbors = [];
%                 end
%             end
% 
%             topologies = [];
%             for k = 1:1:n-1
%                 % Create the graph G_k
%                 s = []; % start nodes
%                 t = []; % end nodes
%                 s_pos = []; % start positions (for graphics)
%                 t_pos = []; % end positions (for graphics) 
%                 weights = []; % distances
%                 for i = 1:1:N
%                     pos_i = obj.chains(i).facilities(k).actPos + [obj.chains(i).facilities(k).radius;0];
%                     for j=1:1:N
%                         pos_j = obj.chains(j).facilities(k+1).actPos - [obj.chains(j).facilities(k+1).radius;0];
%                         %distVal = norm(pos_i-pos_j);
%                         distVal = K{i,j}(k+1,k);
%                         if j~=i && distVal~=0
%                             s = [s,i];
%                             t = [t,j];
%                             s_pos = [s_pos,pos_i];
%                             t_pos = [t_pos,pos_j];
%                             weights = [weights,distVal];
% 
%                             obj.chains(i).facilities(k).outNeighbors = [obj.chains(i).facilities(k).outNeighbors, j];
%                             obj.chains(j).facilities(k+1).inNeighbors = [obj.chains(j).facilities(k+1).inNeighbors, i];
%                         end
%                     end
%                 end
%                 topologies = [topologies, InfoFlowTopology(k,N,s,t,weights,s_pos,t_pos)];
%             end
% 
%             obj.topologies = topologies;
% 
%         end


        
    end
end