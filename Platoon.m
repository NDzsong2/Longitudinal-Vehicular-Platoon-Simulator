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
       
        graphics = []
    end
    
    methods

        function obj = Platoon(k,n_k,parameters,states,des_states,noiseMean,noiseStd)

            obj.platoonIndex = k;
            obj.numOfVehicles = n_k;
            
            % Create the vehicles
            vehicles = [];
            for i = 1:1:n_k
                % create an object from the Vehicle class
                vehicle = Vehicle(k,i,parameters(:,i),states(:,i),des_states(:,i),noiseMean(:,i),noiseStd(:,i));
                vehicles = [vehicles, vehicle];
            end
            obj.vehicles = vehicles;

            obj.topology = Topology(n_k);
            obj.updateNeighbors();   % update the neighbor information of each vehicle object inside obj.vehicles based on obj.topology

        end
        

        function outputArg = updateNeighbors(obj)
            for i = 1:1:obj.numOfVehicles
                obj.vehicles(i).inNeighbors = predecessors(obj.topology.graph,i);
                obj.vehicles(i).outNeighbors = successors(obj.topology.graph,i);
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

            numOfLinks = length(obj.topology.startNodes);
            for i = 1:1:numOfLinks
                % Draw a link
                startVehicleIndex = obj.topology.startNodes(i);
                endVehicleIndex = obj.topology.endNodes(i);

                startPos = obj.vehicles(startVehicleIndex).states(1) - obj.vehicles(startVehicleIndex).vehicleParameters(2)/2;
                endPos = obj.vehicles(endVehicleIndex).states(1) - obj.vehicles(endVehicleIndex).vehicleParameters(2)/2;
                midPos = (startPos + endPos)/2;

                startPosY = obj.vehicles(startVehicleIndex).vehicleParameters(2)*3/8;
                endPosY = obj.vehicles(endVehicleIndex).vehicleParameters(2)*3/8;
                obj.graphics(i) = plot([startPos,midPos,endPos],[0,4,0]+[startPosY,0,endPosY],'-b');
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
                    if ~isempty(obj.graphics(i))
                        delete(obj.graphics(i));

                    % Redraw a link
                    startVehicleIndex = obj.topology.startNodes(i);
                    endVehicleIndex = obj.topology.endNodes(i);
                    
                    startPos = obj.vehicles(startVehicleIndex).states(1) - obj.vehicles(startVehicleIndex).vehicleParameters(2)/2;
                    endPos = obj.vehicles(endVehicleIndex).states(1) - obj.vehicles(endVehicleIndex).vehicleParameters(2)/2;
                    midPos = (startPos + endPos)/2;
                    
                    startPosY = obj.vehicles(startVehicleIndex).vehicleParameters(2)*3/8;
                    endPosY = obj.vehicles(endVehicleIndex).vehicleParameters(2)*3/8;
                    obj.graphics(i) = plot([startPos,midPos,endPos],[0,4,0]+[startPosY,0,endPosY],'-b');
                    end
                end            
        end


        % Show the real time states (position and velocity) before each vehicle's arrow 
%         function outputArg = drawState(obj,figNum)
%             % Draw vehicle and the real time states
%             for i = 1:1:obj.numOfVehicles
% 
%                 x_i = obj.vehicles(i).states;
%                 obj.vehicles(i).drawState(figNum);
% 
%             end
% 
%             if ~isempty(obj.graphics)
%                 delete(obj.graphics)
%             end
%             
% %             handle = plot(poly1,'FaceColor','r');
% %             obj.graphics = handle;
% 
%         end
        
%           function outputArg = computeControlInputs(obj,platoons,dt)
%             % For each facility, all the control inputs and outputs needs to be determined. 
%             % Each facility has controlInputs and controlOutputs, each of dimention N. 
%             % Each facility can decide the controlInputs, but the controlOutputs are determined by the
%             % other facilities/demands. 
%             controlThreshold = 2000;
% 
%             for i = obj.numOfVehicles:-1:1
%                 k = obj.platoonIndex;
% 
%                 % u_{ii,k+1} (determined automatically)
%                 if k == obj.numOfVehicles
%                     obj.vehicles(k).controlOutputs(i) = obj.demand;
%                 else
%                     obj.vehicles(k).controlOutputs(i) = obj.facilities(k+1).controlInputs(i);
%                 end
% 
%                 % u_{ii,k} = u_{ii,k+1} + \rho_{i,k}\bar{x}_{i,k} + \bar{K}_{ii,k}e_{i,k} (determined based on feedback)
%                 K_ii_k = obj.controlGains(i,k); %-1;
%                 e_i_k = obj.facilities(k).stateX - obj.facilities(k).xBar;
%                 rho_i_k = obj.facilities(k).decayRate;
%                 controlInput = obj.facilities(k).controlOutputs(i) + rho_i_k*obj.facilities(k).xBar + K_ii_k*e_i_k;
%                 if controlInput < 0
%                     controlInput = 0;
%                 elseif controlInput>controlThreshold
%                     controlInput = controlThreshold;
%                 end
% 
%                 if k>1
%                     if obj.facilities(k-1).stateX < controlInput*dt
%                         controlInput = 0;
%                     end
%                 end
% 
%                 obj.facilities(k).controlInputs(i) = controlInput; %5*rand(1,1); % temp
% 
% 
%                 % u_{ij,k} = K_{ij,k}(e_{i,k}-e_{j,k-1}) (determined based on feedback)
%                 if k~=1
%                     inNeighbors = obj.facilities(k).inNeighbors;
%                     for j = inNeighbors
%                         e_j_k_1 = chains(j).facilities(k-1).stateX - chains(j).facilities(k-1).xBar;
%                         
%                         K_ij_k = obj.controlGains(j,k); %-2;
%                         controlInput = K_ij_k*(e_i_k-e_j_k_1);
%                         if controlInput < 0
%                             controlInput = 0;
%                         elseif controlInput>controlThreshold
%                             controlInput = controlThreshold;
%                         end
% 
%                         if k>1
%                             if chains(j).facilities(k-1).stateX < controlInput*dt
%                                 controlInput = 0;
%                             end
%                         end
%                         obj.facilities(k).controlInputs(j) = controlInput; %rand(1,1); 
%                     end 
%                 end
%             end
%         end

%         function outputArg = updateControlOutputs(obj,chains)
%             % For each facility, all the control inputs and outputs needs to be determined. 
%             % Each facility has controlInputs and controlOutputs, each of dimention N. 
%             % Each facility can decide the controlInputs, but the controlOutputs are determined by the
%             % other facilities/demands. 
            
%             for k = 1:1:obj.numOfVehicles
%                 i = obj.platoonIndex;
% 
%                 % u_{ji,k+1} (determined automatically)
%                 if k~=obj.numOfVehicles
%                     outNeighbors = obj.facilities(k).outNeighbors;
%                     for j = outNeighbors
%                         obj.facilities(k).controlOutputs(j) = chains(j).facilities(k+1).controlInputs(i);
%                     end 
%                 end
%             end
%         end
        
%         function outputArg = generateDemand(obj)            
%             w = obj.demandMean + obj.demandStd*randn(1,1);
%             stepSize = 2;
%             if w < obj.demand - stepSize
%                 w = obj.demand - stepSize;
%             elseif w > obj.demand + stepSize
%                 w = obj.demand + stepSize;
%             end
%             obj.demand = w;
% 
%             if obj.demand < 0
%                 obj.demand = 0.01;
%             end
%         end


        function outputArg = generateStates(obj,dt)
            for i = 1:1:obj.numOfVehicles
                obj.vehicles(i).update(dt);
            end
        end


        function outputArg = generateNoises(obj)
            for i = 1:1:obj.numOfVehicles
                obj.vehicles(i).generateNoise();
            end
        end

%         function totalError = update(obj,dt)
%             error = 0;
%             for i = 1:1:obj.numOfVehicles
%                 obj.vehicles(i).update(dt);
%                 error = error + (obj.vehicles(i).state-obj.vehicles(i).des_state);
%             end
%             totalError = error;
% 
%         end

        
    end
end
