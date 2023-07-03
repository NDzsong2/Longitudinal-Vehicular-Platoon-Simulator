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
                obj.vehicles(i).controllerGains1{i} = L_ii;

                % In error dynamics formulation - II
                L_ii = -[5,5,5];
                obj.vehicles(i).controllerGains2{i} = L_ii;

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
                midPointHeight = -3*sign(startPos-endPos)+0.05*abs(startPos-endPos);

                startPosY = obj.vehicles(startVehicleIndex).vehicleParameters(2)*3/8;
                endPosY = obj.vehicles(endVehicleIndex).vehicleParameters(2)*3/8;
                obj.graphics(i) = plot([startPos,midPos,endPos],[0,midPointHeight,0]+[startPosY,0,endPosY],'-b');
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
                        midPointHeight = -3*sign(startPos-endPos)+0.05*abs(startPos-endPos);
                        
                        startPosY = obj.vehicles(startVehicleIndex).vehicleParameters(2)*3/8;
                        endPosY = obj.vehicles(endVehicleIndex).vehicleParameters(2)*3/8;
%                         obj.graphics(i) = plot([startPos,midPos,endPos],[0,4,0]+[startPosY,0,endPosY],'-b');
                        obj.graphics(i) = plot([startPos,midPos,endPos],[0,midPointHeight,0]+[startPosY,0,endPosY],'-b');
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
            for i = 1:1:obj.numOfVehicles
                obj.vehicles(i).update(t,dt);
            end
        end
        
    end
end

