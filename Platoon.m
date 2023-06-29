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
        
        function outputArg = generateNoises(obj)
            for i = 1:1:obj.numOfVehicles
                obj.vehicles(i).generateNoise();
            end
        end

        function totalError = update(obj,dt)
            for i = 1:1:obj.numOfVehicles
                obj.vehicles(i).update(dt);
            end
        end
        
    end
end

