classdef Vehicle < handle
    %FACILITY Summary of this class goes here
    %   Detailed explanation goes here
    
    properties

        % Indices
        platoonIndex      % k
        vehicleIndex      % i

        % Parameters
        vehicleParameters
        noiseMean
        noiseStd
        
        % States
        des_states           % desired signals to be tracked
        states               % x_ik
        noise                % v_ik
        controlInput
        outputs

        % state history
        stateHistory = []

        % Predefined controls
        plannedControls = [] % matrix of paris [t_i,u_i]

        % GeometricProperties (for plotting)          
        inNeighbors = []
        outNeighbors = []

        % graphicHandles
        graphics = []
    end
    
    methods

        function obj = Vehicle(k,i,parameters,states,des_states,noiseMean,noiseStd)

            % Constructor
            obj.platoonIndex = k;
            obj.vehicleIndex = i;

            obj.vehicleParameters = parameters;                     %[mass,length,height1,height2]

            obj.states = states;                                      % states of the i^{th} vehicle
            obj.des_states = des_states;                              % need to track this signal (desired position,velocity and 0 acceleration for i^{th} vehicle)
            
            
            % External disturbances represented by random noise
            obj.noiseMean = noiseMean;
            obj.noiseStd = noiseStd;

            obj.noise = noiseMean + noiseStd*randn(1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Initial controller values

            obj.controlInput = 5 * ones(1);
            obj.outputs = zeros(1);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            obj.inNeighbors = [];
            obj.outNeighbors = [];
            
        end


        % This function is used to draw a "car" shape object
        function outArg = drawVehicle(obj,figNum)
            figure(figNum); hold on;
      
            length = obj.vehicleParameters(2);
            height1 = obj.vehicleParameters(2)/8;
            height2 = obj.vehicleParameters(2)/8;
            radius = obj.vehicleParameters(2)/16;
            pos = obj.states(1);
            
           
            % Draw the car body
            % poly1 = polyshape([pos, (pos-length), (pos-length), pos],[0.5, 0.5, 0.5+height1, 0.5+height1]+8*(obj.platoonIndex-1)*[1, 1, 1, 1]);
            poly1 = polyshape([pos, (pos-length), (pos-length), pos],[0.5, 0.5, 0.5+height1, 0.5+height1]);
            obj.graphics(1) = plot(poly1,'FaceColor','r');

%             poly2 = polyshape([0.5*(pos+0.5*(pos+(pos-length))), 0.5*((pos-length)+0.5*(pos+(pos-length))),...
%                 0.5*((pos-length)+0.5*(pos+(pos-length))), 0.5*(pos+0.5*(pos+(pos-length)))],...
%                 [0.5+height1, 0.5+height1, (0.5+height1)+height2, (0.5+height1)+height2]+8*(obj.platoonIndex-1)*[1, 1, 1, 1]);
            poly2 = polyshape([0.5*(pos+0.5*(pos+(pos-length))), 0.5*((pos-length)+0.5*(pos+(pos-length))),...
                0.5*((pos-length)+0.5*(pos+(pos-length))), 0.5*(pos+0.5*(pos+(pos-length)))],...
                [0.5+height1, 0.5+height1, (0.5+height1)+height2, (0.5+height1)+height2]);
            obj.graphics(2) = plot(poly2,'FaceColor','r');
            
            % Draw two wheels
%             viscircles([0.5*((pos-length)+0.5*(pos+(pos-length))), 0.3+8*(obj.platoonIndex-1)], 0.3,'Color','k');
%             viscircles([0.5*(pos+0.5*(pos+(pos-length))), 0.3+8*(obj.platoonIndex-1)], 0.3,'Color','k');

            obj.graphics(3) = viscircles([0.5*((pos-length)+0.5*(pos+(pos-length))), 0.3], radius,'Color','k');
            obj.graphics(4) = viscircles([0.5*(pos+0.5*(pos+(pos-length))), 0.3], radius,'Color','k');
        end


        % This function is used to plot the real time states (i.e., position) of the 
        % i^{th} vehicle at the position before each vehicle
%         function outputArg = drawState(obj,figNum)
%             figure(figNum); hold on;
% 
%             if ~isempty(obj.graphics)
%                 delete(obj.graphics)
%             end
%             
%         end

        function outputArg = redrawVehicle(obj,figNum)
            figure(figNum); hold on;

            if ~isempty(obj.graphics)
                delete(obj.graphics);

                length = obj.vehicleParameters(2);
                height1 = obj.vehicleParameters(2)/8;
                height2 = obj.vehicleParameters(2)/8;
                radius = obj.vehicleParameters(2)/16;
                pos = obj.states(1);
            
           
                % Draw the car body
                poly1 = polyshape([pos, (pos-length), (pos-length), pos],[0.5, 0.5, 0.5+height1, 0.5+height1]);
                obj.graphics(1) = plot(poly1,'FaceColor','r');
                
                poly2 = polyshape([0.5*(pos+0.5*(pos+(pos-length))), 0.5*((pos-length)+0.5*(pos+(pos-length))),...
                0.5*((pos-length)+0.5*(pos+(pos-length))), 0.5*(pos+0.5*(pos+(pos-length)))],...
                [0.5+height1, 0.5+height1, (0.5+height1)+height2, (0.5+height1)+height2]);
                obj.graphics(2) = plot(poly2,'FaceColor','r');
            
                % Draw two wheels
                obj.graphics(3) = viscircles([0.5*((pos-length)+0.5*(pos+(pos-length))), 0.3], radius,'Color','k');
                obj.graphics(4) = viscircles([0.5*(pos+0.5*(pos+(pos-length))), 0.3], radius,'Color','k');

            end
            
        end



        function outputArg = generateNoise(obj,t,dt)

            if obj.vehicleIndex==1
                w = 0; % Leader is not affected by the noise.
            else 
                w = obj.noiseMean + obj.noiseStd.*randn(3,1);
            end

            obj.noise = w;

            %%%% Some old code
%             stepSize = 1;
%             if w < obj.noise - stepSize
%                 w = obj.noise - stepSize;
%             elseif w > obj.noise + stepSize
%                 w = obj.noise + stepSize;
%             end

        end


        function outputArg = computeControlInputs(obj,t,dt)
            % Leader's control (from planned)
            if obj.vehicleIndex==1
                
                if obj.plannedControls(1,1)==t
                    obj.controlInput = obj.plannedControls(1,2);
                    obj.plannedControls = obj.plannedControls(2:end,:);
                else
                    obj.controlInput = 0;
                end
            
            end
        end

        % Update the state values of the system dynamics
        function outputArg = update(obj,t,dt)
            
            A = [0 1 0; 0 0 1; 0 0 0];
            B = [0 0 1]';

            updateValue = A*obj.states + B*obj.controlInput + obj.noise;
            
            newStates = obj.states + dt*(updateValue);
            obj.states = newStates;                     
            
            % Collect all the state points at each step
            obj.stateHistory = [obj.stateHistory, newStates];

        end


    end
end

