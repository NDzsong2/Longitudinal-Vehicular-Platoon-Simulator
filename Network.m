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



        function output = loadPlatoonControllers(obj)
            for k = 1:1:obj.numOfPlatoons

                %% Controller Types:
                % There can be three factors that determines the controller
                % type: (i) Centralized/Decentralized, (ii)
                % Stabilizing/Robust, and (iii) Error Dynamics Type
                
                errorDynamicsType = 2;  % Lets use the error dynamics formulation II for now...
                isCentralized = 1;      % Lets start with focusing on centralized controller synthesis
                isOnlyStabilizing = 1;  % Lets start we just a stabilizing controller (without caring about the disturbance robustness)
                
                if errorDynamicsType == 1           % Error dynamics formulation I
                    if isCentralized == 1           % Centralized
                        if isOnlyStabilizing == 1   % Only Stabilizing
                            status = obj.platoons(k).centralizedStabilizingControllerSynthesis1();
                        else                        % Robust
                            status = obj.platoons(k).centralizedRobustControllerSynthesis1();
                        end
                    else                            % Decentralized
                        if isOnlyStabilizing == 1   % Only Stabilizing
                            status = obj.platoons(k).decentralizedStabilizingControllerSynthesis1();
                        else                        % Robust
                            status = obj.platoons(k).decentralizedRobustControllerSynthesis1();
                        end
                    end
                else                                % Error dynamics formulation II
                    if isCentralized == 1           % Centralized
                        if isOnlyStabilizing == 1   % Only Stabilizing
                            status = obj.platoons(k).centralizedStabilizingControllerSynthesis2();
                        else                        % Robust
                            status = obj.platoons(k).centralizedRobustControllerSynthesis2();
                        end
                    else                            % Decentralized
                        if isOnlyStabilizing == 1   % Only Stabilizing
                            status = obj.platoons(k).decentralizedStabilizingControllerSynthesis2();
                        else                        % Robust
                            status = obj.platoons(k).decentralizedRobustControllerSynthesis2();
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
        
    end
end