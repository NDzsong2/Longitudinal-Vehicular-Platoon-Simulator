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

        function newObj = copy(obj)
            % Create a new object of the same class without using the constructor
            % Initialize arrays for constructor arguments
            parameters = cell(1, obj.numOfPlatoons);
            states = cell(1, obj.numOfPlatoons);
            desiredSeparation = cell(1, obj.numOfPlatoons);
            noiseMean = cell(1, obj.numOfPlatoons);
            noiseStd = cell(1, obj.numOfPlatoons);
            
            for k = 1:obj.numOfPlatoons
                platoon = obj.platoons(k);
                numVehicles = platoon.numOfVehicles;
                
                parameters{k} = zeros(size(platoon.vehicles(1).vehicleParameters, 1), numVehicles);
                states{k} = zeros(size(platoon.vehicles(1).states, 1), numVehicles);
                desiredSeparation{k} = zeros(size(platoon.vehicles(1).desiredSeparation, 1), numVehicles);
                noiseMean{k} = zeros(size(platoon.vehicles(1).noiseMean, 1), numVehicles);
                noiseStd{k} = zeros(size(platoon.vehicles(1).noiseStd, 1), numVehicles);
                
                for i = 1:numVehicles
                    vehicle = platoon.vehicles(i);
                    parameters{k}(:, i) = vehicle.vehicleParameters;
                    states{k}(:, i) = vehicle.states;
                    desiredSeparation{k}(:, i) = vehicle.desiredSeparation;
                    noiseMean{k}(:, i) = vehicle.noiseMean;
                    noiseStd{k}(:, i) = vehicle.noiseStd;
                end
            end
            
            % Create a new object using the constructor with the necessary parameters
            newObj = Network(obj.networkIndex, obj.numOfPlatoons, obj.numOfVehicles, parameters, states, desiredSeparation, noiseMean, noiseStd);
            
            
            % Manually assign all properties
            propertiesList = properties(obj);
            for i = 1:length(propertiesList)
                propName = propertiesList{i};
                if isa(obj.(propName), 'handle')
                    if isobject(obj.(propName)) && numel(obj.(propName)) > 1
                        % If the property is an array of objects, copy each element
                        for j = 1:numel(obj.(propName))
                            newObj.(propName)(j) = obj.(propName)(j).copy();
                        end
                    else
                        % Recursively copy handle objects
                        newObj.(propName) = obj.(propName).copy();
                    end
                else
                    % For value types, directly assign the property
                    newObj.(propName) = obj.(propName);
                end
            end
            
            % Copy the platoons array
            for i = 1:obj.numOfPlatoons
                newObj.platoons(i) = obj.platoons(i).copy();
            end

            if ~isempty(newObj.graphics)
                newObj.graphics = [];
            end
        end

        function obj = removeVehicles(obj, platoonIndex, vehicleIndices, errorDynamicsType)
            % Remove vehicles given their indices and the platoon index
            if platoonIndex > obj.numOfPlatoons || platoonIndex < 1
                disp('Invalid platoon index');
            end
            
            % Remove vehicles using the method in the Platoon class
            obj.platoons(platoonIndex) = obj.platoons(platoonIndex).removeVehicles(vehicleIndices,errorDynamicsType);

            % Update the number of vehicles in the network
            obj.numOfVehicles(platoonIndex) = obj.platoons(platoonIndex).numOfVehicles;

        end

        function outputArg = drawNetwork(obj,figNum)
            figure(figNum); hold on;
            
            if ~isempty(obj.graphics)
                delete(obj.graphics(1));
                delete(obj.graphics(2)); 
                delete(obj.graphics(3)); 
                delete(obj.graphics(4));
                delete(obj.graphics(5)); 
                delete(obj.graphics(6));
            end

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
            totalSqError = zeros(3,1);

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
                totalSqError_k = obj.platoons(k).update(t,dt);
                totalSqError = totalSqError + totalSqError_k;

            end

            % Update the time, error and cost
            costSoFar = obj.cost^2*obj.time;
            % costSoFar = obj.cost^2;
            obj.time = obj.time + dt;

            
            obj.error = sqrt([25;9;1].*totalSqError); %This is 2-norms of three error components
            costSoFar = costSoFar + sum(obj.error.^2)*dt;
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
                % axis([min(posX1,posX2),max(posX1,posX2),posY1,posY2])
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



        %% Controller computation
        function [status, gammaSqVal, timeVals] = loadPlatoonControllers(obj,errorDynamicsType,isCentralized,isDSS,isOnlyStabilizing,gammaSqBar,nuBar,rhoBar,pVals)
            for k = 1:1:obj.numOfPlatoons

                % Controller Types:
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
                    if isCentralized == 1 && ~isDSS % Centralized & Not DSS
                        if isOnlyStabilizing == 1   % Only Stabilizing
                            status = obj.platoons(k).centralizedStabilizingControllerSynthesis2(pVals(k,:));
                        else                        % Robust
                            [status, gammaSqVal, timeVals] = obj.platoons(k).centralizedRobustControllerSynthesis2(pVals(k,:)); % nuBar,rhoBar,gammaSqBar are no longer needed
                        end
                    elseif ~isCentralized == 1 && ~isDSS    % Decentralized & Not DSS
                        if isOnlyStabilizing == 1           % Only Stabilizing
                            status = obj.platoons(k).decentralizedStabilizingControllerSynthesis2(pVals(k,:));
                        else                                % Robust
                            [status, gammaSqVal, timeVals] = obj.platoons(k).decentralizedRobustControllerSynthesis2(pVals(k,:));
                        end
                    elseif ~isCentralized == 1 && isDSS     % Decentralized & DSS
                            status = obj.platoons(k).decentralizedRobustControllerSynthesisDSS2(pVals(k,:));  
                                                            % Robust
                    end
                end
                
                % Success or Failure
                if status == 1
                    % disp(['Synthesis Success at Platoon ',num2str(k),'.']);
                else
                    disp(['Synthesis Failed at Platoon ',num2str(k),'.']);
                end
                
            end
        end
        
        function pVals = optimizeCodesignParameters(obj,isCentralized,isDSS)
            pVals = [];
            for k = 1:1:obj.numOfPlatoons
                pVals_k = obj.platoons(k).optimizeCodesignParameters(isCentralized,isDSS);
                pVals = [pVals; pVals_k];
            end
        end


        %%% Solution evaluation

        function [comCost,robCost] = evaluateTopology(obj,showPlots)
            % Update the states and plot them in real time as an animation
            dt = 0.01;
            tMax = 20; 
            tArray = 0:dt:tMax;
            timeLength = length(tArray);
            
            % Leader's trajectory specification
            tVals = tMax*[0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1];     % Special time instants (inbetween which the acceleration is constant)
            vVals = [0, 10, 20, 20, 30, 30, 20, 10, 20, 30, 30];   % Velocities to be achived at these time instants
            obj.generateLeadersControlProfiles(dt,tVals,vVals);

            % Updating the platoons
            for t = tArray
            
                % Use this update function to achieve the update of the entire program
                obj.update(t,dt);
                
            end
            
            robCost = obj.cost;

            comCost = 0;
            for k = 1:1:obj.numOfPlatoons
                comCost = comCost + obj.platoons(k).evaluateTopology();
            end
            
            if showPlots
                % Plot the state histories
                for k = 1:1:obj.numOfPlatoons
                    C = linspecer(obj.platoons(k).numOfVehicles(1));

                    % figure
                    % % Subplot 1: Position History
                    % % subplot(2,2,1)
                    % axes('NextPlot', 'replacechildren', 'ColorOrder', C);
                    % hold on
                    % plot(tArray, obj.platoons(k).vehicles(1).stateHistory(1,:)', '--', 'LineWidth', 2);
                    % for i = 2:1:obj.platoons(k).numOfVehicles
                    %     plot(tArray, obj.platoons(k).vehicles(i).stateHistory(1,:)', '-', 'LineWidth', 2);
                    % end
                    % set(gca, 'FontSize', 16);
                    % xlabel('$t$/time(s)', 'Interpreter', 'latex', 'FontSize', 24);
                    % ylabel('$x(t)$/position(m)', 'Interpreter', 'latex', 'FontSize', 24);
                    % xlim([0 tMax]);
                    % ylim([-80 300]);
                    % grid on 
                    % 
                    figure
                    % Subplot 2: Position Error History
                    % subplot(2,2,2)
                    axes('NextPlot', 'replacechildren', 'ColorOrder', C);
                    hold on
                    plot(tArray, zeros(1, timeLength), '--', 'LineWidth', 2);
                    for i = 2:1:obj.platoons(k).numOfVehicles
                        plot(tArray, obj.platoons(k).vehicles(i).errorHistory(1,:)', '-', 'LineWidth', 2);
                    end
                    set(gca, 'FontSize', 16);
                    xlabel('$t$/time(s)', 'Interpreter', 'latex', 'FontSize', 24);
                    ylabel('$\tilde{x}(t)$/position-error(m)', 'Interpreter', 'latex', 'FontSize', 24);
                    xlim([0 tMax]);
                    ylim([-2 2]);
                    grid on 
                
                    figure
                    % Subplot 3: Velocity History
                    % subplot(2,2,3)
                    axes('NextPlot', 'replacechildren', 'ColorOrder', C);
                    hold on
                    plot(tArray, obj.platoons(k).vehicles(1).stateHistory(2,:)', '--', 'LineWidth', 2);
                    for i = 2:1:obj.platoons(k).numOfVehicles
                        plot(tArray, obj.platoons(k).vehicles(i).stateHistory(2,:)', '-', 'LineWidth', 2);
                    end
                    set(gca, 'FontSize', 16);
                    xlabel('$t$/time(s)', 'Interpreter', 'latex', 'FontSize', 24);
                    ylabel('$v(t)$/velocity(m/s)', 'Interpreter', 'latex', 'FontSize', 24);
                    xlim([0 tMax]);
                    ylim([-5 45]);
                    grid on 
                
                    figure
                    % Subplot 4: Velocity Error History
                    % subplot(2,2,4)
                    axes('NextPlot', 'replacechildren', 'ColorOrder', C);
                    hold on
                    plot(tArray, zeros(1, timeLength), '--', 'LineWidth', 2, 'DisplayName', ['Veh. ', num2str(1)]);
                    for i = 2:1:obj.platoons(k).numOfVehicles
                        plot(tArray, obj.platoons(k).vehicles(i).errorHistory(2,:)', '-', 'LineWidth', 2, 'DisplayName', ['Veh. ', num2str(i)]);
                    end
                    set(gca, 'FontSize', 16);
                    xlabel('$t$/time(s)', 'Interpreter', 'latex', 'FontSize', 24);
                    ylabel('$\tilde{v}(t)$/velocity-error(m/s)', 'Interpreter', 'latex', 'FontSize', 24);
                    % legend('Location', 'northeast', 'FontSize', 20, 'NumColumns', 2, 'Orientation', 'horizontal');
                    xlim([0 tMax]);
                    ylim([-3 3]);
                    grid on 
                end
            end


            % % Plot the state histories
            % for k = 1:1:obj.numOfPlatoons
            %     C = linspecer(obj.platoons(k).numOfVehicles(1));
            %     figure
            % 
            %     subplot(2,2,1)
            %     axes('NextPlot', 'replacechildren', 'ColorOrder',C);
            %     hold on
            %     plot(tArray,obj.platoons(k).vehicles(1).stateHistory(1,:)','--',LineWidth=2);
            %     for i = 2:1:obj.platoons(k).numOfVehicles
            %         plot(tArray,obj.platoons(k).vehicles(i).stateHistory(1,:)','-',LineWidth=2);
            %     end
            %     set(gca,fontsize=16);
            %     xlabel('$t$/time(s)','Interpreter','latex',fontsize=24)
            %     ylabel('$x(t)$/position(m)','Interpreter','latex',fontsize=24)
            %     % legend(Location="eastoutside")
            %     xlim([0 10]);
            %     ylim([-80 300]);
            %     grid on 
            % 
            % 
            %     % Plot the location error histories
            %     subplot(2,2,2)
            %     axes('NextPlot','replacechildren', 'ColorOrder',C);
            %     hold on
            %     plot(tArray,zeros(1,timeLength),'--',LineWidth=2);
            %     for i = 2:1:obj.platoons(k).numOfVehicles
            %         plot(tArray,obj.platoons(k).vehicles(i).errorHistory(1,:)','-',LineWidth=2);
            %     end
            %     set(gca,fontsize=16);
            %     xlabel('$t$/time(s)','Interpreter','latex',fontsize=24)
            %     ylabel('$\tilde{x}(t)$/position-error(m)','Interpreter','latex',fontsize=24)
            %     % legend(Location="eastoutside")
            %     xlim([0 10]);
            %     ylim([-1 2]);
            %     grid on 
            % 
            % 
            %     subplot(2,2,3)
            %     axes('NextPlot','replacechildren', 'ColorOrder',C);
            %     hold on
            %     plot(tArray,obj.platoons(k).vehicles(1).stateHistory(2,:)','--',LineWidth=2);
            %     for i = 2:1:obj.platoons(k).numOfVehicles
            %         plot(tArray,obj.platoons(k).vehicles(i).stateHistory(2,:)','-',LineWidth=2);
            %     end
            %     set(gca,fontsize=16);
            %     xlabel('$t$/time(s)','Interpreter','latex',fontsize=24)
            %     ylabel('$v(t)$/velocity(m/s)','Interpreter','latex',fontsize=24)
            %     % legend(Location="eastoutside")
            %     xlim([0 10]);
            %     ylim([-5 45]);
            %     grid on 
            % 
            % 
            % 
            %     subplot(2,2,4)
            %     axes('NextPlot','replacechildren', 'ColorOrder',C);
            %     hold on
            %     plot(tArray,zeros(1,timeLength),'--',LineWidth=2,DisplayName=['Veh. ',num2str(1)]);
            %     for i = 2:1:obj.platoons(k).numOfVehicles
            %         plot(tArray,obj.platoons(k).vehicles(i).errorHistory(2,:)','-',LineWidth=2,DisplayName=['Veh. ',num2str(i)]);
            %     end
            %     set(gca,fontsize=16);
            %     xlabel('$t$/time(s)','Interpreter','latex',fontsize=24)
            %     ylabel('$\tilde{v}(t)$/velocity-error(m/s)','Interpreter','latex',fontsize=24)
            %     legend(Location="northeast",FontSize=20,NumColumns=2,Orientation="horizontal")
            %     xlim([0 10]);
            %     ylim([-1.5 2]);
            %     grid on 
            % 
            % 
            % 
            % end

        end
    end
end