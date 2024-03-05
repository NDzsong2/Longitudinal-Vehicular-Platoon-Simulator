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

        % Passivity Indices
        nu
        rho

        % Local Controller Gains
        localControllerGains1 = []
        localControllerGains2 = []

        % Global ControllerGains
        controllerGains1 = []
        controllerGains2 = []
        
        % Data to be distributed (in decentralized schemes)
        dataToBeDistributed
        controllerGainsCollection

        % States
        desiredSeparation    %From the leader
        desiredSeparations   %From all others
        states               % x_ik
        noise                % v_ik
        controlInput
        errors
        outputs

        % state history
        stateHistory = []

        % error history
        errorHistory = []

        % Predefined controls
        plannedControls = [] % matrix of paris [t_i,u_i]

        % GeometricProperties (for plotting)          
        inNeighbors = []
        outNeighbors = []

        % graphicHandles
        graphics = []
        
        % collection of R_i matrix for DSS condition verification
        R_i
    end
    
    methods

        function obj = Vehicle(k,i,parameters,states,desiredSeparation,noiseMean,noiseStd)

            % Constructor
            obj.platoonIndex = k;
            obj.vehicleIndex = i;

            obj.vehicleParameters = parameters;                     %[mass,length,height1,height2]

            obj.states = states;                                    % states of the i^{th} vehicle
            obj.desiredSeparation = desiredSeparation;              % need to track this signal (desired position,velocity and 0 acceleration for i^{th} vehicle)
            
            
            % External disturbances represented by random noise
            obj.noiseMean = noiseMean;
            obj.noiseStd = noiseStd;

            obj.noise = noiseMean + noiseStd*randn(1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Initial controller values
            obj.errors = zeros(3,1);
            obj.controlInput = zeros(1);
            obj.outputs = zeros(1);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            obj.inNeighbors = [];
            obj.outNeighbors = [];
            
        end


        % This function is used to draw a "car" shape object
        function outputArg = drawVehicle(obj,figNum)
            figure(figNum); hold on;
            
            if ~isempty(obj.graphics)
                delete(obj.graphics);
            end

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

            % Vehicle number
            obj.graphics(5) = text(pos,0.2,num2str(obj.vehicleIndex));
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
                
                % Vehicle number
                obj.graphics(5) = text(pos,0,num2str(obj.vehicleIndex));
            end
            
        end



        function outputArg = generateNoise(obj)

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

        function outputArg = computePlatooningErrors1(obj,leaderStates,neighborInformation)

            locationError = 0;
            velocityError = 0;

            for jInd = 1:1:length(obj.inNeighbors)

                j = obj.inNeighbors(jInd);
                k_ijBar = obj.controllerGains1{j};
                d_ij = obj.desiredSeparations(j);
                X_j = neighborInformation{j};

                locationError_j = k_ijBar*(obj.states(1)-X_j(1)-d_ij);
                locationError = locationError + locationError_j;

                velocityError_j = k_ijBar*(obj.states(2)-X_j(2));
                velocityError = velocityError + velocityError_j;
                
            end

            accelerationError = obj.states(3)-leaderStates(3); %a_i-a_0

            newErrors = [locationError;velocityError;accelerationError];

            obj.errors = newErrors;
            obj.errorHistory = [obj.errorHistory, newErrors];

        end

        function outputArg = computePlatooningErrors2(obj,leaderStates)
            separationFromLeader = obj.desiredSeparation; 
            newErrors = obj.states - leaderStates + [separationFromLeader;0;0];
            obj.errors = newErrors;
            obj.errorHistory = [obj.errorHistory, newErrors];
        end


        function outputArg = computeControlInputs1(obj,t)
            
            if obj.vehicleIndex==1  % Leader's control (from planned)
                
                if obj.plannedControls(1,1)==t
                    obj.controlInput = obj.plannedControls(1,2);
                    obj.plannedControls = obj.plannedControls(2:end,:); % Delete the executed planned control
                else
                    obj.controlInput = 0;
                end

            else                    % Followers control (based on errors) under Error-Dynamics - I
                
                i = obj.vehicleIndex;
                L_ii = obj.controllerGains1{i} + obj.localControllerGains1;
                e_i = obj.errors;
                obj.controlInput = L_ii*e_i;

            end
        end

        function outputArg = computeControlInputs2(obj,t,neighborInformation)
            
            if obj.vehicleIndex==1  % Leader's control (from planned)
                
                if obj.plannedControls(1,1)==t
                    obj.controlInput = obj.plannedControls(1,2);
                    obj.plannedControls = obj.plannedControls(2:end,:); % Delete the executed planned control
                else
                    obj.controlInput = 0;
                end

            else                    % Followers control (based on errors) under Error-Dynamics - II
               
                i = obj.vehicleIndex;
                L_ii = obj.controllerGains2{i} + obj.localControllerGains2;
                e_i = obj.errors;
                controlInput = L_ii*e_i;
                for jInd = 1:1:length(obj.inNeighbors)
                    j = obj.inNeighbors(jInd);
                    if j~=1
                        L_ij = obj.controllerGains2{j};
                        e_j = neighborInformation{j};
                        controlInput = controlInput + L_ij*(e_i - e_j);
                    end 
                end
                obj.controlInput = controlInput;
                
            end
        end


        % Update the state values of the system dynamics
        function vehicleError = update(obj,t,dt)
            
            vehicleError = obj.errors;

            A = [0 1 0; 0 0 1; 0 0 0];
            B = [0 0 1]';

            updateValue = A*obj.states + B*obj.controlInput + obj.noise;
            
            newStates = obj.states + dt*(updateValue);
            obj.states = newStates;                     
            
            % Collect all the state points at each step
            obj.stateHistory = [obj.stateHistory, newStates];

        end


        function outputArg = loadPassivityIndices(obj,nu,rho)
            obj.nu = nu;
            obj.rho = rho;
        end

        function status = synthesizeLocalControllers(obj,errorDynamicsType,nuBar,rhoBar)
            % Here we will synthesize the local controllers for local error
            % dynamics to optimize the passivity properties
            
            % Error Dynamics Type
            % When nu = 0, both methods seems to lead to rho = -1/2 (this will not do)
            % When rho = 0, second methods lead to nu = -1/2

            if errorDynamicsType == 1
                A = [0,1,0;0,0,0;0,0,0]; % For error dynamics type 1
            else
                A = [0,1,0;0,0,1;0,0,0]; % For error dynamics type 2   
            end
            B = [0;0;1];
            I = eye(3);
            O = zeros(3);

            rhoBarBar = 1/rhoBar;

            nuHat = -0.01;            % nuBar <= nu <= nuHat < 0
            rhoHat = 0.01;            % 0 <= rhoHat <= rho <= rhoBar ( or 0 <= rhoBarBar <= rhoBar <= rhoHatBar ) 
            rhoHatBar = 1/rhoHat;

            % Set up the LMI problem
            solverOptions = sdpsettings('solver','mosek','verbose',0);            
            P = sdpvar(3,3,'symmetric'); 
            K = sdpvar(1,3,'full'); 

            rhoTilde = sdpvar(1,1,'full'); %Representing: 1/rho
            nu = sdpvar(1,1,'full');

            % Basic Constraints
            con1 = P >= 0;
            %%con2 = trace(P) == 1; % In this setting this is not required actually
           
            % Approach 4 with rho = prespecified, nu < 0 and nu is free to maximize
            DMat = [rhoTilde*I];
            MMat = [P, O];
            ThetaMat = [-A*P-P*A'-B*K-K'*B', -I+0.5*P; -I+0.5*P, -nu*I];
            W = [DMat, MMat; MMat', ThetaMat];
            con3 = W >= 0;
            
            % Some modesty constraints on resulting nu and rho from the local design
            con4 = nu >= nuBar;             % -8.1
            con5 = rhoTilde >= rhoBarBar;     % 1/4.1

            con6 = nu <= nuHat;
            con7 = rhoTilde <= rhoHatBar;

            % To help the global design (\gammaSqBar=10, p_i = 1/N)
            gammaSqBar = 10;
            con8 = nu >= -gammaSqBar/(1/5) ;     % nu >= -50
            con9 = rhoTilde <= 4*gammaSqBar/(1/5);  % rhoBar >= 200

            % Total Cost and Constraints
            cons = [con1,con3,con4,con5,con6,con7,con8,con9];
            %costFun =  0*(-nu + rhoBar);           % For stabilizing, set coefficient to 0
            %costFun = 0.0000001*(-nu + rhoBar);    % Otherwise set to 0.0000001.
            costFun = 0.0000001*(- nu + rhoTilde);

            % Solution
            sol = optimize(cons,costFun,solverOptions);
            status = sol.problem == 0; % sol.info;

            PVal = value(P)
            KVal = value(K)
            LVal = KVal/PVal
            
            nuVal = value(nu)
            rhoVal = 1/value(rhoTilde)

            disp('TestVal = -nu,rhoTilde/4')
            val = [-value(nu),  value(rhoTilde)/4]
            
            % Updating the information
            obj.nu = nuVal;
            obj.rho = rhoVal;

            obj.localControllerGains1 = LVal; % Here we need \bar{k}_{i0}^{Local} = 1
            obj.localControllerGains2 = LVal; 

            if status == 1
                disp(['Synthesis Success at Vehicle ',num2str(obj.vehicleIndex),'.'])
            else
                disp(['Synthesis Failed at Vehicle ',num2str(obj.vehicleIndex),'.'])
            end
            
        end

        %% Parametrized Approach to Synthesize Local Controllers (while also assisting synthesis of global controllers)
        function [status,PVal,KVal,LVal,nuVal,rhoVal,gammaSqVal] = synthesizeLocalControllersParameterized(obj,errorDynamicsType,pVal)
            % Here we will synthesize the local controllers for local error
            % dynamics to optimize the passivity properties
            % This is the true local controller synthesis for a given p_i value
        
            if errorDynamicsType == 1
                A = [0,1,0;0,0,0;0,0,0]; % For error dynamics type 1
            else
                A = [0,1,0;0,0,1;0,0,0]; % For error dynamics type 2   
            end
            B = [0;0;1];
            I = eye(3);
            O = zeros(3);
        
            % Set up the LMI problem
            solverOptions = sdpsettings('solver','mosek','verbose',0);            
            P = sdpvar(3,3,'symmetric'); 
            K = sdpvar(1,3,'full'); 
        
            nu = sdpvar(1,1,'full');
            rhoTilde = sdpvar(1,1,'full'); % Representing: 1/rho
            
            gammaSq = sdpvar(1,1,'full');
        
            % For: nuBar < nu < nuHat < 0
            nuBar = -gammaSq/pVal;
        
            % For: 0 < rhoHat1,rhoHat2 < rho < rhoBar
            % For: 0 < rhoTildeHat < rhoTilde < rhoTildeBar1,rhoTildeBar2
            rhoTildeBar1 = 4*gammaSq/pVal;
            rhoTildeBar2 = pVal;
        
            % Basic Constraints
            con1 = P >= 0;
            
            % Approach 4 with rho = prespecified, nu < 0 and nu is free to maximize
            DMat = [rhoTilde*I];
            MMat = [P, O];
            ThetaMat = [-A*P-P*A'-B*K-K'*B', -I+0.5*P; -I+0.5*P, -nu*I];
            W = [DMat, MMat; MMat', ThetaMat];
            con2 = W >= 0;
            
            %%Constraints on resulting nu and rho from the local design 
            % nuBar < nu < nuHat < 0 
            con3 = nu >= nuBar;              % Helps global design
            
            % 0 < rhoTildeHat < rhoTilde < rhoTildeBar1,rhoTildeBar2  
            con4 = rhoTilde <= rhoTildeBar1;    % Helps global design
            con5 = rhoTilde <= rhoTildeBar2;    % Helps global design
            
            % Total Cost and Constraints
            cons = [con1,con2,con3,con4,con5];
            %costFun =  0*(-nu + rhoBar);           % For stabilizing, set coefficient to 0
            %costFun = 0.0000001*(-nu + rhoBar);    % Otherwise set to 0.0000001.
            costFun = 0*gammaSq;
        
            % Solution
            sol = optimize(cons,costFun,solverOptions);
            status = sol.problem == 0; % sol.info;
        
            PVal = value(P)   % This is the P_i value obtained from solving the local control synthesis
            KVal = value(K)   % This is \tilde{L}_{ii}
            LVal = KVal/PVal  % KVal is L and LVal is K in our paper. This LVal is the local controller gain \bar{L}_{ii}
            
            nuVal = value(nu);
            rhoVal = 1/value(rhoTilde);
            gammaSqVal = value(gammaSq);
        
            % Updating the information
            obj.nu = nuVal;
            obj.rho = rhoVal;

            obj.localControllerGains1 = LVal; % Here we need \bar{k}_{i0}^{Local} = 1
            obj.localControllerGains2 = LVal; 

%             if status == 1
%                 disp(['Synthesis Success at Vehicle ',num2str(obj.vehicleIndex),' with gammaSq=',num2str(value(gammaSqVal)),'.'])
%             else
%                 disp(['Synthesis Failed at Vehicle ',num2str(obj.vehicleIndex),'.'])
%             end
            
        end


        %% Decentralized Stabilizing Controller Synthesis (Error Dynamics I)
        function [isStabilizable,K_ii,K_ijVals,K_jiVals] = stabilizingControllerSynthesis1(obj, previousSubsystems, subsystems, nuBar, rhoBar)

            i = length(previousSubsystems)+1;
            iInd = obj.vehicleIndex-1;
            disp(['Stabilizing at: ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
            
            status = obj.synthesizeLocalControllers(1,nuBar,rhoBar);
                        
            % Seting up the LMI problem
            solverOptions = sdpsettings('solver','mosek','verbose',0);
            isSoft = 1; % Whether to use a soft or hard graph constraint
            normType = 2; % Type of the norm to be used in the LMI

            I_n = eye(3);
            O_n = zeros(3);

            nu_i = obj.nu;
            rho_i = obj.rho;
            X_i_11 = -nu_i*I_n;
            X_i_12 = 0.5*I_n;
            % X_i_21 = X_i_12';
            X_i_22 = -rho_i*I_n;
            X_ii_12 = X_i_11\X_i_12;
            X_ii_21 = X_ii_12';

            obj.dataToBeDistributed.X = X_ii_12;

            null_ii = [1,1,1; 1,1,0; 0,0,0];
            % adj_ii = [0,0,0; 0,0,1; 1,1,1];
            cost_ii = 1*[0,0,0; 0,0,1; 1,1,1];
            
            if isempty(previousSubsystems)
                
                % The subsystem only need to test W_ii > 0
                p_i = sdpvar(1,1);
                Q_ii = sdpvar(3,3,'full');
                
                con1 = p_i >= 0;

                Theta_ii = - X_ii_21*Q_ii - Q_ii'*X_ii_12 - p_i*X_i_22;
                W_ii = [p_i*X_i_11, Q_ii; Q_ii', Theta_ii];
                con2 = W_ii >= 0;
            
                con3 = Q_ii.*(null_ii==1)==O_n;

                costFun = norm(Q_ii.*cost_ii,normType);

                sol = optimize([con1,con2,con3],costFun,solverOptions);
                isStabilizable = sol.problem==0;

                p_iVal = value(p_i);
                Q_iiVal = value(Q_ii);
                W_iiVal = value(W_ii);
                tildeW_i = W_iiVal; % Note that, here, \tilde{W}_ii = W_ii = \tilde{W}_i. This also needs to be stored

                if abs(det(tildeW_i))<0.000000001
                    disp("Error: det(tildeW_ii) low");
                    isStabilizable = 0;
                end
                
                K_ii = (p_iVal*X_i_11)\Q_iiVal;
                obj.controllerGainsCollection.decenStabCont1{iInd} = K_ii;

                K_jiVals = [];
                K_ijVals = [];
                
                obj.dataToBeDistributed.P = p_iVal;
                obj.dataToBeDistributed.tildeW = tildeW_i;  % Storing
                
                disp(['Data saved ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
                if ~isStabilizable
                    disp(['Not stabilizable at: ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
                end

            else
                % This subsystem has to talk with all the previosSubsystems
                % tildeW_ii > 0 iff [M_i, W_i'; W_i, W_ii] > 0 is required where 
                % M_i =
                % inv((scriptD_i*scriptA_i^T)^{-1}*(scriptD_i)*(scriptD_i*scriptA_i^T)^{-1}') = scriptA_i*scriptD_i*scriptA_i'

                % M_i term
                blockSize = 3; 
                scriptA_i = [];
                scriptD_i = [];
                for j = 1:1:length(previousSubsystems)
                    jInd = previousSubsystems(j);

                    % Getting stored info from jInd to create \mathcal{A}_i and \mathcal{D}_i matrices (their j-th columns)
                    tildeW_j = subsystems(jInd).dataToBeDistributed.tildeW;

                    Z = zeros(blockSize, blockSize*(i-1-j)); % (i-1)-j blocks of blockSize X blockSize zero matrices
                    z = zeros(blockSize, blockSize*(j-1));
                    if j==1
                        tildeW_jj = tildeW_j;                    
                        scriptA_i = [tildeW_jj, Z];         % The first row of \mathcal{A}_i.
                        scriptD_i = [inv(tildeW_jj), Z];    % The first row of \mathcal{D}_i.
                    else
                        tildeW_jj = tildeW_j(:,blockSize*(j-1)+1:blockSize*j);   % last blockSizeXblockSize block in the row block vector
                        tildeW_j  = tildeW_j(:,1:blockSize*(j-1));                % first (j-1) blocks in the row block vector
                        scriptA_i = [scriptA_i; [tildeW_j, tildeW_jj, Z]];    % The j-th row of \mathcal{A}_i.
                        scriptD_i = [scriptD_i; [z, inv(tildeW_jj), Z]];         % The j-th row of \mathcal{D}_i.
                    end                    
                end
                disp(['Data at ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
                scriptA_i
                scriptD_i                

                M1_i = inv(scriptD_i*scriptA_i');
                % M_i = inv(M1_i*scriptD_i*M1_i') % THis fills (i-1)x(i-1) blocks in the LMI
                M_i = scriptA_i*scriptD_i*scriptA_i';

                if issymmetric(scriptD_i) & issymmetric(scriptA_i) & ~issymmetric(M_i)
                    tf = norm(M_i-M_i.',inf);
                    disp(['Symmetry Error !!! Magnitude:',num2str(tf)]);
                    % M_i
                    M_i = 0.5*(M_i + M_i');
                end
                
                % W_ii and W_i terms
                p_i = sdpvar(1,1);
                Q_ii = sdpvar(3,3,'full');

                Theta_ii = - X_ii_21*Q_ii - Q_ii'*X_ii_12 - p_i*X_i_22;
                W_ii = [p_i*X_i_11, Q_ii; Q_ii', Theta_ii];

                W_i = [];
                for j = 1:1:length(previousSubsystems)
                    jInd = previousSubsystems(j);

                    null_ij{j} = [1,1,1; 1,1,0; 1,1,1];
                    null_ji{j} = [1,1,1; 1,1,0; 1,1,1];

                    if any(obj.inNeighbors==jInd) % iInd <----- jInd
                        adj_ij{j} = [0,0,0; 0,0,1; 0,0,0]; 
                        cost_ij{j} = 0.01*[0,0,0; 0,0,1; 0,0,0];
                    else
                        adj_ij{j} = [0,0,0; 0,0,0; 0,0,0];
                        cost_ij{j} = 1*[0,0,0; 0,0,1; 0,0,0];
                    end

                    if any(obj.outNeighbors==jInd) % iInd -----> jInd
                        adj_ji{j} = [0,0,0; 0,0,1; 0,0,0];
                        cost_ji{j} = 0.01*[0,0,0; 0,0,1; 0,0,0];
                    else
                        adj_ji{j} = [0,0,0; 0,0,0; 0,0,0];
                        cost_ji{j} = 1*[0,0,0; 0,0,1; 0,0,0];
                    end

                    Q_ij{j} = sdpvar(3,3,'full');
                    Q_ji{j} = sdpvar(3,3,'full');

                    X_jj_12 = subsystems(jInd).dataToBeDistributed.X;
                    
                    Theta_ij = - X_ii_21*Q_ij{j} - Q_ji{j}'*X_jj_12;
                    
                    W_ij = [O_n, Q_ij{j}; Q_ji{j}', Theta_ij];
                    W_i = [W_i, W_ij];
                end

                con1 = p_i >= 0;
                con2 = [M_i, W_i';W_i, W_ii] >= 0;
                
                con3 = [];
                con4 = [];
                costFun = 0;
                for j = 1:1:length(previousSubsystems)
                    con3_ij = Q_ij{j}.*(null_ij{j}==1)==O_n;
                    con3_ji = Q_ji{j}.*(null_ji{j}==1)==O_n;
                    con3 = [con3, con3_ij, con3_ji];
                    
                    con4_ij = Q_ij{j}.*(adj_ij{j}==0)==O_n;
                    con4_ji = Q_ji{j}.*(adj_ji{j}==0)==O_n;
                    con4 = [con4, con4_ij, con4_ji]; 

                    costFun = costFun + norm(Q_ij{j}.*cost_ij{j},normType) + norm(Q_ji{j}.*cost_ji{j},normType);
                end

                con3_ii = Q_ii.*(null_ii==1)==O_n;
                con3 = [con3, con3_ii];
                
                costFun = costFun + norm(Q_ii.*cost_ii,normType);
                
                if isSoft
                    cons = [con1,con2,con3];
                else
                    cons = [con1,con2,con3,con4];
                end

                sol = optimize(cons,costFun,solverOptions);
                isStabilizable = sol.problem==0;

                p_iVal = value(p_i);
                Q_iiVal = value(Q_ii);
                W_iVal = value(W_i);
                W_iiVal = value(W_ii);

                K_ii = (p_iVal*X_i_11)\Q_iiVal;
                obj.controllerGainsCollection.decenStabCont1{iInd} = K_ii;
                for j = 1:1:length(previousSubsystems)
                    jInd = previousSubsystems(j);
                    
                    Q_ijVal = value(Q_ij{j});
                    Q_jiVal = value(Q_ji{j});
                    p_jVal = subsystems(jInd).dataToBeDistributed.P;
                    X_j_11 = -subsystems(jInd).nu*I_n;
                                        
                    K_ij = (p_iVal*X_i_11)\Q_ijVal;
                    K_ijVals{jInd} = K_ij;
                    obj.controllerGainsCollection.decenStabCont1{jInd} = K_ij;
                                        
                    K_ji = (p_jVal*X_j_11)\Q_jiVal
                    K_jiVals{jInd} = K_ji; % these values will be loaded outside the function
                end  
                
                % Need to compute \tilede{W}_i and \tilde{W}_{ii} for storage
                tildeW_i = W_iVal*M1_i;
                tildeW_ii = W_iiVal - tildeW_i*scriptD_i*tildeW_i'; % Note that here, \tilde{W}_ii, W_ii, \tilde{W}_i are different.

                tildeW_i = [tildeW_i, tildeW_ii];

                if abs(det(tildeW_ii))<0.000000001
                    disp("Error: det(tildeW_ii) low");
                    isStabilizable = 0;
                end

                obj.dataToBeDistributed.P = p_iVal;         % Storing
                obj.dataToBeDistributed.tildeW = tildeW_i; % Storing
                disp(['Data saved ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
                if ~isStabilizable
                    disp(['LMI is not feasible at: ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
                end
                
            end
        end


        %% Decentralized Robust Controller Synthesis (Error Dynamics I)
        function [isRobustStabilizable,K_ii,K_ijVals,K_jiVals] = robustControllerSynthesis1(obj, previousSubsystems, subsystems, nuBar, rhoBar, gammaSqBar)

            i = length(previousSubsystems)+1;
            iInd = obj.vehicleIndex-1;
            disp(['Stabilizing at: ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
            
            status = obj.synthesizeLocalControllers(1,nuBar,rhoBar);
                        
            % Seting up the LMI problem
            solverOptions = sdpsettings('solver','mosek','verbose',0);
            isSoft = 1; % Whether to use a soft or hard graph constraint
            normType = 2; % Type of the norm to be used in the LMI

            I_n = eye(3);
            O_n = zeros(3);

            nu_i = obj.nu;
            rho_i = obj.rho;
            X_i_11 = -nu_i*I_n;
            X_i_12 = 0.5*I_n;
            % X_i_21 = X_i_12';
            X_i_22 = -rho_i*I_n;
            X_ii_12 = X_i_11\X_i_12;
            X_ii_21 = X_ii_12';

            obj.dataToBeDistributed.X = X_ii_12;

            null_ii = [1,1,1; 1,1,0; 0,0,0];
            % adj_ii = [0,0,0; 0,0,1; 1,1,1];
            cost_ii = 1*[0,0,0; 0,0,1; 1,1,1];

            if isempty(previousSubsystems)
                
                % The subsystem only need to test W_ii > 0
                p_i = sdpvar(1,1);
                Q_ii = sdpvar(3,3,'full');
                gammaSq_i = sdpvar(1,1);
                
                con1 = p_i >= 0;
                con2 = gammaSq_i >= 0;
                con3 = gammaSq_i <= gammaSqBar;

%                 DMat = [X_p_11, O; O, I];
%                 MMat = [Q, X_p_11; I, O];
%                 ThetaMat = [-X_21*Q-Q'*X_12-X_p_22, -X_p_21; -X_p_12, gammaSq*I];
%                 con4 = [DMat, MMat; MMat', ThetaMat] >= 0; % The real one  
                
                DMat_ii = [p_i*X_i_11, O_n; O_n, I_n];
                MMat_ii = [Q_ii, p_i*X_i_11; I_n, O_n]; 
                TMat_ii = [-X_ii_21*Q_ii-Q_ii'*X_ii_12-p_i*X_i_22, -p_i*X_i_21; -p_i*X_i_12, gammaSq_i*I_n];
                W_ii = [DMat_ii, MMat_ii; MMat_ii', TMat_ii];

                con4 = W_ii >= 0;

                con5 = Q_ii.*(null_ii==1)==O_n;

                costFun = 1*norm(Q_ii.*cost_ii,normType) + 1*gammaSq;

                sol = optimize([con1,con2,con3,con4,con5],costFun,solverOptions);
                isRobustStabilizable = sol.problem==0;

                p_iVal = value(p_i);
                Q_iiVal = value(Q_ii);
                gammaSq_iVal = value(gammaSq_i);
                W_iiVal = value(W_ii);
                tildeW_i = W_iiVal; % Note that, here, \tilde{W}_ii = W_ii = \tilde{W}_i. This also needs to be stored

                if abs(det(tildeW_i))<0.000000001
                    disp("Error: det(tildeW_ii) low");
                    isRobustStabilizable = 0;
                end
                
                K_ii = (p_iVal*X_i_11)\Q_iiVal;
                obj.controllerGainsCollection.decenRobustCont1{iInd} = K_ii;

                K_jiVals = [];
                K_ijVals = [];
                
                obj.dataToBeDistributed.P = p_iVal;
                obj.dataToBeDistributed.gammaSq = gammaSq_iVal;
                obj.dataToBeDistributed.tildeW = tildeW_i;  % Storing
                
                disp(['Data saved ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
                if ~isRobustStabilizable
                    disp(['Not stabilizable at: ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
                end

            else
                % This subsystem has to talk with all the previosSubsystems
                % tildeW_ii > 0 iff [M_i, W_i'; W_i, W_ii] > 0 is required where 
                % M_i =
                % inv((scriptD_i*scriptA_i^T)^{-1}*(scriptD_i)*(scriptD_i*scriptA_i^T)^{-1}') = scriptA_i*scriptD_i*scriptA_i'

                % M_i term
                blockSize = 3; 
                scriptA_i = [];
                scriptD_i = [];
                for j = 1:1:length(previousSubsystems)
                    jInd = previousSubsystems(j);

                    % Getting stored info from jInd to create \mathcal{A}_i and \mathcal{D}_i matrices (their j-th columns)
                    tildeW_j = subsystems(jInd).dataToBeDistributed.tildeW;

                    Z = zeros(blockSize, blockSize*(i-1-j)); % (i-1)-j blocks of blockSize X blockSize zero matrices
                    z = zeros(blockSize, blockSize*(j-1));
                    if j==1
                        tildeW_jj = tildeW_j;                    
                        scriptA_i = [tildeW_jj, Z];         % The first row of \mathcal{A}_i.
                        scriptD_i = [inv(tildeW_jj), Z];    % The first row of \mathcal{D}_i.
                    else
                        tildeW_jj = tildeW_j(:,blockSize*(j-1)+1:blockSize*j);   % last blockSizeXblockSize block in the row block vector
                        tildeW_j  = tildeW_j(:,1:blockSize*(j-1));                % first (j-1) blocks in the row block vector
                        scriptA_i = [scriptA_i; [tildeW_j, tildeW_jj, Z]];    % The j-th row of \mathcal{A}_i.
                        scriptD_i = [scriptD_i; [z, inv(tildeW_jj), Z]];         % The j-th row of \mathcal{D}_i.
                    end                    
                end
                disp(['Data at ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
                scriptA_i
                scriptD_i                

                M1_i = inv(scriptD_i*scriptA_i');
                % M_i = inv(M1_i*scriptD_i*M1_i') % THis fills (i-1)x(i-1) blocks in the LMI
                M_i = scriptA_i*scriptD_i*scriptA_i';

                if issymmetric(scriptD_i) & issymmetric(scriptA_i) & ~issymmetric(M_i)
                    tf = norm(M_i-M_i.',inf);
                    disp(['Symmetry Error !!! Magnitude:',num2str(tf)]);
                    % M_i
                    M_i = 0.5*(M_i + M_i');
                end
                
                % W_ii and W_i terms
                p_i = sdpvar(1,1);
                Q_ii = sdpvar(3,3,'full');
                gammaSq_i = sdpvar(1,1);

                con1 = p_i >= 0;
                con2 = gammaSq_i >= 0;
                con3 = gammaSq_i <= gammaSqBar;
              
                DMat_ii = [p_i*X_i_11, O_n; O_n, I_n];
                MMat_ii = [Q_ii, p_i*X_i_11; I_n, O_n]; 
                TMat_ii = [-X_ii_21*Q_ii-Q_ii'*X_ii_12-p_i*X_i_22, -p_i*X_i_21; -p_i*X_i_12, gammaSq_i*I_n];
                W_ii = [DMat_ii, MMat_ii; MMat_ii', TMat_ii];

                W_i = [];
                for j = 1:1:length(previousSubsystems)
                    jInd = previousSubsystems(j);

                    null_ij{j} = [1,1,1; 1,1,0; 1,1,1];
                    null_ji{j} = [1,1,1; 1,1,0; 1,1,1];

                    if any(obj.inNeighbors==jInd) % iInd <----- jInd
                        adj_ij{j} = [0,0,0; 0,0,1; 0,0,0]; 
                        cost_ij{j} = 0.01*[0,0,0; 0,0,1; 0,0,0];
                    else
                        adj_ij{j} = [0,0,0; 0,0,0; 0,0,0];
                        cost_ij{j} = 1*[0,0,0; 0,0,1; 0,0,0];
                    end

                    if any(obj.outNeighbors==jInd) % iInd -----> jInd
                        adj_ji{j} = [0,0,0; 0,0,1; 0,0,0];
                        cost_ji{j} = 0.01*[0,0,0; 0,0,1; 0,0,0];
                    else
                        adj_ji{j} = [0,0,0; 0,0,0; 0,0,0];
                        cost_ji{j} = 1*[0,0,0; 0,0,1; 0,0,0];
                    end

                    Q_ij{j} = sdpvar(3,3,'full');
                    Q_ji{j} = sdpvar(3,3,'full');

                    X_jj_12 = subsystems(jInd).dataToBeDistributed.X;
                    
                    DMat_ij = [O_n, O_n; O_n, O_n];
                    MMat_ij = [Q_ij{j}, O_n; O_n, O_n];
                    MMat_ji = [Q_ji{j}, O_n; O_n, O_n];
                    
                    TMat_ij = [-X_ii_21*Q_ij{j}-Q_ji{j}'*X_jj_12, O_n; O_n, O_n];
                    W_ij = [DMat_ij, MMat_ij; MMat_ji', TMat_ij];
                    W_i = [W_i, W_ij];
                end

                con4 = [M_i, W_i';W_i, W_ii] >= 0;
                
                con5 = [];
                con6 = [];
                costFun = 0;
                for j = 1:1:length(previousSubsystems)
                    con5_ij = Q_ij{j}.*(null_ij{j}==1)==O_n;
                    con5_ji = Q_ji{j}.*(null_ji{j}==1)==O_n;
                    con5 = [con5, con5_ij, con5_ji];
                    
                    con6_ij = Q_ij{j}.*(adj_ij{j}==0)==O_n;
                    con6_ji = Q_ji{j}.*(adj_ji{j}==0)==O_n;
                    con6 = [con6, con6_ij, con6_ji]; 

                    costFun = costFun + norm(Q_ij{j}.*cost_ij{j},normType) + norm(Q_ji{j}.*cost_ji{j},normType);
                end

                con5_ii = Q_ii.*(null_ii==1)==O_n;
                con5 = [con5, con5_ii];
                
                costFun = costFun + norm(Q_ii.*cost_ii,normType);
                
                if isSoft
                    cons = [con1,con2,con3,con4,con5];
                else
                    cons = [con1,con2,con3,con4,con5,con6];
                end

                sol = optimize(cons,costFun,solverOptions);
                isRobustStabilizable = sol.problem==0;

                p_iVal = value(p_i);
                Q_iiVal = value(Q_ii);
                gammaSq_iVal = value(gammaSq_i);
                W_iVal = value(W_i);
                W_iiVal = value(W_ii);

                K_ii = (p_iVal*X_i_11)\Q_iiVal;
                obj.controllerGainsCollection.decenStabCont1{iInd} = K_ii;
                for j = 1:1:length(previousSubsystems)
                    jInd = previousSubsystems(j);
                    
                    Q_ijVal = value(Q_ij{j});
                    Q_jiVal = value(Q_ji{j});
                    p_jVal = subsystems(jInd).dataToBeDistributed.P;
                    X_j_11 = -subsystems(jInd).nu*I_n;
                          
                    K_ij = (p_iVal*X_i_11)\Q_ijVal;
                    K_ijVals{jInd} = K_ij;
                    obj.controllerGainsCollection.decenStabCont1{jInd} = K_ij;
                                        
                    K_ji = (p_jVal*X_j_11)\Q_jiVal;
                    K_jiVals{jInd} = K_ji; % these values will be loaded outside the function
                end
                
                % Need to compute \tilede{W}_i and \tilde{W}_{ii} for storage
                tildeW_i = W_iVal*M1_i;
                tildeW_ii = W_iiVal - tildeW_i*scriptD_i*tildeW_i'; % Note that here, \tilde{W}_ii, W_ii, \tilde{W}_i are different.
                tildeW_i = [tildeW_i, tildeW_ii];

                if abs(det(tildeW_ii))<0.000000001
                    disp("Error: det(tildeW_ii) low");
                    isRobustStabilizable = 0;
                end

                obj.dataToBeDistributed.P = p_iVal;             
                obj.dataToBeDistributed.gammaSq = gammaSq_iVal;
                obj.dataToBeDistributed.tildeW = tildeW_i;      % Storing

                disp(['Data saved ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
                if ~isRobustStabilizable
                    disp(['LMI is not feasible at: ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
                end

            end
            %%%% Revised upto this
        end


        %% Decentralized Stabilizing Controller Synthesis (Error Dynamics II)
        function [isStabilizable,K_ii,K_ijVals,K_jiVals] = stabilizingControllerSynthesis2(obj, previousSubsystems, subsystems, nuBar, rhoBar)
            
            %%%% Revised upto this
            i = length(previousSubsystems)+1;
            iInd = obj.vehicleIndex-1;
            disp(['Stabilizing at: ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
            
            status = obj.synthesizeLocalControllers(2,nuBar,rhoBar);
                        
            % Seting up the LMI problem
            solverOptions = sdpsettings('solver','mosek','verbose',0);
            isSoft = 1; % Whether to use a soft or hard graph constraint
            normType = 2; % Type of the norm to be used in the LMI

            I_n = eye(3);
            O_n = zeros(3);

            nu_i = obj.nu;
            rho_i = obj.rho;
            X_i_11 = -nu_i*I_n;
            X_i_12 = 0.5*I_n;
            % X_i_21 = X_i_12';
            X_i_22 = -rho_i*I_n;
            X_ii_12 = X_i_11\X_i_12;
            X_ii_21 = X_ii_12';

            obj.dataToBeDistributed.X = X_ii_12;

            null_ii = [1,1,1; 1,1,1; 0,0,0];
            % adj_ii = [0,0,0; 0,0,0; 1,1,1];
            cost_ii = 1*[0,0,0; 0,0,0; 1,1,1];
            
            if isempty(previousSubsystems)
                
                % The subsystem only need to test W_ii > 0
                p_i = sdpvar(1,1);
                Q_ii = sdpvar(3,3,'full');
                
                con1 = p_i >= 0;

                Theta_ii = - X_ii_21*Q_ii - Q_ii'*X_ii_12 - p_i*X_i_22;
                W_ii = [p_i*X_i_11, Q_ii; Q_ii', Theta_ii];
                con2 = W_ii >= 0;
            
                con3 = Q_ii.*(null_ii==1)==O_n;

                costFun = norm(Q_ii.*cost_ii,normType);

                sol = optimize([con1,con2,con3],costFun,solverOptions);
                isStabilizable = sol.problem==0;

                p_iVal = value(p_i);
                Q_iiVal = value(Q_ii);
                W_iiVal = value(W_ii);
                tildeW_i = W_iiVal; % Note that, here, \tilde{W}_ii = W_ii = \tilde{W}_i. This also needs to be stored

                if abs(det(tildeW_i))<0.000000001
                    disp("Error: det(tildeW_ii) low");
                    isStabilizable = 0;
                end
                
                K_ii = (p_iVal*X_i_11)\Q_iiVal;
                obj.controllerGainsCollection.decenStabCont1{iInd} = K_ii;

                K_jiVals = [];
                K_ijVals = [];
                
                obj.dataToBeDistributed.P = p_iVal;
                obj.dataToBeDistributed.tildeW = tildeW_i;  % Storing
                
                disp(['Data saved ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
                if ~isStabilizable
                    disp(['Not stabilizable at: ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
                end

            else
                % This subsystem has to talk with all the previosSubsystems
                % tildeW_ii > 0 iff [M_i, W_i'; W_i, W_ii] > 0 is required where 
                % M_i =
                % inv((scriptD_i*scriptA_i^T)^{-1}*(scriptD_i)*(scriptD_i*scriptA_i^T)^{-1}') = scriptA_i*scriptD_i*scriptA_i'

                % M_i term
                blockSize = 3; 
                scriptA_i = [];
                scriptD_i = [];
                for j = 1:1:length(previousSubsystems)
                    jInd = previousSubsystems(j);

                    % Getting stored info from jInd to create \mathcal{A}_i and \mathcal{D}_i matrices (their j-th columns)
                    tildeW_j = subsystems(jInd).dataToBeDistributed.tildeW;

                    Z = zeros(blockSize, blockSize*(i-1-j)); % (i-1)-j blocks of blockSize X blockSize zero matrices
                    z = zeros(blockSize, blockSize*(j-1));
                    if j==1
                        tildeW_jj = tildeW_j;                    
                        scriptA_i = [tildeW_jj, Z];         % The first row of \mathcal{A}_i.
                        scriptD_i = [inv(tildeW_jj), Z];    % The first row of \mathcal{D}_i.
                    else
                        tildeW_jj = tildeW_j(:,blockSize*(j-1)+1:blockSize*j);   % last blockSizeXblockSize block in the row block vector
                        tildeW_j  = tildeW_j(:,1:blockSize*(j-1));                % first (j-1) blocks in the row block vector
                        scriptA_i = [scriptA_i; [tildeW_j, tildeW_jj, Z]];    % The j-th row of \mathcal{A}_i.
                        scriptD_i = [scriptD_i; [z, inv(tildeW_jj), Z]];         % The j-th row of \mathcal{D}_i.
                    end                    
                end
                disp(['Data at ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
                scriptA_i
                scriptD_i                

                M1_i = inv(scriptD_i*scriptA_i');
                % M_i = inv(M1_i*scriptD_i*M1_i') % THis fills (i-1)x(i-1) blocks in the LMI
                M_i = scriptA_i*scriptD_i*scriptA_i';

                if issymmetric(scriptD_i) & issymmetric(scriptA_i) & ~issymmetric(M_i)
                    tf = norm(M_i-M_i.',inf);
                    disp(['Symmetry Error !!! Magnitude:',num2str(tf)]);
                    % M_i
                    M_i = 0.5*(M_i + M_i');
                end
                
                % W_ii and W_i terms
                p_i = sdpvar(1,1);
                Q_ii = sdpvar(3,3,'full');

                Theta_ii = - X_ii_21*Q_ii - Q_ii'*X_ii_12 - p_i*X_i_22;
                W_ii = [p_i*X_i_11, Q_ii; Q_ii', Theta_ii];

                W_i = [];
                for j = 1:1:length(previousSubsystems)
                    jInd = previousSubsystems(j);

                    null_ij{j} = [1,1,1; 1,1,1; 0,0,0];
                    null_ji{j} = [1,1,1; 1,1,1; 0,0,0];

                    if any(obj.inNeighbors==jInd) % iInd <----- jInd
                        adj_ij{j} = [0,0,0; 0,0,0; 1,1,1]; 
                        cost_ij{j} = 0.01*[0,0,0; 0,0,0; 1,1,1];
                    else
                        adj_ij{j} = [0,0,0; 0,0,0; 0,0,0];
                        cost_ij{j} = 1*[0,0,0; 0,0,0; 1,1,1];
                    end

                    if any(obj.outNeighbors==jInd) % iInd -----> jInd
                        adj_ji{j} = [0,0,0; 0,0,0; 1,1,1];
                        cost_ji{j} = 0.01*[0,0,0; 0,0,0; 1,1,1];
                    else
                        adj_ji{j} = [0,0,0; 0,0,0; 0,0,0];
                        cost_ji{j} = 1*[0,0,0; 0,0,0; 1,1,1];
                    end

                    Q_ij{j} = sdpvar(3,3,'full');
                    Q_ji{j} = sdpvar(3,3,'full');

                    X_jj_12 = subsystems(jInd).dataToBeDistributed.X;
                    
                    Theta_ij = - X_ii_21*Q_ij{j} - Q_ji{j}'*X_jj_12;
                    
                    W_ij = [O_n, Q_ij{j}; Q_ji{j}', Theta_ij];
                    W_i = [W_i, W_ij];
                end

                con1 = p_i >= 0;
                con2 = [M_i, W_i';W_i, W_ii] >= 0;
                
                con3 = [];
                con4 = [];
                costFun = 0;
                for j = 1:1:length(previousSubsystems)
                    con3_ij = Q_ij{j}.*(null_ij{j}==1)==O_n;
                    con3_ji = Q_ji{j}.*(null_ji{j}==1)==O_n;
                    con3 = [con3, con3_ij, con3_ji];
                    
                    con4_ij = Q_ij{j}.*(adj_ij{j}==0)==O_n;
                    con4_ji = Q_ji{j}.*(adj_ji{j}==0)==O_n;
                    con4 = [con4, con4_ij, con4_ji]; 

                    costFun = costFun + norm(Q_ij{j}.*cost_ij{j},normType) + norm(Q_ji{j}.*cost_ji{j},normType);
                end

                con3_ii = Q_ii.*(null_ii==1)==O_n;
                con3 = [con3, con3_ii];
                
                costFun = costFun + norm(Q_ii.*cost_ii,normType);
                
                if isSoft
                    cons = [con1,con2,con3];
                else
                    cons = [con1,con2,con3,con4];
                end

                sol = optimize(cons,costFun,solverOptions);
                isStabilizable = sol.problem==0;

                p_iVal = value(p_i);
                Q_iiVal = value(Q_ii);
                W_iVal = value(W_i);
                W_iiVal = value(W_ii);

                K_ii = (p_iVal*X_i_11)\Q_iiVal;
                obj.controllerGainsCollection.decenStabCont1{iInd} = K_ii;
                for j = 1:1:length(previousSubsystems)
                    jInd = previousSubsystems(j);
                    
                    Q_ijVal = value(Q_ij{j});
                    Q_jiVal = value(Q_ji{j});
                    p_jVal = subsystems(jInd).dataToBeDistributed.P;
                    X_j_11 = -subsystems(jInd).nu*I_n;
                                        
                    K_ij = (p_iVal*X_i_11)\Q_ijVal;
                    K_ijVals{jInd} = K_ij;
                    obj.controllerGainsCollection.decenStabCont1{jInd} = K_ij;
                                        
                    K_ji = (p_jVal*X_j_11)\Q_jiVal
                    K_jiVals{jInd} = K_ji; % these values will be loaded outside the function
                end  
                
                % Need to compute \tilede{W}_i and \tilde{W}_{ii} for storage
                tildeW_i = W_iVal*M1_i;
                tildeW_ii = W_iiVal - tildeW_i*scriptD_i*tildeW_i'; % Note that here, \tilde{W}_ii, W_ii, \tilde{W}_i are different.

                tildeW_i = [tildeW_i, tildeW_ii];

                if abs(det(tildeW_ii))<0.000000001
                    disp("Error: det(tildeW_ii) low");
                    isStabilizable = 0;
                end

                obj.dataToBeDistributed.P = p_iVal;         % Storing
                obj.dataToBeDistributed.tildeW = tildeW_i; % Storing
                disp(['Data saved ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
                if ~isStabilizable
                    disp(['LMI is not feasible at: ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
                end
                
            end
        end


        %% Decentralized Robust Controller Synthesis (Error Dynamics II)
        function [isRobustStabilizable,K_ii,K_ijVals,K_jiVals,gammaSq_iVal,statusL,LVal] = robustControllerSynthesis2(obj, previousSubsystems, subsystems, pVal, displayMasseges, isSoft)

            % i = length(previousSubsystems)+1;
            iInd = obj.vehicleIndex-1;
            N = iInd;
            costCoefficient1 = 1;
            costCoefficient2 = 1; 
            costCoefficient3 = 1; 
     
            [statusL,PVal,KVal,LVal,nuVal,rhoVal,gammaSqLVal] = obj.synthesizeLocalControllersParameterized(2,pVal);
            if displayMasseges
                disp(['Robust Stabilizing at: ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
                if statusL == 1
                    disp(['Local Synthesis Success at: ',num2str(iInd),' with gammaSq=',num2str(value(gammaSqLVal)),'.'])
                else
                    disp(['Local Synthesis Failed at: ',num2str(iInd),'.'])
                end
            end
            
            if statusL == 0
                isRobustStabilizable = 0;
                return
            end 

            % Seting up the LMI problem
            solverOptions = sdpsettings('solver','mosek','verbose',0);
%             solverOptions = sdpsettings('solver','mosek','verbose',2,'warning',1);
            normType = 2; % Type of the norm to be used in the LMI

            I_n = eye(3);
            O_n = zeros(3);

            nu_i = obj.nu;
            rho_i = obj.rho;
            X_i_11 = -nu_i*I_n;
            X_i_12 = 0.5*I_n;
            X_i_21 = X_i_12';
            X_i_22 = -rho_i*I_n;
            X_ii_12 = X_i_11\X_i_12;
            X_ii_21 = X_ii_12';

            obj.dataToBeDistributed.X = X_ii_12;

            null_ii = [1,1,1; 1,1,1; 0,0,0];
            % adj_ii = [0,0,0; 0,0,0; 1,1,1];
            cost_ii = 0*[0,0,0; 0,0,0; 1,1,1];

            if isempty(previousSubsystems)
                
                % The subsystem only need to test W_ii > 0
                p_i = sdpvar(1,1);
                Q_ii = sdpvar(3,3,'full');
                gammaSq_i = sdpvar(1,1);
                
                costFun0 = sum(sum(Q_ii.*cost_ii));

                con0 = costFun0 >= 0.001;
                con1 = p_i >= 0;

%                 DMat = [X_p_11, O; O, I];
%                 MMat = [Q, X_p_11; I, O];
%                 ThetaMat = [-X_21*Q-Q'*X_12-X_p_22, -X_p_21; -X_p_12, gammaSq*I];
%                 con4 = [DMat, MMat; MMat', ThetaMat] >= 0; % The real one  
                
                DMat_ii = [p_i*X_i_11, O_n; O_n, I_n];
                MMat_ii = [Q_ii, p_i*X_i_11; I_n, O_n]; 
                TMat_ii = [-X_ii_21*Q_ii-Q_ii'*X_ii_12-p_i*X_i_22, -p_i*X_i_21; -p_i*X_i_12, gammaSq_i*I_n];
                
                W_ii = [DMat_ii, MMat_ii; MMat_ii', TMat_ii];

                con2 = W_ii >= 0;%10^(-6)*eye(size(W_ii));

                con3 = Q_ii.*(null_ii==1)==O_n;

                costFun =  costCoefficient1*costFun0 + costCoefficient2*gammaSq_i + costCoefficient3*(gammaSq_i-gammaSqLVal)^2;

                sol = optimize([con1,con2,con3],costFun,solverOptions);
                isRobustStabilizable = sol.problem==0;

                p_iVal = value(p_i);
                Q_iiVal = value(Q_ii);
                gammaSq_iVal = value(gammaSq_i);
                costFun0Val = value(costFun0);

                con2Val = value(W_ii);
                eigVals = eig(con2Val);
%                 minEigVal = min(eigVals);
% %                 con0Val = value(con0)
%                 con1Val = value(con1)
%                 con2Val = value(con2)
%                 con3Vals = [];
%                 for k = 1:1:length(con3)
%                     con3Vals = [con3Vals, value(con3(k))];
%                 end
%                 con3Val = any(con3Vals(:) == 0)
                
                W_iiVal = value(W_ii);
                tildeW_i = W_iiVal; % Note that, here, \tilde{W}_ii = W_ii = \tilde{W}_i. This also needs to be stored
                
                K_ii = (p_iVal*X_i_11)\Q_iiVal;
                obj.controllerGainsCollection.decenRobustCont1{iInd} = K_ii;

                K_jiVals = [];
                K_ijVals = [];
                
                obj.dataToBeDistributed.P = p_iVal;
                obj.dataToBeDistributed.gammaSq = gammaSq_iVal;
                obj.dataToBeDistributed.tildeW = tildeW_i;  % Storing
                
                if displayMasseges
                    disp(['Data saved at: ',num2str(iInd),'.']);
                    if ~isRobustStabilizable
                        disp(['LMI is not feasible at: ',num2str(iInd),', thus terminating here.']);
                    else
                        disp(['LMI is feasible at: ',num2str(iInd),', thus continued.'])
                    end
                end

                if ~isRobustStabilizable && (min(eigVals)<-0.000001 || p_iVal < 0)
                    disp(['Decentralized Synthesis Failed at: ',num2str(iInd),'.'])
                else
                    isRobustStabilizable = 1;
                    disp(['Decentralized Synthesis Success at: ',num2str(iInd),' with gammaSq=',num2str(value(gammaSq_iVal)),'.'])
                end

            else
                % This subsystem has to talk with all the previosSubsystems
                % tildeW_ii > 0 iff [M_i, W_i'; W_i, W_ii] > 0 is required where 
                % M_i =
                % inv((scriptD_i*scriptA_i^T)^{-1}*(scriptD_i)*(scriptD_i*scriptA_i^T)^{-1}') = scriptA_i*scriptD_i*scriptA_i'

                % M_i term
                blockSize = 12; 
                for j = 1:1:length(previousSubsystems)
                    jInd = previousSubsystems(j);

                    % Getting stored info from jInd to create \mathcal{A}_i and \mathcal{D}_i matrices (their j-th columns)
                    tildeW_j = subsystems(jInd+1).dataToBeDistributed.tildeW;

                    if j == 1
                        tildeW_jj = tildeW_j;
                        M_i = tildeW_jj;
                    else
                        tildeW_jj = tildeW_j(:,blockSize*(j-1)+1:blockSize*j);   % last blockSizeXblockSize block in the row block vector
                        tildeW_j  = tildeW_j(:,1:blockSize*(j-1));                % first (j-1) blocks in the row block vector
                        M_i = [M_i, tildeW_j'; tildeW_j, tildeW_jj];
                    end                    
                end
                
                % LMI variables
                p_i = sdpvar(1,1);
                Q_ii = sdpvar(3,3,'full');
                gammaSq_i = sdpvar(1,1);

                % W_ii and W_i terms
%                 DMat = [X_p_11, O; O, I];
%                 MMat = [Q, X_p_11; I, O];
%                 ThetaMat = [-X_21*Q-Q'*X_12-X_p_22, -X_p_21; -X_p_12, gammaSq*I];
%                 con4 = [DMat, MMat; MMat', ThetaMat] >= 0; % The real one  
                DMat_ii = [p_i*X_i_11, O_n; O_n, I_n];
                MMat_ii = [Q_ii, p_i*X_i_11; I_n, O_n]; 
                TMat_ii = [-X_ii_21*Q_ii-Q_ii'*X_ii_12-p_i*X_i_22, -p_i*X_i_21; -p_i*X_i_12, gammaSq_i*I_n];
                W_ii = [DMat_ii, MMat_ii; MMat_ii', TMat_ii];

                W_i = [];
                for j = 1:1:length(previousSubsystems)

                    jInd = previousSubsystems(j);

                    null_ij{j} = [1,1,1; 1,1,1; 0,0,0];
                    null_ji{j} = [1,1,1; 1,1,1; 0,0,0];

                    if any(obj.inNeighbors==jInd) % iInd <----- jInd
                        adj_ij{j} = [0,0,0; 0,0,0; 1,1,1]; 
                        cost_ij{j} = 1*[0,0,0; 0,0,0; 1,1,1];
                    else
                        adj_ij{j} = [0,0,0; 0,0,0; 0,0,0];
                        cost_ij{j} = (20/N)*abs(iInd-jInd)*[0,0,0; 0,0,0; 1,1,1];
                    end

                    if any(obj.outNeighbors==jInd) % iInd -----> jInd
                        adj_ji{j} = [0,0,0; 0,0,0; 1,1,1];
                        cost_ji{j} = 1*[0,0,0; 0,0,0; 1,1,1];
                    else
                        adj_ji{j} = [0,0,0; 0,0,0; 0,0,0];
                        cost_ji{j} = (20/N)*abs(iInd-jInd)*[0,0,0; 0,0,0; 1,1,1];
                    end

                    Q_ij{j} = sdpvar(3,3,'full');
                    Q_ji{j} = sdpvar(3,3,'full');

                    X_jj_12 = subsystems(jInd+1).dataToBeDistributed.X;
                    
%                 DMat = [X_p_11, O; O, I];
%                 MMat = [Q, X_p_11; I, O];
%                 ThetaMat = [-X_21*Q-Q'*X_12-X_p_22, -X_p_21; -X_p_12, gammaSq*I];
%                 con4 = [DMat, MMat; MMat', ThetaMat] >= 0; % The real one  
                    DMat_ij = [O_n, O_n; O_n, O_n];
                    MMat_ij = [Q_ij{j}, O_n; O_n, O_n];
                    MMat_ji = [Q_ji{j}, O_n; O_n, O_n];
                    
                    TMat_ij = [-X_ii_21*Q_ij{j}-Q_ji{j}'*X_jj_12, O_n; O_n, O_n];
                    W_ij = [DMat_ij, MMat_ij; MMat_ji', TMat_ij];
                    W_i = [W_i, W_ij];

                end

                con1 = p_i >= 0;

                % con2Mat = [M_i, W_i';W_i, W_ii];
                con2 = [M_i, W_i';W_i, W_ii] >= 0; %10^(-6)*eye(size(M_i)+size(W_ii));
                
                con3 = []; % Soft graph constraints
                con4 = []; % Hard graph constraints
                costFun0 = 0;
                for j = 1:1:length(previousSubsystems)
                    con3_ij = Q_ij{j}.*(null_ij{j}==1)==O_n;
                    con3_ji = Q_ji{j}.*(null_ji{j}==1)==O_n;
                    con3 = [con3, con3_ij, con3_ji];
                    
                    con4_ij = Q_ij{j}.*(adj_ij{j}==0)==O_n;
                    con4_ji = Q_ji{j}.*(adj_ji{j}==0)==O_n;
                    con4 = [con4, con4_ij, con4_ji]; 

                    costFun0 = costFun0 + sum(sum(Q_ij{j}.*cost_ij{j})) + sum(sum(Q_ji{j}.*cost_ji{j}));
                end

                con3_ii = Q_ii.*(null_ii==1)==O_n;
                con3 = [con3, con3_ii];
                
                costFun0 = costFun0 + sum(sum(Q_ii.*cost_ii));
                con0 = costFun0 >= 0.001;
                
                if isSoft
                    cons = [con0,con1,con2,con3];
                else
                    cons = [con0,con1,con2,con3,con4];
                end
                
                costFun =  costCoefficient1*costFun0 + costCoefficient2*gammaSq_i + costCoefficient3*(gammaSq_i-gammaSqLVal)^2;

                sol = optimize(cons,costFun,solverOptions);
                isRobustStabilizable = sol.problem==0;

                p_iVal = value(p_i);
                Q_iiVal = value(Q_ii);
                gammaSq_iVal = value(gammaSq_i);
                costFun0Val = value(costFun0);

                con2Val = value([M_i, W_i';W_i, W_ii]);
                eigVals = eig(con2Val);   
%                 minEigVal = min(eigVals);
%                 con0Val = value(con0)
%                 con1Val = value(con1)
%                 con2Val = value(con2)
%                 con3Vals = [];
%                 for k = 1:1:length(con3)
%                     con3Vals = [con3Vals, value(con3(k))];
%                 end
%                 con3Val = any(con3Vals(:) == 0)

                W_iVal = value(W_i);
                W_iiVal = value(W_ii);

                K_ii = (p_iVal*X_i_11)\Q_iiVal;
                obj.controllerGainsCollection.decenStabCont1{iInd} = K_ii;
                for j = 1:1:length(previousSubsystems)
                    jInd = previousSubsystems(j);
                    
                    Q_ijVal = value(Q_ij{j});
                    Q_jiVal = value(Q_ji{j});
                    p_jVal = subsystems(jInd+1).dataToBeDistributed.P;
                    X_j_11 = -subsystems(jInd+1).nu*I_n;
                          
                    K_ij = (p_iVal*X_i_11)\Q_ijVal;
                    K_ijVals{jInd} = K_ij;
                    obj.controllerGainsCollection.decenStabCont1{jInd} = K_ij;
                                        
                    K_ji = (p_jVal*X_j_11)\Q_jiVal;
                    K_jiVals{jInd} = K_ji; % these values will be loaded to subsystem j outside of this function
                end
                
                % Need to compute \tilede{W}_i and \tilde{W}_{ii} for storage
                tildeW_i = W_iVal;
                tildeW_ii = W_iiVal;

                tildeW_i = [tildeW_i, tildeW_ii];

                % Storing
                obj.dataToBeDistributed.P = p_iVal;             
                obj.dataToBeDistributed.gammaSq = gammaSq_iVal;
                obj.dataToBeDistributed.tildeW = tildeW_i;      

                if displayMasseges
                    disp(['Data saved at: ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
                    if ~isRobustStabilizable
                        disp(['LMI is not feasible at: ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
                    else
                        disp(['LMI is feasible at: ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
                    end
                end

                if ~isRobustStabilizable && (min(eigVals)<-0.000001 || p_iVal < 0)
                    disp(['Decentralized Synthesis Failed at: ',num2str(iInd),'.'])
                else
                    isRobustStabilizable = 1;
                    disp(['Decentralized Synthesis Success at: ',num2str(iInd),' with gammaSq=',num2str(value(gammaSq_iVal)),'.'])
                end 

            end
        end


        %% Decentralized Robust Controller Synthesis With DSS Constraints (Error Dynamics II)
        function [isRobustStabilizable,K_ii,K_ijVals,K_jiVals,gammaSq_iVal,statusL,LVal] = robustControllerSynthesisDSS2(obj, previousSubsystems, subsystems, pVal, displayMasseges, isSoft)

            % i = length(previousSubsystems)+1;
            iInd = obj.vehicleIndex-1;
            N = iInd;
            costCoefficient1 = 1;
            costCoefficient2 = 1; 
            costCoefficient3 = 1; 
            
            % Add PVal and KVal in the original local controller synthesis function here to be used later
            [statusL,PVal,KVal,LVal,nuVal,rhoVal,gammaSqLVal] = obj.synthesizeLocalControllersParameterized(2,pVal);
            if displayMasseges
                disp(['Robust Stabilizing at: ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
                if statusL == 1
                    disp(['Local Synthesis Success at: ',num2str(iInd),' with gammaSq=',num2str(value(gammaSqLVal)),'.'])
                else
                    disp(['Local Synthesis Failed at: ',num2str(iInd),'.'])
                end
            end

            if statusL == 0
                isRobustStabilizable = 0;
                return
            end 

            % Setting up the LMI problem
            solverOptions = sdpsettings('solver','mosek','verbose',0);
            normType = 2; % Type of the norm to be used in the LMI

            I_n = eye(3);
            O_n = zeros(3);

            nu_i = obj.nu;
            rho_i = obj.rho;
            X_i_11 = -nu_i*I_n;
            X_i_12 = 0.5*I_n;
            X_i_21 = X_i_12';
            X_i_22 = -rho_i*I_n;
            X_ii_12 = X_i_11\X_i_12;
            X_ii_21 = X_ii_12';
            
            % Set a fixed value for epsilon_i and make epsilon_i+rho_i>1 satisfied
            epsilon_i = 1.01-rho_i;  

            % min and max eigenvalues of R_i
            R_i = inv(PVal)
            obj.R_i = R_i;
            MaxEigR_i = max(eig(R_i));
            MinEigR_i = min(eig(R_i));

            obj.dataToBeDistributed.X = X_ii_12;    

            null_ii = [1,1,1; 1,1,1; 0,0,0];
            % adj_ii = [0,0,0; 0,0,0; 1,1,1];
            cost_ii = 0*[0,0,0; 0,0,0; 1,1,1];

            if isempty(previousSubsystems)

                % The subsystem only need to test W_ii > 0 (since in this case, there are no subsystems before this ith subsystem)
                p_i = sdpvar(1,1);
                Q_ii = sdpvar(3,3,'full');
                gammaSq_i = sdpvar(1,1);

                costFun0 = sum(sum(Q_ii.*cost_ii)); 

                con0 = costFun0 >= 0.0001;
                con1 = p_i >= 0;

                DMat_ii = [p_i*X_i_11, O_n; O_n, I_n];
                MMat_ii = [Q_ii, p_i*X_i_11; I_n, O_n]; 
                TMat_ii = [-X_ii_21*Q_ii-Q_ii'*X_ii_12-p_i*X_i_22, -p_i*X_i_21; -p_i*X_i_12, gammaSq_i*I_n];

                W_ii = [DMat_ii, MMat_ii; MMat_ii', TMat_ii];     

                con2 = W_ii >= 0; %10^(-6)*eye(size(W_ii));

                con3 = Q_ii.*(null_ii==1) == O_n;     
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % Additional DSS constraint 1 (on K_ii) is added here (for the first follower, there are no neighboring vehicles)

                con4 = R_i*Q_ii+Q_ii'*R_i-p_i*nu_i*epsilon_i*I_n <= 0;
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                costFun =  costCoefficient1*costFun0 + costCoefficient2*gammaSq_i + costCoefficient3*(gammaSq_i-gammaSqLVal)^2;

                % Our modified optimization problem with the DSS constraint (con4)
                sol = optimize([con1,con2,con3,con4],costFun,solverOptions);
                isRobustStabilizable = sol.problem == 0;

                p_iVal = value(p_i);
                Q_iiVal = value(Q_ii);
                gammaSq_iVal = value(gammaSq_i);
                costFun0Val = value(costFun0);

                con2Val = value(W_ii);
                eigVals = eig(con2Val);

                W_iiVal = value(W_ii);
                tildeW_i = W_iiVal; % Note that, here, \tilde{W}_ii = W_ii = \tilde{W}_i. This also needs to be stored

                K_ii = (p_iVal*X_i_11)\Q_iiVal;
                obj.controllerGainsCollection.decenRobustCont1{iInd} = K_ii;

                K_jiVals = [];
                K_ijVals = [];

                obj.dataToBeDistributed.P = p_iVal;
                obj.dataToBeDistributed.gammaSq = gammaSq_iVal;
                obj.dataToBeDistributed.tildeW = tildeW_i;  % Storing

                if displayMasseges                                            
                    disp(['Data saved at: ',num2str(iInd),'.']);
                    if ~isRobustStabilizable
                        disp(['LMI is not feasible at: ',num2str(iInd),', thus terminating here.']);
                    else
                        disp(['LMI is feasible at: ',num2str(iInd),', thus continued.'])
                    end
                end

                if ~isRobustStabilizable && (min(eigVals)<-0.000001 || p_iVal < 0)
                    disp(['Decentralized Synthesis Failed at: ',num2str(iInd),'.'])
                else
                    isRobustStabilizable = 1;
                    disp(['Decentralized Synthesis Success at: ',num2str(iInd),' with gammaSq=',num2str(value(gammaSq_iVal)),'.'])
                end

            else
                % This subsystem has to talk with all the previousSubsystems
                % tildeW_ii > 0 iff [M_i, W_i'; W_i, W_ii] > 0 is required where 
                % M_i = inv((scriptD_i*scriptA_i^T)^{-1}*(scriptD_i)*(scriptD_i*scriptA_i^T)^{-1}') = scriptA_i*scriptD_i*scriptA_i'

                % M_i term
                blockSize = 12; 
                for j = 1:1:length(previousSubsystems)
                    jInd = previousSubsystems(j);

                    % Getting stored info from jInd to create \mathcal{A}_i and \mathcal{D}_i matrices (their j-th columns)
                    tildeW_j = subsystems(jInd+1).dataToBeDistributed.tildeW;

                    if j == 1
                        tildeW_jj = tildeW_j;
                        M_i = tildeW_jj;
                    else
                        tildeW_jj = tildeW_j(:,blockSize*(j-1)+1:blockSize*j);   % last blockSizeXblockSize block in the row block vector
                        tildeW_j  = tildeW_j(:,1:blockSize*(j-1));                % first (j-1) blocks in the row block vector
                        M_i = [M_i, tildeW_j'; tildeW_j, tildeW_jj];        % This tildeW_jj is iteratively updated based on the previous stored tildeW_ii
                    end                    
                end

                % LMI variables
                p_i = sdpvar(1,1);
                Q_ii = sdpvar(3,3,'full');
                gammaSq_i = sdpvar(1,1);

                % W_ii and W_i terms
%                 DMat = [X_p_11, O; O, I];
%                 MMat = [Q, X_p_11; I, O];
%                 ThetaMat = [-X_21*Q-Q'*X_12-X_p_22, -X_p_21; -X_p_12, gammaSq*I];
%                 con4 = [DMat, MMat; MMat', ThetaMat] >= 0; % The real one  
 
                % When i = j  
                DMat_ii = [p_i*X_i_11, O_n; O_n, I_n];
                MMat_ii = [Q_ii, p_i*X_i_11; I_n, O_n]; 
                TMat_ii = [-X_ii_21*Q_ii-Q_ii'*X_ii_12-p_i*X_i_22, -p_i*X_i_21; -p_i*X_i_12, gammaSq_i*I_n];
                W_ii = [DMat_ii, MMat_ii; MMat_ii', TMat_ii];

                W_i = [];                            
                for j = 1:1:length(previousSubsystems)

                    jInd = previousSubsystems(j);

                    null_ij{j} = [1,1,1; 1,1,1; 0,0,0];
                    null_ji{j} = [1,1,1; 1,1,1; 0,0,0];
                    
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    % In order to solve the optimization with DSS constraint, 
                    % we confine the structure of Q_ij and Q_ji using these matrices, where only the last element has value

                    null_ij_DSS{j} = [1 1 1;1 1 1;1 1 0]; 
                    null_ji_DSS{j} = [1 1 1;1 1 1;1 1 0];
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                    if any(obj.inNeighbors==jInd) % iInd <----- jInd
                        adj_ij{j} = [0,0,0; 0,0,0; 1,1,1]; % is in-neighbors
                        cost_ij{j} = 1*[0,0,0; 0,0,0; 1,1,1];
                    else
                        adj_ij{j} = [0,0,0; 0,0,0; 0,0,0]; % not in-neighbors
                        cost_ij{j} = (20/N)*abs(iInd-jInd)*[0,0,0; 0,0,0; 1,1,1];
                    end

                    if any(obj.outNeighbors==jInd) % iInd -----> jInd
                        adj_ji{j} = [0,0,0; 0,0,0; 1,1,1]; % is out-neighbors
                        cost_ji{j} = 1*[0,0,0; 0,0,0; 1,1,1];
                    else
                        adj_ji{j} = [0,0,0; 0,0,0; 0,0,0]; % not out-neighbors
                        cost_ji{j} = (20/N)*abs(iInd-jInd)*[0,0,0; 0,0,0; 1,1,1];
                    end

                    Q_ij{j} = sdpvar(3,3,'full');
                    Q_ji{j} = sdpvar(3,3,'full');

                    X_jj_12 = subsystems(jInd+1).dataToBeDistributed.X;

%                 DMat = [X_p_11, O; O, I];
%                 MMat = [Q, X_p_11; I, O];
%                 ThetaMat = [-X_21*Q-Q'*X_12-X_p_22, -X_p_21; -X_p_12, gammaSq*I];
%                 con4 = [DMat, MMat; MMat', ThetaMat] >= 0; % The real one  
                    DMat_ij = [O_n, O_n; O_n, O_n];
                    MMat_ij = [Q_ij{j}, O_n; O_n, O_n];
                    MMat_ji = [Q_ji{j}, O_n; O_n, O_n];
                    TMat_ij = [-X_ii_21*Q_ij{j}-Q_ji{j}'*X_jj_12, O_n; O_n, O_n];

                    % When i \neq j
                    W_ij = [DMat_ij, MMat_ij; MMat_ji', TMat_ij];
                    W_i = [W_i, W_ij];

                end

                con1 = p_i >= 0;

                % con2Mat = [M_i, W_i';W_i, W_ii];
                con2 = [M_i, W_i';W_i, W_ii] >= 0; %10^(-6)*eye(size(M_i)+size(W_ii));

                con3 = []; % Soft graph constraints
                con3_hard = []; % Hard graph constraints
                costFun0 = 0;
                con5_value = 0;

                % To Guarantee DSS, we confine the K_ij (Q_ij) values with our proposed alternative DSS condition (DSS constraint 2 option 1)
                % (for the second and other followers, the information from neighboring vehicles are considered, i.e., K_ij)
                for j = 1:1:length(previousSubsystems)

                    % jInd = previousSubsystems(j);
                    % Confine the Q_{ij} and Q_{ji} values with the unique structure
                    con3_ij = Q_ij{j}.*(null_ij{j}==1)==O_n;
                    con3_ji = Q_ji{j}.*(null_ji{j}==1)==O_n;
                    con3 = [con3, con3_ij, con3_ji];

                    con3_ij_hard = Q_ij{j}.*(adj_ij{j}==0)==O_n;
                    con3_ji_hard = Q_ji{j}.*(adj_ji{j}==0)==O_n;
                    con3_hard = [con3_hard, con3_ij_hard, con3_ji_hard]; 

                    costFun0 = costFun0 + sum(sum(Q_ij{j}.*cost_ij{j})) + sum(sum(Q_ji{j}.*cost_ji{j}));
                    
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    % Iteratively add the K_ij values to the LHS of the DSS constraint (DSS constraint 2 option 1)

                    mu_i = (rho_i+epsilon_i-1)/MaxEigR_i;
                    con5_value = con5_value + sqrt(1/(mu_i*MinEigR_i))*norm(R_i*Q_ij{j})+1/(2^j)*(p_i*nu_i);  
                    % The 2-norm is (k31^2 + k32^2 + k33^2)^(1/2)*(p13^2 +p23^2 + p33^2)^(1/2), 
                    % and thus, it can be somehow simplified
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                end
                
                % Confine the Q_{ii} value with the unique structure
                con3_ii = Q_ii.*(null_ii==1)==O_n;

                % Collect all the constraints on Q_{ij}'s value
                con3 = [con3, con3_ii];
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % The additional DSS constraint 1 (on Q_{ii} or K_{ii}) is added here 

                con4 = R_i*Q_ii+Q_ii'*R_i-p_i*nu_i*epsilon_i*I_n <= 0;
                
                % To Guarantee DSS, we confine the K_ij (Q_ij) values with our proposed alternative DSS condition (DSS constraint 2 option 2)
                % (for the second and other followers, the information from neighboring vehicles are considered, i.e., K_ij)
                 
                % for j = 1:1:length(previousSubsystems)
                %     con3_ij = Q_ij{j}.*(null_ij_DSS{j}==1) == O_n;
                %     con3_ji = Q_ji{j}.*(null_ji_DSS{j}==1) == O_n;
                %     con3 = [con3, con3_ij, con3_ji];
                % 
                %     con3_ij_hard = Q_ij{j}.*(adj_ij{j}==0) == O_n;
                %     con3_ji_hard = Q_ji{j}.*(adj_ji{j}==0) == O_n;
                %     con3_hard = [con3_hard, con3_ij_hard, con3_ji_hard]; 
                % 
                %     costFun0 = costFun0 + sum(sum(Q_ij{j}.*cost_ij{j})) + sum(sum(Q_ji{j}.*cost_ji{j}));
                % 
                %     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %     % Iteratively add the K_ij values to the LHS of the DSS constraint
                % 
                %     mu_i = (rho_i+epsilon_i-1)/MaxEigR_i;
                %     con5_value = con5_value + sqrt(1/(mu_i*MinEigR_i))*norm(R_i*Q_ij{j})+1/(2^j)*(p_i*nu_i);  
                %     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % end
                % K_ij = k_ij*[0 0 0; 0 0 0; 0 0 1];

                % Remaining DSS constraints
                % First attempt: follow the expression in the paper
                % Second attempt: use a different norm, i.e., 1-norm
                % Third attempt: confine ourselves to a specific type of K_ij value, say a scalar k_ij
                con5 = con5_value <= 0;

                % These two conditions are not necessary if epsilon_i is carefully selected 
                % con6 = rho_i+epsilon_i-1 > 0;
                % con7 = epsilon_i > 0;
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                costFun0 = costFun0 + sum(sum(Q_ii.*cost_ii));
                con0 = costFun0 >= 0.0001;
                
                if isSoft               
                    cons = [con0,con1,con2,con3,con4,con5];
                else
                    cons = [con0,con1,con2,con3,con3_hard,con4,con5];
                end

                costFun =  costCoefficient1*costFun0 + costCoefficient2*gammaSq_i + costCoefficient3*(gammaSq_i-gammaSqLVal)^2;
                
                % Here, we solve the co-design optimization problem with DSS constraints
                sol = optimize(cons,costFun,solverOptions);
                isRobustStabilizable = sol.problem==0;

                p_iVal = value(p_i);
                Q_iiVal = value(Q_ii);
                gammaSq_iVal = value(gammaSq_i);
                costFun0Val = value(costFun0);

                con2Val = value([M_i, W_i';W_i, W_ii]);
                eigVals = eig(con2Val);   

                W_iVal = value(W_i);
                W_iiVal = value(W_ii);
                
                % Obtain the solved K_{ii} values
                K_ii = (p_iVal*X_i_11)\Q_iiVal;
                obj.controllerGainsCollection.decenStabCont1{iInd} = K_ii;

                for j = 1:1:length(previousSubsystems)
                    jInd = previousSubsystems(j);

                    Q_ijVal = value(Q_ij{j});
                    Q_jiVal = value(Q_ji{j});
                    p_jVal = subsystems(jInd+1).dataToBeDistributed.P;
                    X_j_11 = -subsystems(jInd+1).nu*I_n;
                    
                    % Obtain the solved K_{ij} values
                    K_ij = (p_iVal*X_i_11)\Q_ijVal;
                    K_ijVals{jInd} = K_ij;
                    obj.controllerGainsCollection.decenStabCont1{jInd} = K_ij;
                    
                    % Obtain the solved K_{ji} values
                    K_ji = (p_jVal*X_j_11)\Q_jiVal;
                    K_jiVals{jInd} = K_ji; % these values will be loaded to subsystem j outside of this function
                end

                % Need to compute \tilede{W}_i and \tilde{W}_{ii} for storage
                tildeW_i = W_iVal;
                tildeW_ii = W_iiVal;

                tildeW_i = [tildeW_i, tildeW_ii];

                % Storing
                obj.dataToBeDistributed.P = p_iVal;             
                obj.dataToBeDistributed.gammaSq = gammaSq_iVal;
                obj.dataToBeDistributed.tildeW = tildeW_i;      

                if displayMasseges
                    disp(['Data saved at: ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
                    if ~isRobustStabilizable
                        disp(['LMI is not feasible at: ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
                    else
                        disp(['LMI is feasible at: ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
                    end
                end

                if ~isRobustStabilizable && (min(eigVals)<-0.000001 || p_iVal < 0)
                    disp(['Decentralized Synthesis Failed at: ',num2str(iInd),'.'])
                else
                    isRobustStabilizable = 1;
                    disp(['Decentralized Synthesis Success at: ',num2str(iInd),' with gammaSq=',num2str(value(gammaSq_iVal)),'.'])
                end 

            end
        end

    end
end