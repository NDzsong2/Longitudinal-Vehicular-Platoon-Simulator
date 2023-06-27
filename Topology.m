classdef Topology < handle
    %UNTITLED5 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        startNodes
        endNodes
        
        graph  % store the topology as a matlab digraph variable

        graphics
    end

    methods

        function obj = Topology(n_k)

            startNodes = [];
            endNodes = [];
            for i = 1:1:n_k
                nodeNames{i} = num2str(i-1);
                if i <= 2
                    startNodes = [startNodes, i];
                    endNodes = [endNodes, i+1];
                elseif i > 2 && i < n_k
                    startNodes = [startNodes, i,i];
                    endNodes = [endNodes, i+1,i-1];
                elseif i == n_k
                    startNodes = [startNodes, i];
                    endNodes = [endNodes, i-1];
                end
            end
            obj.startNodes = startNodes;
            obj.endNodes = endNodes; 
            weights = ones(length(startNodes),1);
            
            % Here, we need the digraph for further compute the graph matrices
            obj.graph = digraph(startNodes,endNodes,weights,nodeNames);
                        
        end
        
%         function outputArg = drawTopology(obj,figNum,platoonSample)
%             adjHeight_coeff = 10;
% 
%             figure(figNum); hold on;
%             if ~isempty(obj.graphics)
%                 delete(obj.graphics);
%             end
% 
%             % Here, we need the digraph (augmented but with no leader) with curved communication links
%             for l = 1:1:length(obj.startNodes)
%                 startNode = obj.startNodes(l);
%                 endNode = obj.endNodes(l);
%                 p1 = platoonSample.vehicles(startNode).position;
%                 p2 = platoonSample.vehicles(endNode).position;
%                 pm = (p1+p2)/2
%                 pm(2) = pm(2) + adjHeight_coeff * (abs(startNode-endNode))
% 
%                 % plot line from p1--->pm
%                 plot([p1(1),pm(1)],[p1(2),pm(2)])
% 
%                 % plot line from pm---->p2
%                 plot([pm(1),p2(1)],[p2(2),pm(2)])
% 
%                 % plot an arrow head
% %                 Coordinates = [-1,-1;1,0;-1,1] % cordinates of a triangle centerd at (0,0)
% %                 Rotation = ...% will depend on pm and p2 (use atan2(pm-p2))
% %                 Translation = % depend on p2 +- epsilon
% %                 transformedCordinates = Rotation*Coordinates + Translation  
% %                 pgon = polyshape(transformedCoordinates(:,1),transformedCoordinates(:,2))
% %                 plot(pgon)
%                 
% 
% %                 p1 = obj.startPositions(:,l)';
% %                 p2 = obj.endPositions(:,l)';
% %                 linkLength_x = abs(p2(1)-p1(1));
% %                 link_AvePoint = [0.5 * linkLength_x; adjHeight_coeff * linkLength_x];   % We need to change the second item into vehicle numbers
% %                 if p1(1) < p2(1)
% %                     link_xPoints = [p1(1) link_AvePoint(1) p2(1)];
% %                     link_yPoints = [p1(2) link_AvePoint(2) p2(2)];
% %                     linkLength_xx = p1(1):0.01:linkLength_x;     
% %                     linkLength_yy = spline(link_xPoints,link_yPoints,linkLength_xx);
% %                     obj.graphics(l) = plot(linkLength_xx,linkLength_yy);
% % %                 obj.graphics(l) = quiver(p1(1),p1(2),dp(1),dp(2),0,'filled','Color','m','LineWidth',1,...
% % %                         'ShowArrowHead','on','MaxHeadSize',0.5,'AutoScale','off');
% %                 end
%             end
%         end
    end
end






