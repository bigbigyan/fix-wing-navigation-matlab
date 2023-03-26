% calculate the heuristic cost from current node to goal node by 3D bubins
% curve
function h_cost = HeuristicCost(current_node,target_node,uav_property)

h_cost = abs(target_node(1) - current_node(1)) + abs(target_node(2) - current_node(2)) + abs(target_node(3) - current_node(3));

% connectionObj = uavDubinsConnection('DisabledPathTypes',{'RLRN'});
% connectionObj = uavDubinsConnection;
% connectionObj.AirSpeed = uav_property(1)/10;
% connectionObj.MaxRollAngle = uav_property(2);
% connectionObj.FlightPathAngleLimit = [-uav_property(3),uav_property(3)];
% 
% [pathSegObj,h_cost] = connect(connectionObj,current_node,target_node);


end
