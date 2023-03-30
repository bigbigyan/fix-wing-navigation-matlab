% calculate the heuristic cost from current node to goal node by 3D bubins
% curve
function h_cost = HeuristicCost(current_node,target_node,uav_property)

g = 9.98;  % gravity accelerate
velicoty = uav_property(1)/10;
roll = uav_property(2);
pitch = uav_property(3);

r_min = velicoty*velicoty*tan(roll)/g;  %minimise turn radiu
absolute_value = sqrt((target_node(1) - current_node(1))^2 + (target_node(2) - current_node(2))^2 + (target_node(3) - current_node(3))^2) * 1.7;

p1 = [current_node(1),current_node(2),current_node(4)]; % 2D current node
p2 = [target_node(1),target_node(2),target_node(4)]; % 2D target node

length_2D = dubins_core(p1, p2, r_min);

if (absolute_value < length_2D*tan(pitch) || absolute_value == length_2D*tan(pitch))
    % low altitute
    r1 = atan(absolute_value/length_2D);
    length_3D = length_2D/cos(r1);
elseif absolute_value > (length_2D + 2*pi*r_min)*tan(pitch)
    % high altitude
    k = floor(1 / (2*pi*r_min) * (absolute_value/tan(pitch) - length_2D));
    proportional_r2 = (absolute_value/tan(pitch)) / (length_2D + 2*pi*k*r_min);
    r2 = proportional_r2*r_min;
    length_3D = dubins_core(p1,p2,r2) / cos(pitch);
else
    % medium altitude
    l = absolute_value/tan(pitch);
    length_3D = l / cos(pitch);
end

h_cost = length_3D;

% h_cost = abs(target_node(1) - current_node(1)) + abs(target_node(2) - current_node(2)) + abs(target_node(3) - current_node(3));

% h_cost = sqrt((target_node(1) - current_node(1))^2 + (target_node(2) - current_node(2))^2 + (target_node(3) - current_node(3))^2) * 1.7;

end
