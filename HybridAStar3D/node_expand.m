% function to return an expanded node array
function expand_node = node_expand(current_state,target_state,max_x,max_y,max_z,g_cost,Close,map,display_data,uav_property)
    
    expand_node = [];
    expand_count = 1;

    x_current = current_state(1);
    y_current = current_state(2);
    z_current = current_state(3);
    % yaw_current = current_state(4);

    motion_primitives = generate_primitives(current_state,uav_property);

    m2 = size(motion_primitives,1);
    c2 = size(Close,1);
    
    for m1 = 1:m2
        x_next = motion_primitives(m1,1);
        y_next = motion_primitives(m1,2);
        z_next = motion_primitives(m1,3);
        yaw_next = motion_primitives(m1,4);
        next_state = [x_next,y_next,z_next,yaw_next];
        % which cell in map the next state fall in
        x_node = floor(x_next);
        y_node = floor(y_next);
        z_node = floor(z_next);

        % if expend node with in map
        if((x_next > 0 && x_next < max_x) && (y_next > 0 && y_next < max_y) && (z_next > 0 && z_next < max_z))
            flag = 1;
            % if node within close list
            for c1 = 1:c2
                if(x_node == Close(c1,1) && y_node == Close(c1,2) && z_next == Close(c1,3))
                    flag = 0;
                end 
            end
            % if node in obstacle
            if map(x_node,y_node,z_node) == 1
                flag = 0;
            end
            % height limited
            height_limited = z_next - display_data(x_node,y_node);
            if height_limited > 10
                flag = 0;
            end

            % populate the data of expand_node
            if(flag == 1)
                expand_node(expand_count,1) = x_next;
                expand_node(expand_count,2) = y_next;
                expand_node(expand_count,3) = z_next;
                expand_node(expand_count,4) = yaw_next;
                expand_node(expand_count,5) = g_cost + 8 * (1/z_next) + ...
                    sqrt((x_next - x_current)^2 + (y_next - y_current)^2 + (z_next - z_current)^2);
                expand_node(expand_count,6) = HeuristicCost(next_state,target_state,uav_property);
                expand_node(expand_count,7) = expand_node(expand_count,5) + expand_node(expand_count,6);
                expand_node(expand_count,8) = x_node;
                expand_node(expand_count,9) = y_node;
                expand_node(expand_count,10) = z_node;
                expand_count = expand_count + 1;
            end % end of populating the data of expand_node
        end % end of node in the map
    end % end for each expand state

end % function end