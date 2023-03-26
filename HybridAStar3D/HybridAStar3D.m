function [waypoints,open_count] = HybridAStar3D(max_x,max_y,max_z,startpoint,goalpoint,map,display_data,uav_property)

% set optimal path node
waypoints = [];

% set start node
x_start = startpoint(1);
y_start = startpoint(2);
z_start = startpoint(3);
yaw_start = startpoint(4);

% set goal node
x_target = goalpoint(1);
y_target = goalpoint(2);
z_target = goalpoint(3);
yaw_target = goalpoint(4);
target_state = [x_target,y_target,z_target,yaw_target];

% open list  | x | y | z | yaw | parent x | parent y | parent z | 
%            parent yaw | g(n) | h(n) | f(n) | x_node | y_node | z_node |
% close list | x | y | z | yaw | parent x | parent y | parent z | parent yaw |x_node | y_node | z_node |
Open = [];
Close = [];

% set the starting node as first node
x_state = x_start;
y_state = y_start;
z_state = z_start;
yaw_state = yaw_start;
current_state = [x_state,y_state,z_state,yaw_state];

x_node = floor(x_state);
y_node = floor(y_state);
z_node = floor(z_state);
current_node = [x_node,y_node,z_node];

x_parent_state = x_start;
y_parent_state = y_start;
z_parent_state = z_start;
yaw_parent_state = yaw_start;
parent_state = [x_parent_state,y_parent_state,z_parent_state,yaw_parent_state];

g_cost = 0;
h_cost = HeuristicCost(current_state,target_state,uav_property);
f_cost = g_cost + h_cost;

open_count = 1;
close_count = 1;
% insert the open list and closed list
Open(open_count,:) = Insert_open(current_state,parent_state,g_cost,h_cost,f_cost,current_node);
Close(close_count,:) = Insert_close(current_state,parent_state,current_node);
no_path = 1; % if exist the path? 1 is, 0 no

% start algorithm
while(~isempty(Open))
    % node expand 
    % value of return list is : x y z yaw g h f x_node y_node z_node 
    expand_node = node_expand(current_state,target_state,max_x,max_y,max_z,g_cost,Close,map,display_data,uav_property);
    expand_count = size(expand_node,1);

    for i = 1:expand_count
        flag = 0;   % if expand node in open list? is 1; no 0
        % check if expand node is in open list, if in change the cost
        for j = 1 : open_count
            if(expand_node(i,8) == Open(j,12) && expand_node(i,9) == Open(j,13) && expand_node(i,10) == Open(j,14))
                Open(j,11) = min(Open(j,11),expand_node(i,7));
                if Open(j,11) == expand_node(i,7)
                    % update the parent node and cost
                    Open(j,5) = x_state;
                    Open(j,6) = y_state;
                    Open(j,7) = z_state;
                    Open(j,8) = yaw_state;
                    Open(j,9) = expand_node(i,5);
                    Open(j,10) = expand_node(i,6);
                    Open(j,12) = expand_node(i,8);
                    Open(j,13) = expand_node(i,9);
                    Open(j,14) = expand_node(i,10);
                end    % end of update
                flag = 1;
            end    % end of one node check
        end    % end of check all the node in open list
        % add expand node to open list
        if flag == 0
            current_state = [expand_node(i,1),expand_node(i,2),expand_node(i,3),expand_node(i,4)];
            parent_state = [x_state,y_state,z_state,yaw_state];
            current_node = [expand_node(i,8),expand_node(i,9),expand_node(i,10)];
            open_count = open_count + 1;
            Open(open_count,:) = Insert_open(current_state,parent_state,expand_node(i,5),expand_node(i,6),expand_node(i,7),current_node);
        end    % end of insert new elements into the open list
    end    % end of adding node into open list for all expand node

    % find the node with smallest f cost
    index_min_node = min_fn(Open);
    if(index_min_node ~= -1)
        x_state = Open(index_min_node,1);
        y_state = Open(index_min_node,2);
        z_state = Open(index_min_node,3);
        yaw_state = Open(index_min_node,4);
        x_parent_state = Open(index_min_node,5);
        y_parent_state = Open(index_min_node,6);
        z_parent_state = Open(index_min_node,7);
        yaw_parent_state = Open(index_min_node,8);
        g_cost = Open(index_min_node,9);
        x_node = Open(index_min_node,12);
        y_node = Open(index_min_node,13);
        z_node = Open(index_min_node,14);
        % move the node to close list
        close_count = close_count + 1;
        current_state = [x_state,y_state,z_state,yaw_state];
        parent_state = [x_parent_state,y_parent_state,z_parent_state,yaw_parent_state];
        current_node = [x_node,y_node,z_node];
        Close(close_count,:) = Insert_close(current_state,parent_state,current_node);
        % move out the node to open list
        Open(index_min_node,:) = [];
        open_count = open_count - 1;
    else
        % no path exist to the target
        no_path = 0;
    end
    % arrive the goal node
    if(x_node == x_target && y_node == y_target && z_node == z_target)
        break;
    end

end    % end of the while loop

i=size(Close,1);
Optimal_path=[];
xval=Close(i,1);
yval=Close(i,2);
zval=Close(i,3);
yawval = Close(i,4);
x_node = Close(i,9);
y_node = Close(i,10);
z_node = Close(i,11);

i=1;
Optimal_path(i,1)=xval;
Optimal_path(i,2)=yval;
Optimal_path(i,3)=zval;
i=i+1;

if (x_node == x_target && y_node == y_target && z_node == z_target)
    inode=0;
   %Traverse Close and determine the parent nodes
   parent_x=Close(node_index(Close,xval,yval,zval,yawval),5);%node_index returns the index of the node
   parent_y=Close(node_index(Close,xval,yval,zval,yawval),6);
   parent_z=Close(node_index(Close,xval,yval,zval,yawval),7);
   parent_yaw=Close(node_index(Close,xval,yval,zval,yawval),8);
   while( parent_x ~= x_start || parent_y ~= y_start || parent_z ~= z_start)
           Optimal_path(i,1) = parent_x;
           Optimal_path(i,2) = parent_y;
           Optimal_path(i,3) = parent_z;
           %Get the grandparents:
           inode=node_index(Close,parent_x,parent_y,parent_z,parent_yaw);
           parent_x=Close(inode,5);%node_index returns the index of the node
           parent_y=Close(inode,6);
           parent_z=Close(inode,7);
           parent_yaw = Close(inode,8);
           i=i+1;
   end
j=size(Optimal_path,1);
plot3(Optimal_path(:,1)+.5,Optimal_path(:,2)+.5,Optimal_path(:,3)+.5,'b','linewidth',5);
waypoints = Optimal_path;
else
 pause(1);
 h=msgbox('Sorry, No path exists to the Target!','warn');
 uiwait(h,5);
end

end % end of the function

% function to populate the open list
function new_open = Insert_open(current_state,parent_state,g_cost,h_cost,f_cost,current_node)
    % open list  | x | y | z | yaw | parent x | parent y | parent z |
    % parent yaw | g(n) | h(n) | f(n) | x_node | y_node | z_node |
    new_open = [1,14];
    new_open(1,1) = current_state(1);
    new_open(1,2) = current_state(2);
    new_open(1,3) = current_state(3);
    new_open(1,4) = current_state(4);
    new_open(1,5) = parent_state(1);
    new_open(1,6) = parent_state(2);
    new_open(1,7) = parent_state(3);
    new_open(1,8) = parent_state(4);
    new_open(1,9)  = g_cost;
    new_open(1,10) = h_cost;
    new_open(1,11) = f_cost;
    new_open(1,12) = current_node(1);
    new_open(1,13) = current_node(2);
    new_open(1,14) = current_node(3);
end

% function to populate the close list
function new_close = Insert_close(current_state,parent_state,current_node)
    % close list | x | y | z | yaw | parent x | parent y | parent z | parent yaw |
    new_close = [1,11];
    new_close(1,1) = current_state(1);
    new_close(1,2) = current_state(2);
    new_close(1,3) = current_state(3);
    new_close(1,4) = current_state(4);
    new_close(1,5) = parent_state(1);
    new_close(1,6) = parent_state(2);
    new_close(1,7) = parent_state(3);
    new_close(1,8) = parent_state(4);
    new_close(1,9) = current_node(1);
    new_close(1,10) = current_node(2);
    new_close(1,11) = current_node(3);
end

% function to return the node index with minimum f cost
function i_min = min_fn(Open)
    if size(Open) ~= 0
        [min_fn,i_min] = min(Open(:,11));
    else
        i_min = -1;
    end
end

function n_index = node_index(Close,xval,yval,zval,yawval)
    %This function returns the index of the location of a node in the list close
    i=1;
    while(Close(i,1) ~= xval || Close(i,2) ~= yval || Close(i,3) ~= zval ||Close(i,4) ~= yawval)
        i=i+1;
    end
    n_index=i;
end




