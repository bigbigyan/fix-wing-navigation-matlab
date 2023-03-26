% function to generate the motion primitives
function motion_primitives = generate_primitives(current_node,uav_property)
    motion_primitives = []; % [x,y,z,yaw]
    primitives_count = 1;

    x_current = current_node(1);
    y_current = current_node(2);
    z_current = current_node(3);
    yaw_current = current_node(4);

    velocity = uav_property(1);
    % roll = uav_property(2);
    pitch = uav_property(3);
    yaw = uav_property(4);
    cruise_pitch = uav_property(5);

    for p = pitch:-pitch:-pitch
        for y = yaw:-yaw:-yaw
            v = velocity/10;
            vz = v * sin(p/2-cruise_pitch);
            vx = v * cos(p/2-cruise_pitch) * cos(y/2 + yaw_current);
            vy = v * cos(p/2-cruise_pitch) * sin(y/2 + yaw_current);

            hx = vx * 1; % vx * time
            hy = vy * 1; % vy * time
            hz = vz * 1; % vz * time

            motion_primitives(primitives_count,1) = x_current + hx;
            motion_primitives(primitives_count,2) = y_current + hy;
            motion_primitives(primitives_count,3) = z_current + hz;
            yaw = yaw_current + y;
            if yaw > 2*pi
                yaw = yaw - 2*pi;
            end
            if yaw < -2*pi
                yaw = yaw + 2*pi;
            end
            motion_primitives(primitives_count,4) = yaw;

            primitives_count = primitives_count + 1;
        end
    end

end