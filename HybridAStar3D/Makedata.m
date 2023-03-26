% function Makedata()

% make data of map
load("TerrainData.mat")

% define the 3D map array
max_x = 100;
max_y = 100;
max_z = 50;

cut_data = Final_Data(301:400 , 101:200);

mesh(double(cut_data));

min_final_data = min(min(cut_data));

for i = 1:max_x
    for j = 1:max_y
        new_data(i,j) = ceil((cut_data(i,j) - min_final_data)/100); 
        display_data(i,j) = (cut_data(i,j) - min_final_data)/100;
    end
end

% map matrix initialization
    % space value = 0 obstacle value = 1
map = zeros(max_x , max_y , max_z);

for i = 1:max_x
    for j = 1:max_y
        z_date = new_data(i,j);
        for z = 1:z_date
            map(i,j,z) = 1;
        end
    end
end

save('Makedata','max_x',"max_y",'max_z','map','display_data','Final_Data')







