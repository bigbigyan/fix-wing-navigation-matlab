clear
% map data
load("Makedata.mat");

% satrt and goal point
startpoint = [20,20,10,0];  % x,y,z,yaw  unit: 10m
goalpoint = [90,70,10,pi/2];

% property of fix-wing
cruise_velocity = 20; % unit: m/s
cruise_pitch = 0; % unit: rad 0 ~ max_pitch
max_roll = pi/6; % unit: rad 0 ~ pi/2
max_pitch = pi/3; % unit: rad 0 ~ pi/2
max_yaw = pi/3; % unit: rad 0 ~ pi/2
uav_property = [cruise_velocity,max_roll,max_pitch,max_yaw,cruise_pitch];

% algorithm
% compute time
tic
timeVal = tic;
[waypoints,open_count] = HybridAStar3D(max_x,max_y,max_z,startpoint,goalpoint,map,display_data,uav_property);
toc(timeVal)
elapsedTime = toc(timeVal);

%B-spline
for i = 1:size(waypoints,1)
    x(i) = waypoints(i,1);
    y(i) = waypoints(i,2);
    z(i) = waypoints(i,3);
end
k = 4;
[X,Y,Z] = Bspline(x,y,z,k);

%figure
figure(1)
axis([1 max_x 1 max_y 1 max_z]);
plot3(waypoints(:,1),waypoints(:,2),waypoints(:,3),'b','linewidth',1);
hold on
plot3(X,Y,Z,'r','linewidth',1);
hold on
surf(display_data(1:100,1:100)','linestyle','none');
plot3(20,20,10,'*');
plot3(90,70,10,'^');
set(gca,'xticklabel','');
set(gca,'yticklabel','');
set(gca,'zticklabel',{'2000','4000','6000','4000','5000','6000','7000','8000','9000','10000'});
xlabel('纬度');
ylabel('经度');
zlabel('高度（m）');
grid on
%%%%%%%绘制垂直剖面航图
% figure(2)
% X_WayPoints = waypoints(end:-1:1,1);
% Y_WayPoints = waypoints(end:-1:1,2);
% Z_WayPoints = waypoints(end:-1:1,3);
% Total_X_WayPoints = [20 X_WayPoints'];
% Total_Y_WayPoints = [20 Y_WayPoints'];
% Total_Z_WayPoints = [7 Z_WayPoints'];
% Terrain_Data = Final_Data(301:400,101:200);
% num = size(Total_X_WayPoints);
% for i= 1:num(1,2)
%     Terrain_Z_WayPoints(i) = Terrain_Data(floor(Total_X_WayPoints(1,i)),floor(Total_Y_WayPoints(1,i)));
% end
% lat_lonD = [];
% lat_lonDisReal = [];
% lat_lonDisReal(1) = 0;
% plat = (37.3565 - (25/54)*Total_X_WayPoints./100)';
% plon = (101.7130 + (25/54)*Total_Y_WayPoints./100)';
% pi=3.1415926;
% num = size(plat)-1;
% for i = 1:num(1,1)
%     lat_lonD(i)=distance(plat(i),plon(i),plat(i+1),plon(i+1));
%     lat_lonD(i)=lat_lonD(i)*6371*2*pi/360;
%     lat_lonDisReal(i+1) = lat_lonDisReal(i) + lat_lonD(i);
% end
% MIN_Final_Data = min(min(Final_Data(301:400,101:200)));
% Total_Z_WayPoints = Total_Z_WayPoints.*100 + MIN_Final_Data;
% h1 = plot(lat_lonDisReal,Total_Z_WayPoints,'b');
% hold on
% plot(lat_lonDisReal,Terrain_Z_WayPoints,'c');
% h2 = plot(lat_lonDisReal,Terrain_Z_WayPoints + 1000,'r');
% X_fill = lat_lonDisReal;
% Y_fill = Terrain_Z_WayPoints;
% Y_size = size(Y_fill);
% Y_fill_low = zeros(Y_size(1,1),Y_size(1,2));
% X_fillfor = [fliplr(X_fill),X_fill];
% Y_fillfor = [fliplr(Y_fill_low),Y_fill];
% h3 = fill(X_fillfor,Y_fillfor,'c','FaceAlpha',1,'EdgeAlpha',0.3,'EdgeColor','k');
% hleg = legend([h1,h2,h3],'规划航迹垂直剖面投影','低空飞行上边界','地形垂直剖面');
% set(hleg,'Location','NorthWest','Fontsize',8);
% hold off
% xlabel('飞行路程（km）');
% ylabel('飞行高度（m）');
% xmaxTeam = lat_lonDisReal(1,num+1);
% xmax = xmaxTeam(1,1);
% axis([0 xmax 2500 5500]);
% grid on



