function map = map_generate(map_xy_size,obs_xy_size,obs_num,CoE,rand_or_quadrotor)
%生成随机地图/固定地图
if nargin == 3 || nargin == 4
    CoE = 1;
    rand_or_fix = "rand";
end

% figure
% hold on
map = zeros(map_xy_size(2),map_xy_size(1));
block = ones(obs_xy_size(2)+1+2*(CoE-1),obs_xy_size(1)+1+2*(CoE-1));
axis([0,map_xy_size(1),0,map_xy_size(2)])

% %改变坐标轴位置
% set(gca,'XAxisLocation','top')     
% set(gca,'ydir','reverse')    

i = 1; 
if rand_or_quadrotor == "rand"
    while i ~= obs_num+1
        grid_xy = fix([rand(1)*(map_xy_size(1)-1)+1 , rand(1)*(map_xy_size(2)-1)+1]);
        obs_xy = [grid_xy(1)+CoE,grid_xy(2)+CoE];
        if grid_xy(1)+ obs_xy_size(1) + 2*(CoE-1) < map_xy_size(1) && grid_xy(2)+obs_xy_size(2) + 2*(CoE-1) < map_xy_size(2)
            rectangle('Position',[obs_xy(1) obs_xy(2) obs_xy_size(1) obs_xy_size(2)],'FaceColor',[0 .5 .5]);
            map(grid_xy(2):grid_xy(2)+2*(CoE-1)+obs_xy_size(2),grid_xy(1):grid_xy(1)+2*(CoE-1)+obs_xy_size(1)) = block;
            i = i+1;
        end
    end
% 固定地图
elseif rand_or_quadrotor == "quadrotor"
    rectangle('Position',[obs_xy(1) obs_xy(2) obs_xy_size(1) obs_xy_size(2)],'FaceColor',[0 .5 .5]);
    map(13:15,1:3) = ones(3,3);
%     rectangle('Position',[2 14 1 1],'FaceColor',[0 .5 .5]);
    map(8:10,4:6) = ones(3,3);
%     rectangle('Position',[5 9 1 1],'FaceColor',[0 .5 .5]);
    map(3:5,7:9) = ones(3,3);
%     rectangle('Position',[8 4 1 1],'FaceColor',[0 .5 .5]);
    map(15:17,8:10) = ones(3,3);
%     rectangle('Position',[9 16 1 1],'FaceColor',[0 .5 .5]);
    map(10:12,12:14) = ones(3,3);
%     rectangle('Position',[13 11 1 1],'FaceColor',[0 .5 .5]);
    map(5:7,16:18) = ones(3,3);
%     rectangle('Position',[17 6 1 1],'FaceColor',[0 .5 .5]);
end

end

