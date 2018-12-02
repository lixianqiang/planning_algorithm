clear ;clc; close all 

%绘制地图
obs_xy_size = [5 5];
map_xy_size = [100 100];
obs_num = 10;
CoE = 5;
rand_or_quadrotor = "rand";
map = map_generate(map_xy_size,obs_xy_size,obs_num,CoE,rand_or_quadrotor);

%起点与终点,初始化
q_start.coord = [1 1];
q_start.g  = 0;
q_start.h = 0;
q_start.f = 0;
q_start.parent = 0;
q_start.num = 1;
q_goal.coord  = [99 99];

node = [q_start];
openlist = [q_start];
closelist = [];

%搜索开始
while(1)
[openlist closelist] = expend(openlist,closelist,map,q_goal);
tnum= size(closelist,2);
tmpgoal_sig = [];
for k = 1:tnum
    goalsig = (closelist(k).coord == q_goal.coord);
    tmpgoal_sig = [tmpgoal_sig goalsig(1)+goalsig(2)];
end
index = find(tmpgoal_sig==2);
if isempty(index)
    continue
end
node_index = index;
node_num = closelist(node_index).parent;
next_index=find([closelist.num]==node_num);
while(closelist(node_index).num~=1)
    line([closelist(node_index).coord(1),closelist(next_index).coord(1)],[closelist(node_index).coord(2),closelist(next_index).coord(2)]);
    node_index=next_index;
    node_num=closelist(node_index).parent;
    next_index=find([closelist.num]==node_num);
end
break
end
xlabel("X轴(m)")
ylabel("Y轴(m)")

