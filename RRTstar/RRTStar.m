clear ;clc; close all 

%绘制地图
obs_xy_size = [5 5];
map_xy_size = [100 100];
obs_num = 10;
CoE = 1;
rand_or_quadrotor = "rand";
map = map_generate(map_xy_size,obs_xy_size,obs_num,CoE,rand_or_quadrotor);

%参数
d_steer = 1;
r_near = 4;

%节点集合G_node,数量num
G_node = [];
node_num = 1;
vertex = 1000;

%起点与终点
q_start.coord = [1 1];
q_start.theta = 0;
q_start.cost  = 0;
q_start.parent = 1;
q_start.num = 1;
q_goal.coord  = [99 99];
q_goal.theta = pi/2;

G_node = [G_node q_start];

%扩展节点
while(node_num ~= vertex)
    %随机节点生成
    while(1)
        q_rand.coord= [(map_xy_size(1)-1)*rand(1)+1 (map_xy_size(2)-1)*rand(1)+1];
        if map(fix(q_rand.coord(2)),fix(q_rand.coord(1))) ~=1
            break
        end
    end
    if(node_num == vertex-1)
            q_rand = q_goal;
    end
    q_nearest = nearest(G_node,q_rand,node_num);
    q_new = steer(q_nearest,q_rand,d_steer);  
    q_new.cost = q_nearest.cost + dist(q_nearest.coord,q_new.coord');
    q_new.parent = q_nearest.num;
    q_new.num = node_num + 1;

    if obstaclefree(q_nearest,q_new,map) == true
        G_q_near = near(G_node,q_new,node_num,r_near);
        c_min = [];
        x_min = [];
        for i = 1:length(G_q_near)
            if obstaclefree(G_q_near(i),q_new,map) == true
                c_min = [c_min G_q_near(i).cost + dist(G_q_near(i).coord,q_new.coord')];
                x_min = [x_min G_q_near(i)];
            end
        end
        [cost_min,index] = min(c_min);
        if cost_min < q_new.cost
            q_new.cost = cost_min;
            q_new.parent = x_min(index).num;                        
        end
        % E <- E U {x_min,x_new}
        for i = 1:length(x_min)
            if x_min(i).cost > q_new.cost + dist(x_min(i).coord,q_new.coord')
                idx = find([G_node.num] == x_min(i).num);
                G_node(idx).parent = q_new.num;
            end
        end
    else
        continue
    end
    
    G_node = [G_node q_new];
    node_num = node_num+1;
    %看是否已经到达终点，如果不是就扩展vertex的数目+1
    while(node_num == vertex)
        if G_node(node_num).coord ~= q_goal.coord
            vertex = vertex + 1;
            break
        end
        break
    end
end

%生成树状图
    
    %优化
    parent = [];
    parent = [parent G_node.parent];
    for i = 1:node_num
        line([G_node(i).coord(1),G_node(parent(i)).coord(1)],[G_node(i).coord(2),G_node(parent(i)).coord(2)],'Color','red');
    end
    
    %原始
% num = 0;
% for n = 1:node_num
%     parent = find([G_node.num] == n);
%     son = find([G_node.parent] == n);
%     s_num = length(son);
%     for i = 1:s_num
%         line([G_node(parent).coord(1),G_node(son(i)).coord(1)],[G_node(parent).coord(2),G_node(son(i)).coord(2)],'Color','red')
%     end
%     num = num + s_num;
%     if num == node_num
%         break
%     end
% end

    %优化
index = node_num;
next_index = node_num;
route_node = G_node(index);
while(G_node(next_index).num ~= 1)
    next_index = G_node(index).parent;
    route_node=[route_node;G_node(next_index)];
    index = next_index;
end
    %对节点倒序
route_node = route_node(end:-1:1,1);
route_xy = [];
yaw = [];
route_xy = [route_xy;route_node.coord];
yaw = [yaw route_node.theta];
line(route_xy(1:2:end),route_xy(2:2:end),'Color','blue');
%原始
% index = node_num;
% next_index = node_num;
% while(G_node(next_index).num ~= 1)
%     next_index = G_node(index).parent;
%     line([G_node(index).coord(1),G_node(next_index).coord(1)],[G_node(index).coord(2),G_node(next_index).coord(2)],'Color','blue');
%     index = next_index;
% end



