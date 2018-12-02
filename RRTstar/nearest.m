function [ q_nearest ] = nearest( G_node,q_rand,node_num)
%选择离取样点最近的节点
% %原始
% qdist = [];
% node_tmp = [];
%  for n = 1:node_num
%      qdist = [qdist,dist(G_node(n).coord,q_rand.coord')];
%      node_tmp = [node_tmp G_node(n)];
%  end
% [A I] = min(qdist);
%  q_nearest = node_tmp(I);

%优化
G_node_tmp = [G_node];
q_rand_tmp = repmat(q_rand,1,node_num);
tmp_G_node_coord = [G_node_tmp.coord];
tmp_q_rand_coord = [q_rand_tmp.coord];
dx_dy = (tmp_G_node_coord - tmp_q_rand_coord).^2;
G_node_dist = sqrt(dx_dy(1:2:2*node_num)+dx_dy(2:2:2*node_num));
[A I] = min(G_node_dist);
q_nearest = G_node(I);
end

