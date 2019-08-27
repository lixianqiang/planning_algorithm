function [ G_q_near ] = near( G_node,q_new,node_num,r)
%寻找q_new附近少于额定阈值的节点集合q_near

%原始
% G_q_near = [];
% for n=1:node_num
%         if dist(G_node(n).coord,q_new.coord')<r
%             G_q_near = [G_q_near G_node(n)];
%         end
% end

%优化
G_node_tmp = [G_node];
q_new_tmp = [repmat(q_new,1,node_num)];
tmp_G_node_coord = [G_node_tmp.coord];
tmp_q_new_coord = [q_new_tmp.coord];
dx_dy = (tmp_G_node_coord - tmp_q_new_coord).^2;
G_node_dist = sqrt(dx_dy(1:2:2*node_num)+dx_dy(2:2:2*node_num));
[idx] = find(G_node_dist < r);
G_q_near = [G_node_tmp(idx)];
end
