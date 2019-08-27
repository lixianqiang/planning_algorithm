function [ openlist closelist] = expend(openlist,closelist,map,q_goal)
%À©Õ¹½Úµã

[bb index]=sort([openlist.f]);

node = openlist(index(1));

q1=[node.coord(1)-1 node.coord(2)+1];
q2=[node.coord(1)-1 node.coord(2)];
q3=[node.coord(1)-1 node.coord(2)-1];

q4=[node.coord(1) node.coord(2)+1];
q5=[node.coord(1) node.coord(2)-1];

q6=[node.coord(1)+1 node.coord(2)+1];
q7=[node.coord(1)+1 node.coord(2)];
q8=[node.coord(1)+1 node.coord(2)-1];

q={q1 q2 q3 q4 q5 q6 q7 q8};

openlist(index(1))=[];
closelist=[closelist node];
opnum = size(openlist,2);
clnum = size(closelist,2);
totalnum = opnum+clnum;
count = 1;
xlim=size(map,1);
ylim=size(map,2);
for i=1:8
    if q{i}(1)< 1|| q{i}(2)>ylim || q{i}(1)>xlim || q{i}(2)<1
        continue
    end
    if map(q{i}(2),q{i}(1))==1
        continue
    end
    
    cl_sig = [];
    for k = 1:clnum
        sig = closelist(k).coord == q{i};
        cl_sig = [cl_sig sig(1)+sig(2)];
    end
    idx = find(cl_sig == 2);
    if ~isempty(idx)
        continue
    end

    if opnum == 0
        tmp_node.coord = q{i};
        tmp_node.g = round(10*sqrt(abs(node.coord(1)-tmp_node.coord(1))+abs(node.coord(2)-tmp_node.coord(2))));
        tmp_node.h = 10*(abs(q_goal.coord(1)-tmp_node.coord(1))+abs(q_goal.coord(2)-tmp_node.coord(2)));
        tmp_node.f = tmp_node.g + tmp_node.h;
        tmp_node.num= totalnum+count;
        tmp_node.parent = node.num;
        count = count+1;
        openlist = [openlist tmp_node];
        continue
    end
    
    op_sig = [];
    for t=1:opnum
        sig = openlist(t).coord == q{i};
        op_sig = [op_sig sig(1)+sig(2)];
    end
    idx = find(op_sig==2);
    
    if ~isempty(idx)
        tmp_node.coord = q{i};
        tmp_node.g = round(10*sqrt(abs(node.coord(1)-tmp_node.coord(1))+abs(node.coord(2)-tmp_node.coord(2))));
        tmp_node.h = 10*(abs(q_goal.coord(1)-tmp_node.coord(1))+abs(q_goal.coord(2)-tmp_node.coord(2)));
        if tmp_node.g < openlist(idx).g
            tmp_node.f = tmp_node.g + tmp_node.h;
            tmp_node.num = openlist(idx).num;
            tmp_node.parent = node.num;
            openlist(idx)=[];
            openlist = [openlist tmp_node];
        end
    else
        tmp_node.coord = q{i};
        tmp_node.g = round(10*sqrt(abs(node.coord(1)-tmp_node.coord(1))+abs(node.coord(2)-tmp_node.coord(2))));
        tmp_node.h = 10*(abs(q_goal.coord(1)-tmp_node.coord(1))+abs(q_goal.coord(2)-tmp_node.coord(2)));
        tmp_node.f = tmp_node.g + tmp_node.h;
        tmp_node.num= totalnum+count;
        tmp_node.parent = node.num;
        count = count+1;
        openlist = [openlist tmp_node];
    end
end
end



