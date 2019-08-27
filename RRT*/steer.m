function [ q_new ] = steer(q_nearest,q_rand,d)
% 移动到新节点
    dx = q_rand.coord(1) - q_nearest.coord(1);
    dy = q_rand.coord(2) - q_nearest.coord(2);
    k = dy/dx;
    b = q_nearest.coord(2) - k * q_nearest.coord(1);
    xn1 = q_nearest.coord(1) + sqrt(d^2/(1+k^2));
    yn1 = k * xn1 + b;
    xn2 = q_nearest.coord(1) - sqrt(d^2/(1+k^2));
    yn2 = k * xn2 + b;
    if k ~= Inf && k ~= -Inf
        if dist(q_rand.coord,q_nearest.coord') > d
            if dx>0
                if xn1 - q_nearest.coord(1) >0
                    q_new.coord = [xn1,yn1];
                else
                    q_new.coord = [xn2,yn2];
                end
            else
               if xn1 - q_nearest.coord(1) <0
                   q_new.coord = [xn1,yn1];                   
               else
                   q_new.coord = [xn2,yn2];                   
               end
            end
        else
            q_new.coord = q_rand.coord;
        end
    elseif k == Inf
        if dist(q_rand.coord,q_nearest.coord') > d
            q_new.coord = [q_nearest.coord(1),q_nearest.coord(2)+d];
        else
            q_new.coord = q_rand.coord;        
        end
    else
        if dist(q_rand.coord,q_nearest.coord') > d
            q_new.coord = [q_nearest.coord(1),q_nearest.coord(2)-d];
        else
            q_new.coord = q_rand.coord;
        end
    end
    %节点距离较小的情况下适用
    theta = atan2(dy,dx);
    if theta-q_nearest.theta > 0
        q_new.theta = q_nearest.theta+0.2;
    else
        q_new.theta = q_nearest.theta-0.2;
    end
    %节点距离较大情况下适用
%     theta = atan2(dy,dx);
%     if theta < 0                     %将theta从[-pi pi]转变成[0 2pi]
%         q_new.theta = theta + 2*pi;
%     else
%         q_new.theta = theta;
%     end
end

