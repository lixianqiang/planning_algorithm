function [ result ] = obstaclefree( q_nearest,q_new,map)
%判断最近点与更新点是否存在障碍物
%     y=k*x+b
    dy = q_new.coord(2) - q_nearest.coord(2);
    dx = q_new.coord(1) - q_nearest.coord(1);
    k = dy/dx;
    b = q_nearest.coord(2) - k * q_nearest.coord(1);
    if k ~= Inf && k ~= -Inf
        if dx >0
            for x = q_nearest.coord(1):(dx/9):q_new.coord(1)
                y = k * x + b;
                if map(fix(y),fix(x)) == 1
                    result =  false;
                    return
                end
            end
            result = true;
            return
        else
            for x = q_new.coord(1):(-dx/9):q_nearest.coord(1)
                y = k * x+b;
                if map(fix(y),fix(x)) == 1
                    result =  false;
                    return
                end
            end
            result = true;
            return
        end
    elseif k == Inf
        for y = q_nearest.coord(2):(dy/9):q_new.coord(2)
            if map(fix(y),fix(x)) == 1
                result =  false;
                return
            end
        end
        result = ture;
        return
    else
        for y =q_new.coord(2):(-dy/9):q_nearest.coord(2)
            if map(fix(y),fix(x)) == 1
                result =  false;
                return
            end
        end
        result = true;
        return
    end
end

                
           
        
        

    
    

