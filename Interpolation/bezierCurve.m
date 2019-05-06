%Bezier Curve 
function bezierCurve()
P0 = [0,0];
P1 = [1,1];
P2 = [1,2];
P3 = [0,2];
dt = 0.01;

b0 = P0;
hold on;
for t = 0:dt:1
    b1 = Bezier1(P0,P1,t);
    plot([b0(1) b1(1)],[b0(2) b1(2)],'-');
    b0 = b1;
end

b0 = P0;
hold on;
for t = 0:dt:1
    b1 = Bezier2(P0,P1,P2,t);
    plot([b0(1) b1(1)],[b0(2) b1(2)],'-');
    b0 = b1;
end

b0 = P0;
hold on;
for t = 0:dt:1
    b1 = Bezier3(P0,P1,P2,P3,t);
    
    plot([b0(1) b1(1)],[b0(2) b1(2)],'-');
    b0 = b1;
end

end

%% 贝塞尔曲线
%1阶贝塞尔曲线
function B = Bezier1(p0,p1,t)
    B = p0 + (p1-p0)*t;
end

%2阶贝塞尔曲线
function B = Bezier2(p0,p1,p2,t)
    B = (1-t)^2*p0 + 2*t*(1-t)*p1 + t^2*p2;
end

%3阶贝塞尔曲线
function B = Bezier3(p0,p1,p2,p3,t)
    B = (1-t)^3*p0 + 3*p1*t*(1-t)^2 + 3*p2*t^2*(1-t) + p3*t^3;
end
% 
% %n阶B、贝塞尔曲线
% function B = Beziern(pn,t)
%     
% end