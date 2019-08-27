%Bezier Curve 
% Reference link
%   https://zh.wikipedia.org/wiki/%E8%B2%9D%E8%8C%B2%E6%9B%B2%E7%B7%9A
%   https://zh.wikipedia.org/wiki/%E5%BE%B7%E5%8D%A1%E6%96%AF%E7%89%B9%E9%87%8C%E5%A5%A5%E7%AE%97%E6%B3%95
%   https://blog.csdn.net/Fioman/article/details/2578895
%   https://www.qiujiawei.com/bezier-1/
function BezierCurve()
 x=[1,2,3,4,5,6,7,8,9];
 y=[4.5,6.5,1.2,3.4,5.6,4.8,8.8,1.2,9.9];
% x=[1,1,10,10];
% y=[1,10,10,1];

% f=bezierCurveGen(x,y);
f=bezierCurve(x,y);

t=[0:0.01:1];
xy=subs(f,'t',t);
xy=double(xy)
hold on
plot(x,y,'y')
plot(xy(1,:),xy(2,:),'p')
end

%This function apply De Casteljau's Algorithm to achieve the drawing of BezierCurve 
function func = bezierCurve(x,y)
if length(x)~=length(y)
    disp('please check the num of set [x,y]');
    return
end
n=length(x);
p=arrayfun(@(x,y)[x;y],zeros(n,n),zeros(n,n),'UniformOutput',false);
for i=1:n
    p{1,i}=[x(i);y(i)];
end
syms t
for j=2:n
    for i=1:n+1-j
        p{j,i}=p{j-1,i}*(1-t)+p{j-1,i+1}*t;
    end
end
func = p{j,1};
end

%This Function written according to the general definition of BezierCurve 
function func = bezierCurveGen(x,y)
if length(x)~=length(y)
    disp('please check the num of set [x,y]');
    return
end
syms t
n=length(x)-1;

for i=0:n
    C(1,i+1) = factorial(n)/(factorial(i)*factorial(n-i));
    b(i+1,1) = [C(1,i+1)*t^(i)*(1-t)^(n-i)];
end
P=[x;y];
func=P*b;
end
