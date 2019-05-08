% Cubic Spline 
% Reference link 
%   https://blog.csdn.net/zb1165048017/article/details/48311603
%   https://zh.wikipedia.org/wiki/%E6%A0%B7%E6%9D%A1%E6%8F%92%E5%80%BC
%   https://blog.csdn.net/AdamShan/article/details/80696881
function CubicSpline()
 x=[1,2,3,4,5,6,7,8,9];
 y=[4.5,6.5,1.2,3.4,5.6,4.8,8.8,1.2,9.9];
 f=cubicSpline(x,y);
 for i=1:length(x)-1
     px=[x(i):0.1:x(i+1)];
     py=subs(f(i),'t',px);
     hold on
     plot(x,y,'b');
     plot(px,py,'r');
 end
end

function func = cubicSpline(x,y)
if length(x)~=length(y)
    disp('the number of set[x,y] no match,please check it');
    return 
end
A=zeros(size([x;y]',1));
A(1,1)=1;
A(length(x),length(x))=1;
e=zeros(size([x;y]',1),1);
for i=2:1:length(x)-1
    h(i-1) = x(i)-x(i-1);
    h(i) = x(i+1)-x(i);
    A(i,i-1:i+1) = [h(i-1),2*(h(i-1)+h(i)),h(i)];
    e(i) = 6 * ((y(i+1)-y(i))/h(i) - (y(i)-y(i-1))/h(i-1));
end
M=A\e;
syms t
for i=1:length(M)-1
    a(i)=(x(i+1)-t)^3*M(i)/(6*h(i));
    b(i)=(t-x(i))^3*M(i+1)/(6*h(i));
    c(i)=((y(i+1)-y(i))/h(i)-(M(i+1)-M(i))*h(i)/6)*t;
    d(i)=y(i+1) - M(i+1)*h(i)^2/6 - ((y(i+1)-y(i))/h(i) - (M(i+1)-M(i))*h(i)/6)*x(i+1);
    func(i)=a(i)+b(i)+c(i)+d(i);
end
end
