% Cubic Spline 
% Reference link 
%   https://blog.csdn.net/zb1165048017/article/details/48311603
%   https://zh.wikipedia.org/wiki/%E6%A0%B7%E6%9D%A1%E6%8F%92%E5%80%BC
%   https://blog.csdn.net/AdamShan/article/details/80696881
function CubicSpline()
% x=0:2*pi;
% y=sin(x);
x=[1,2,3,4,5,6,7,8,8.1]';
y=[4.5,6.5,1.2,3.4,5.6,4.8,8.8,1.2,1.21]';
dt=0.01;
dP1=0.5;
dPn=6;
% f=cubicSpline(x,y,dP1,dPn);

[f a b c d]=cubicSpline(x,y);
pXY=[];
hIdx=1;
for i=1:length(x)-1
    px=[x(i):dt:x(i+1)];
    py=subs(f(i),'t',px);
    n=length(px);
    xxx=px(1,hIdx:n);
    yyy=double(py(1,hIdx:n));
    pXY=[pXY [px(1,hIdx:n); double(py(1,hIdx:n))]];
    hIdx=2;
end
hold on
plot(x,y,'b');
plot(pXY(1,:),pXY(2,:),'r');
end

%This Function include two type of boundary conditions
% Respectively, including condition (a) and (b)
% a) S'(x1)=y1' && S'(xn)=yn'
% b) S'(x1)=y1' && S'(xn)=yn' && S''(x1)=S''(xn)=0  
function [func,a,b,c,d] = cubicSpline(x,y,varargin)
if length(x)~=length(y)
    disp('the number of set[x,y] no match,please check it');
    return 
end
n=length(x);
A=zeros(n);
e=zeros(n,1);
h=zeros(n,1);

for i=2:n
    h(i-1)=x(i)-x(i-1);
end

if ~isempty(varargin) && length(varargin)==2
    % condition a)
    dP1=varargin{1};
    dPn=varargin{2};
    A(1,1:2)=[2,1];
    A(n,n-1:n)=[1,2];
    e(1)=6*((y(2)-y(1))/h(1)-dP1)/h(1);
    e(n)=6*(dPn-(y(n)-y(n-1))/h(n-1))/h(n-1);
else
    % condition b)
    A(1,1)=1;
    A(n,n)=1;
    e(1)=0;
    e(n)=0;
end

for i=2:n-1
    A(i,i-1:i+1) = [h(i-1),2*(h(i-1)+h(i)),h(i)];
    e(i) = 6 * ((y(i+1)-y(i))/h(i) - (y(i)-y(i-1))/h(i-1));
end

M=A\e;
syms t
for i=1:length(M)-1
    a(i)=(x(i+1)-t).^3.*M(i)./(6*h(i));
    b(i)=(t-x(i)).^3.*M(i+1)./(6*h(i));
    c(i)=((y(i+1)-y(i))./h(i)-(M(i+1)-M(i)).*h(i)/6).*t;
    d(i)=y(i+1) - M(i+1)*h(i)^2/6 - ((y(i+1)-y(i))/h(i) - (M(i+1)-M(i))*h(i)/6)*x(i+1);
    func(i)=a(i)+b(i)+c(i)+d(i);
end
for i=1:length(M)-1
    d(i)=y(i+1) - M(i+1)*h(i)^2/6 - ((y(i+1)-y(i))/h(i) - (M(i+1)-M(i))*h(i)/6)*x(i+1);
end
end
