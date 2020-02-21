function [xd,yd]=pathplanning(v,v_front,index,yd_old,xd_old,x,width)
t=3; %차선 변경하는데 필요한 시간
d=ceil(t/2*v); %차선 변경하는 동안 x방향 이동거리/2
node = 1; %node 간격

s= 20+v_front*6; %추월 차선에서 이동거리
x0=xd_old(1:index);
x1=x:node:2*d+x;
x2=x+2*d+node:x+s+2*d;
x3=x+s+2*d+node:node:x+s+4*d+node;

y0=yd_old(1:index);
y1=width/(2*d^2)*(x1(1:floor(length(x1)/2))-x).^2+width/2;
y2=-width/(2*d^2)*(x1(floor(length(x1)/2)+1:end)-2*d-x).^2+3/2*width;
y3=y2(end)*ones(1,length(x2));
y4=y2(end:-1:1);%-width/(2*d^2)*(-x3(floor(length(x3)/2)+1:end)-2*d-x).^2+3/2*width+4*d+s;
y5=y1(end:-1:1);%width/(2*d^2)*-x3(1:floor(length(x3)/2)-x).^2+width/2+4*d;

if(length(xd_old)>length([x0 x1 x2 x3]))
    x4=xd_old(length([x0 x1 x2 x3]+node):end);
    y6=yd_old(length([x0 x1 x2 x3]+node):end);
    xd=[x0 x1 x2 x3 x4];
    yd=[y0 y1 y2 y3 y4 y5 y6];
else
    xd=[x0 x1 x2 x3];
    yd=[y0 y1 y2 y3 y4 y5];
end

plot(xd,yd,'--')


