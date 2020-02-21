function [car_pos,car_point,car_laider,r]=ego_car(x,y)
r=25; %라이다 센서 범위
t=0:0.01:2*pi;
X=(x(1)+x(2)+x(3)+x(4))/4+r*cos(t);
Y=(y(1)+y(2)+y(3)+y(4))/4+r*sin(t);
X1=(x(1)+x(2)+x(3)+x(4))/4+0.1*cos(t);
Y1=(y(1)+y(2)+y(3)+y(4))/4+0.1*sin(t);
car_pos=fill(x,y,'r');
car_point=plot(X1,Y1,'b');
car_laider=plot(X,Y);

