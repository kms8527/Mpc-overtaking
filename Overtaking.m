clear all
close all
clc
%%
%initial values

%simulation time
delt= 200e-3;
tf=40;
t=0:delt:tf;

%road information
d=300; % 끝까지 거리
width = 4.5; %차선 너비
car_width=1.8; %m 단위

%constrain variables 
rr=1; %weight value

N=20; % predicted step number
umax = 5 *pi/180;  %unit : rad %maximum input
ymax = width + width; %unit : m %upper lane value when there isnt a obstacle
ymin = 0; %unit : m %lower lane value when there isnt a obstacle

%information of cars
L=4.57; %m 단위 %car_length

s=0;
s_front=0;
s_adj1=0;
s_adj2=0;

v=2; % unit:m/s, velocity
ego_lim=10; %m/s
a=0;
v_front=1; front_lim=0; %m/s
a_front=0;
v_adj1=1; adj1_lim=0;
a_adj1=1;
v_adj2=1; adj2_lim=0;
a_adj2=1;

init_x=50; % 초기 ego vehicle x  좌표
init_y=0;
init_x1=90;
init_y1= width;
init_x2=50;
init_y2=width;

%for MPC constraints matrix
bu=umax*ones(2*N,1);
by=[ymax*ones(N,1); -ymin*ones(N,1)];
%G,H,F,D definition for mpc
[H,F,G,D]=matrix_def(v,delt,N);
Q = 1/rr*D.'*D + H.'*H;
Au=[eye(N); -eye(N)]; %2N,N
Ay=[eye(N); -eye(N)];
bu=umax*ones(2*N,1);
Ay1=Ay*H; %2N,N 
by=[ymax*ones(N,1); -ymin*ones(N,1)]; % 2N,1
x_=0; %initial x position
At=[ Au; Ay1;];

%%
%차량 그래픽을 위한 초기 꼭지점 좌표
x_point=[0,0-L,0-L,0]; %우상 좌상 좌하 우하 //초기 ego veicle 꼭지점 x좌표
y_point=[car_width/2 car_width/2 -car_width/2 -car_width/2] + [width/2 width/2 width/2 width/2]; % 초기 ego veicle 꼭지점 y좌표

x_front=[init_x,init_x-L,init_x-L,init_x]; %우상 좌상 좌하 우하 //front car
y_front=[car_width/2 car_width/2 -car_width/2 -car_width/2] + [width/2 width/2 width/2 width/2]+[init_y init_y init_y init_y]; % front car  꼭지점 y좌표

x_adj1=[init_x1,init_x1-L,init_x1-L,init_x1]; 
y_adj1=[car_width/2 car_width/2 -car_width/2 -car_width/2] + [width/2 width/2 width/2 width/2]+[init_y1 init_y1 init_y1 init_y1];

x_adj2=[init_x2,init_x2-L,init_x2-L,init_x2]; 
y_adj2=[car_width/2 car_width/2 -car_width/2 -car_width/2] + [width/2 width/2 width/2 width/2]+[init_y2 init_y2 init_y2 init_y2];

x=(x_point(1)+x_point(4))/2;
y=(y_point(1)+y_point(4))/2;

ing=false; %현재 추월중 아님 

th_result=zeros(1,100);
%% 그래프
figure('name','animation Test')
hold on
lane(d,width);  %lane 그리는 함수
[car_pos,car_point,car_laider,r]=ego_car(x_point,y_point); %ego car 그리기
front_car= car(x_front,y_front); %front car 그리기
 %라이다 원 그리기 위한것
a_=0:0.01:2*pi;

adj1_car=car(x_adj1,y_adj1);
adj2_car=car(x_adj2,y_adj2);

axis([0 d -d/2 d/2]) %전체에서 보기

%% 제어부
%way point
xd=x_point(1):1:d;
yd=ones(1,length(xd))*width/2;
Yd=width/2;
path=plot(xd,yd,'.');
%check obstacle variable
obs_chk=0;
x=[0; 3.5];
for i=1:length(0:delt:length(t))
    if i==1
        x_ini=x; %initial psi, y
        u_ini = 0;
    else
        x_ini=x(:,i-1);
        u_ini=U(1);
    end
    
    by1=by-Ay*G*x_ini-Ay*F*u_ini; % 2th column = input variable we need
    
    bt=[ bu; by1];
    
    if obs_chk == 0
        f = H.'*(G * x_ini + F * u_ini -Yd*ones(N,1));
    else
        f = H.'*(G * x_ini + F * u_ini -(Yd+width)*ones(N,1))*1;
    end
    
    U=quadprog(Q,f,At,bt);
    x(1,i+1)=x(1,i)+delt*U(1); %steering angle, psi
    x(2,i+1)=x(2,i)+delt*v*sin(x(1,i)); %y axis position
    x_(i+1)=x_(i)+delt*v*cos(x(1,i)); % x axis position
    
    
%for making enviroment car
if(v_front<front_lim)
    v_front=v_front+a_front*delt;
end
if(v_adj1<adj1_lim)
    v_adj1=v_adj1+a_adj1*delt;
end
if(v_adj2<adj2_lim)
    v_adj2=v_adj2+a_adj2*delt;
end

s_front=s_front+v_front*delt;
s_adj1=s_adj1+v_adj1*delt;
s_adj2=s_adj2+v_adj2*delt;

%to update rectangle position
x_point = [x_(i),x_(i)-L,x_(i)-L,x_(i)]+x_(i);
y_point = [car_width/2 car_width/2 -car_width/2 -car_width/2]+x(2,i);
x_front=[init_x,init_x-L,init_x-L,init_x]+s_front; %앞차 4개의 꼭지점 좌표 %현재 등속
y_front=[car_width/2 car_width/2 -car_width/2 -car_width/2] + [width/2 width/2 width/2 width/2]+[init_y init_y init_y init_y];
x_adj1=[init_x1,init_x1-L,init_x1-L,init_x1]+s_adj1;
y_adj1=[car_width/2 car_width/2 -car_width/2 -car_width/2] + [width/2 width/2 width/2 width/2]+[init_y1 init_y1 init_y1 init_y1];
x_adj2=[init_x2,init_x2-L,init_x2-L,init_x2]+s_adj2;
y_adj2=[car_width/2 car_width/2 -car_width/2 -car_width/2] + [width/2 width/2 width/2 width/2]+[init_y2 init_y2 init_y2 init_y2];
X=sum(x_point)/4; %ego 차 x 좌표 센터
Y=sum(y_point)/4; %ego 차 y 좌표 센터
car_dist=sum(x_front)/4-X; %ego 중심점과 front 중심점 거리
car_dist_adj1=sqrt((x_(i)-sum(x_adj1)/4)^2+(y-sum(y_adj1)/4)^2);
car_dist_adj2=sqrt((x_(i)-sum(x_adj2)/4)^2+(y-sum(y_adj2)/4)^2);

if(obs_chk==0 && car_dist <= r && car_dist>0)
    obs_chk=1;
end


%axis([x-r x+r -r +r]) % 따라가며 보기

%plot update
set(car_laider, 'Xdata', X+r*cos(a_),'Ydata', Y+r*sin(a_));
set(car_point, 'Xdata', X+0.1*cos(a_),'Ydata', Y+0.1*sin(a_));
set(car_pos, 'Xdata', x_point,'Ydata', y_point);
set(front_car, 'Xdata', x_front,'Ydata', y_front);
set(adj1_car, 'Xdata', x_adj1,'Ydata', y_adj1);
set(adj2_car, 'Xdata', x_adj2,'Ydata', y_adj2);
drawnow;
end

figure('name','heading angle')
x_=linspace(0,d,length(th_result));
plot(x_,th_result*180/pi)
axis([0 d -1 1])