clear all
close all
clc
%%
%initial values

%road information
d=300; % 끝까지 거리
width = 3.5; %차선 너비

x=[0; 0; 1/2*width]; %initial psi ; x ; y ;


%simulation time
delt= 100e-4;
tf=40;
t=0:delt:tf;



rr=1; %weight value

N=10; % predicted step number


%information of cars
L=4.57; %m 단위 %car_length
car_width=1.8; %m 단위

s=0;
s_front=0;
s_adj1=0;
s_adj2=0;

v=5; % unit:m/s, velocity
ego_lim=10; %m/s
a=0;
v_front=1; front_lim=0; %m/s
a_front=0;
v_adj1=1; adj1_lim=0;
a_adj1=2;
v_adj2=1; adj2_lim=0;
a_adj2=1;

init_x=50; % 초기 ego vehicle x  좌표
init_y=0;
init_x1=90; %side lane front car initial position
init_y1= width;
init_x2=50; %side lane behind car initial position
init_y2=width;


% x_=0; %initial x position

%%
%차량 그래픽을 위한 초기 꼭지점 좌표
x_point=[x(2,1),x(2,1)-L,x(2,1)-L,x(2,1)]; %우상 좌상 좌하 우하 //초기 ego veicle 꼭지점 x좌표
y_point=[car_width/2 car_width/2 -car_width/2 -car_width/2] + x(3,1); % 초기 ego veicle 꼭지점 y좌표

x_front=[init_x,init_x-L,init_x-L,init_x]; %우상 좌상 좌하 우하 //front car
y_front=[car_width/2 car_width/2 -car_width/2 -car_width/2] + [width/2 width/2 width/2 width/2]+[init_y init_y init_y init_y]; % front car  꼭지점 y좌표

x_adj1=[init_x1,init_x1-L,init_x1-L,init_x1]; 
y_adj1=[car_width/2 car_width/2 -car_width/2 -car_width/2] + [width/2 width/2 width/2 width/2]+[init_y1 init_y1 init_y1 init_y1];

x_adj2=[init_x2,init_x2-L,init_x2-L,init_x2]; 
y_adj2=[car_width/2 car_width/2 -car_width/2 -car_width/2] + [width/2 width/2 width/2 width/2]+[init_y2 init_y2 init_y2 init_y2];

x_ego=(x_point(1)+x_point(4))/2;
y_ego=(y_point(1)+y_point(4))/2;

th_result=zeros(1,100);

%%
%constraints
umax = 5 *pi/180;  %unit : rad %maximum input
ymax =3/2 * width; %unit : m %upper lane value when there isnt a obstacle
ymin = 0; %unit : m %lower lane value when there isnt a obstacle
xmax = 9990;
xmin = 0;
%slope maximum, minimum constr
safe_dist=1;
%inclination
a1=1;
a2=a1;
a3=-a1; 
a4=a3; 

unit_m1=[-a1 1; a2 -1]; %unit matrix
As=[]; %slope constrain matrix

    
%for MPC constraints matrix
bu=umax*ones(2*N,1);

%matrix definition for mpc
[H,F,G,D,Q,O,B1,B2,Au,Ay,At,by0]=matrix_def(v,delt,N,rr);

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

%axis([0 d -d/2 d/2]) %전체에서 보기
axis([ceil(x(2,1)-r) ceil(x(2,1)+r) -r +r]) % 따라가며 보기 195행도 같이 바꿀것
%% 제어부
%way point
xd=x_point(1):d;
yd=ones(1,length(xd))*width/2;
Yd=1/2*width;%width;
path=plot(xd,yd,'.'); 
%check obstacle variable
obs_chk=0;
psi_=zeros(N,1);
x_=zeros(N,1);
y_=zeros(N,1);
input=0;
for i=1:length(0:delt:length(t))
    %changing way point
    Xd=100;%init_x+s_front+10;

    if i==1
        x_ini=x; %initial psi, y
        u_ini = 0;
    else
        x_ini=x(:,i-1);
        u_ini=U(1)*0;
        input(i)=U(1);
    end
    
    

    %for overtaking maneuver1,3 constrints
    % [bt,by,b1,b2,b3,b4]=constrain_update(xmax,ymax,xmin,ymin,safe_dist,init_x,init_x1,init_x2,width,by,Ay,G,x_ini,Ay,F,u_ini,bu);
    by=[by0*[xmax; ymax]; by0*[-xmin;-ymin]];
    b1= -a1*(safe_dist+init_x2+s_front)+3/2*width;
    b2= -a2*(-safe_dist+init_x+s_front-L)+1/2*width;
    b3=-a3*(-safe_dist+init_x1+s_adj1-L)+3/2*width;
    b4=-a4*(safe_dist+init_x+L)+1/2*width;
    by1=by-Ay*G*x_ini-Ay*F*u_ini; % 2th column = input variable we need
    %Ay : (4N,2N) G=(2N,3) x_ini=(3,1)


    bt=[ bu; by1];
   
    if obs_chk == 0
        f = H.'*(G * x_ini + F * u_ini + O * B2 - by0 * [Xd; Yd]);
    else
        f = H.'*(G * x_ini + F * u_ini + O * B2 - by0 * [Xd;(Yd+width)]);
    end
    
    U=quadprog(Q,f,At,bt);
    J=1/2*U'*Q*U+f.'*U
    
    x(1,i+1)=x(1,i)+delt*U(1); %steering angle, psi
    x(2,i+1)=x(2,i)+delt*v*cos(x(1,i)); % x axis position
    x(3,i+1)=x(3,i)+delt*v*sin(x(1,i)); %y axis position
    %Line=predictline(N,U,u_ini,x_ini,delt,v,i)
    % predictive line 생성%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    psi_(1) = delt*U(1)+x_ini(1,1);
    x_(1)=x_ini(2)+v*cos(psi_(1));
    y_(1)=x_ini(3)+sin(psi_(1));
    for j=2:N
        psi_(j)=delt*U(j)+U(j-1);
        x_(j)=x_(j-1)+v*cos(psi_(j-1));
        y_(j)=y_(j-1)+sin(psi_(j-1));
    end
    if i==1
        Line = line(x_,y_);
    else
        set(Line,'Xdata',x_,'Ydata',y_);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
    x_point = [x(2,i),x(2,i)-L,x(2,i)-L,x(2,i)];
    y_point = [car_width/2 car_width/2 -car_width/2 -car_width/2]+x(3,i);
    x_front=[init_x,init_x-L,init_x-L,init_x]+s_front; %앞차 4개의 꼭지점 좌표 %현재 등속
    y_front=[car_width/2 car_width/2 -car_width/2 -car_width/2] + [width/2 width/2 width/2 width/2]+[init_y init_y init_y init_y];
    x_adj1=[init_x1,init_x1-L,init_x1-L,init_x1]+s_adj1;
    y_adj1=[car_width/2 car_width/2 -car_width/2 -car_width/2] + [width/2 width/2 width/2 width/2]+[init_y1 init_y1 init_y1 init_y1];
    x_adj2=[init_x2,init_x2-L,init_x2-L,init_x2]+s_adj2;
    y_adj2=[car_width/2 car_width/2 -car_width/2 -car_width/2] + [width/2 width/2 width/2 width/2]+[init_y2 init_y2 init_y2 init_y2];
    X=sum(x_point)/4; %ego 차 x 좌표 센터
    Y=sum(y_point)/4; %ego 차 y 좌표 센터
    car_dist=sum(x_front)/4-X; %ego 중심점과 front 중심점 거리
    car_dist_adj1=sqrt((x(2,i)-sum(x_adj1)/4)^2+(x(3,i)-sum(y_adj1)/4)^2);
    car_dist_adj2=sqrt((x(2,i)-sum(x_adj2)/4)^2+(x(3,i)-sum(y_adj2)/4)^2);

    if(obs_chk==0 && car_dist <= r && car_dist>0)
            obs_chk=1;
    end


    axis([x(2,i)-2*r x(2,i)+2*r -r +r]) % 따라가며 보기 %112행도 같이 바꿀것

    %plot update
    set(car_laider, 'Xdata', X+r*cos(a_),'Ydata', Y+r*sin(a_));
    set(car_point, 'Xdata', X+0.1*cos(a_),'Ydata', Y+0.1*sin(a_));
    set(car_pos, 'Xdata', x_point,'Ydata', y_point);
    set(front_car, 'Xdata', x_front,'Ydata', y_front);
    set(adj1_car, 'Xdata', x_adj1,'Ydata', y_adj1);
    set(adj2_car, 'Xdata', x_adj2,'Ydata', y_adj2);
    
    drawnow;
end
subplot(3,1,1)
x_=linspace(0,d,length(x(1,:)));
plot(x_,x(1,:)*180/pi)
title('deg')
subplot(3,1,2)
x_=linspace(0,d,length(input));
plot(x_,input*180/pi);
title('deg/s')
subplot(3,1,3)
x_=linspace(0,d,length(x(3,:)));
plot(x_,x(3,:))
title('y (m))')