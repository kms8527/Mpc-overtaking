clear all
close all
clc
%%
%initial values

%road information
d=300; % 끝까지 거리
width = 3.5; %차선 너비

x=[0; 0; 1/2*width; 0]; %initial psi' ; x ; y ; v;

% desired waypoint

Yd=width;%300%1/2*width;%width;
vd=3;
%simulation time
delt= 200e-3;
tf=80;
t=0:delt:tf;

rr=1/2; %weight value

N=40; % predicted step number


%information of cars
L=4.57; %m 단위 %car_length
car_width=1.8; %m 단위

s=0;
s_front=0;
s_adj1=0;
s_adj2=0;

ego_lim=10; %m/s
v_front=1; front_lim=0; %m/s
a_front=0;
v_adj1=1; adj1_lim=0;
a_adj1=2;
v_adj2=1; adj2_lim=0;
a_adj2=1;

init_x=50; % 초기 front vehicle x  좌표
init_y=0;
init_x1=90; %side lane front car initial position
init_y1= width;
init_x2=50; %side lane behind car initial position
init_y2=width;

    %matrix definition for mpc
[A,H,F,G,D,Q,O,B1,Au,Ay,At,bu0,by0]=matrix_def(delt,N,rr);

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

%%
%input constraints
pmax = 5 *pi/180;  %unit : rad/s^2 % maximum psi
amax = 1.5;
%output constraints
xmax = 350;
xmin = 0;
ymax =2 * width; %unit : m %upper lane value when there isnt a obstacle
ymin = 0; %unit : m %lower lane value when there isnt a obstacle
vmax = 50;


%slope maximum, minimum constr
safe_dist=1;
%inclination
a1=1;
a2=a1;
a3=-a1; 
a4=a3; 

unit_m1=[-a1 1; a2 -1]; %unit matrix
As=[]; %slope constrain matrix


% %matrix definition for mpc
% [A,H,F,G,D,Q,O,B1,Au,Ay,At,bu0,by0]=matrix_def(delt,N,rr,psi_);
%for MPC input constraints matrix
bu=[bu0*[amax;pmax]; bu0*[amax;pmax]]; %a , psi''  maximum input constraint


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
psi_=zeros(N,1);
path=plot(xd,yd,'.');
%check obstacle variable
obs_chk=0;


for i=1:length(t)
    if i==1
        x_ini=x; %initial psi, y
        u_ini = [0; 0];
        p_ini=0;
        input1=u_ini(1);
        input2=u_ini(2);
    else
        x_ini=x(:,i);
        u_ini=[U(1); U(2)];
        p_ini=psi_(i);
        input1(i)=U(1);
        input2(i)=U(2);
    end
    Xd=50;%init_x+s_front+10;

        %changing way point


    %for overtaking maneuver1,3 constrints
    by=[by0*[xmax; ymax; vmax]; by0*[-xmin;-ymin; vmax]];
    b1= -a1*(safe_dist+init_x2+s_front)+3/2*width;
    b2= -a2*(-safe_dist+init_x+s_front-L)+1/2*width;
    b3=-a3*(-safe_dist+init_x1+s_adj1-L)+3/2*width;
    b4=-a4*(safe_dist+init_x+L)+1/2*width;
    by1=by-Ay*G*x_ini-Ay*F*u_ini; % 2th column = input variable we need
    %Ay : (4N,2N) G=(2N,4) x_ini=(4,1)


    bt=[ bu; by1];
   
    if obs_chk == 0
        f = H.'*(G * x_ini + F * u_ini - by0 * [ Xd; Yd; vd]);
    else
        f = H.'*(G * x_ini + F * u_ini - by0 * [ Xd; Yd+width; vd]);
    end
    if(i == 21)
        asd = 8;
    end
    U=quadprog(Q,f,At,bt);
    J=(1/2*U'*Q*U+f.'*U)

    psi_(i+1)=p_ini+delt*x_ini(1);
    x(1,i+1)=x_ini(1)+delt*U(2) ; %derivative of steering angle, psi'
    x(2,i+1)=x_ini(2)+delt*x_ini(4)*cos(psi_(i)); % x axis position
    x(3,i+1)=x_ini(3)+delt*x_ini(4)*sin(psi_(i)); % y axis position
    x(4,i+1)=x_ini(4)+delt*U(1); % v
    %Line=predictline(N,U,u_ini,x_ini,delt,v,i)
    % predictive line 생성%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    psi_p=psi_(i+1);
    x_p(1,1)=x(1,i+1);
    x_p(2,1)=x(2,i+1);
    x_p(3,1)=x(3,i+1);
    x_p(4,1)=x(4,i+1);
    for j=2:N
       psi_p(j)=psi_p(j-1)+delt*x_p(1,j-1);
       x_p(1,j)=x_p(1,j-1)+delt*U(2*j);
       x_p(2,j)=x_p(2,j-1)+delt*x_p(4,j-1)*cos(psi_p(j-1));
       x_p(3,j)=x_p(3,j-1)+delt*x_p(4,j-1)*sin(psi_p(j-1));
       x_p(4,j)=x_p(4,j-1)+delt*U(2*j-1);
    end
    if i==1
        Line = plot(x_p(2,:),x_p(3,:),'b.');
    else
        set(Line,'Xdata',x_p(2,:),'Ydata',x_p(3,:));
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
    x_point = [x_ini(2),x_ini(2)-L,x_ini(2)-L,x_ini(2)];
    y_point = [car_width/2 car_width/2 -car_width/2 -car_width/2]+x_ini(3);
    x_front=[init_x,init_x-L,init_x-L,init_x]+s_front; %앞차 4개의 꼭지점 좌표 %현재 등속
    y_front=[car_width/2 car_width/2 -car_width/2 -car_width/2] + [width/2 width/2 width/2 width/2]+[init_y init_y init_y init_y];
    x_adj1=[init_x1,init_x1-L,init_x1-L,init_x1]+s_adj1;
    y_adj1=[car_width/2 car_width/2 -car_width/2 -car_width/2] + [width/2 width/2 width/2 width/2]+[init_y1 init_y1 init_y1 init_y1];
    x_adj2=[init_x2,init_x2-L,init_x2-L,init_x2]+s_adj2;
    y_adj2=[car_width/2 car_width/2 -car_width/2 -car_width/2] + [width/2 width/2 width/2 width/2]+[init_y2 init_y2 init_y2 init_y2];
    X=sum(x_point)/4; %ego 차 x 좌표 센터
    Y=sum(y_point)/4; %ego 차 y 좌표 센터
    car_dist=sum(x_front)/4-X; %ego 중심점과 front 중심점 거리
    car_dist_adj1=sqrt((x_ini(2)-sum(x_adj1)/4)^2+(x_ini(3)-sum(y_adj1)/4)^2);
    car_dist_adj2=sqrt((x_ini(2)-sum(x_adj2)/4)^2+(x_ini(3)-sum(y_adj2)/4)^2);
    
    if(obs_chk==0 && car_dist <= r && car_dist>0)
            obs_chk=1;
    end
    if(obs_chk==1 && car_dist<0)
        obs_chk=1;
    end


    axis([x_ini(2)-r x_ini(2)+r -r +r]) % 따라가며 보기 %112행도 같이 바꿀것

    %plot update
    set(car_laider, 'Xdata', X+r*cos(a_),'Ydata', Y+r*sin(a_));
    %set(car_point, 'Xdata', X+0.1*cos(a_),'Ydata', Y+0.1*sin(a_));
    plot(x_ini(2)+0.1*cos(a_),x_ini(3)+0.1*sin(a_));
    set(car_pos, 'Xdata', x_point,'Ydata', y_point);
    set(front_car, 'Xdata', x_front,'Ydata', y_front);
    set(adj1_car, 'Xdata', x_adj1,'Ydata', y_adj1);
    set(adj2_car, 'Xdata', x_adj2,'Ydata', y_adj2);
    
    drawnow;
end
subplot(5,1,1)
x_=linspace(0,d,length(x(1,:)));
plot(x_,psi_(:)*180/pi)
title('deg')
subplot(5,1,2)
x_=linspace(0,d,length(x(1,:)));
plot(x_,x(1,:)*180/pi)
title('deg/s')
subplot(5,1,3)
x_=linspace(0,d,length(input2));
plot(x_,input2*180/pi);
title('deg/s^2')
subplot(5,1,4)
x_=linspace(0,d,length(x(3,:)));
plot(x_,x(4,:))
title('v (m/s))')
subplot(5,1,5)
x_=linspace(0,d,length(input1));
plot(x_,input1)
title('a (m/s^2))')