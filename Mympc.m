clear all
close all
clc
delt= 200e-3;
tf=40;
r=1; %weight value
v=20/3.6; % unit:kph, velocity
N=30; % predicted step number
umax = 3 *pi/180;  %unit : rad %maximum input
ymax = 10; %unit : m %upper lane value when there isnt a obstacle
ymin = 0; %unit : m %lower lane value when there isnt a obstacle
x_ini=[0; 1.5];
u_ini = 0;
Yd=1.5;

%kinicematic model
t=0:delt:tf;
A=[1 0; delt*v 1];  
B=[delt; 0];
G=[];
H=0;
F=[];
D=eye(N);
%interest variable
C=[0 1];

%%
%getting G,H,F,D process
for i = 1:N %i = current row
    G=[G; C*A^i];
    F=[F; C*A^(i-1)*B];
    if i~=1
        final_row=[];
        for j=i:-1:2
            final_row=[final_row C*A^(j-2)*B];
        end
            H=[H zeros(i-1,1); final_row 0];
    end
    if i~=N
            D(i,i+1)=-1;
        else
            D(i,i)=0;
    end
end
%%
%transform for making QP problem
Q=(H'*H)+1/r*(D'*D);

% constraints matrix
Au=[eye(N); -eye(N)]; %2N,N
Ay=[eye(N); -eye(N)];
bu=umax*ones(2*N,1);
Ay1=Ay*H; %2N,N
by=[ymax*ones(N,1); -ymin*ones(N,1)]; % 2N,1
x=x_ini'; %initial state 
x_=0; %initial x position
At=[ Au; Ay1;];
%obstacle variable
obs_chk = 0;
st_point = 45;
end_point = 55;


%when i=1
i = 1; %0번째에 대해서
by1=by-Ay*G*x_ini-Ay*F*u_ini; % 2th column = input variable we need
bt=[bu; by1 ];
f= H'*(G*x_ini+F*u_ini-Yd);
U=quadprog(Q,f,At,bt);
x(i+1,1)=x(i)+delt*U(i); %steering angle, psi
x(i+1,2)=x(i,2)+delt*v*sin(x(i,1)); %y axis position
x_(i+1)=x_(i)+delt*v*cos(x(i,1)); % x axis position

x_ini= [x(1,1); x(1,2)];
u_ini= x(1,1);
figure('name','시뮬레이션');
sim=plot(x_(1)+0.5*cos(0:0.001:2*pi),x_ini(2)+0.5*sin(0:0.001:2*pi),'b');
% sim_pre=plot([],[]);
hold on

line([st_point end_point end_point st_point st_point],[2.0 2.0 0 0 2.0],'color','r')
line([0 180],[6 6], 'color','k')
line([0 180],[3 3], 'color','k')
line([0 180],[0 0], 'color','k')
 axis([0 180 -10 10])



%when from i=1 to i=tf
for i = 2:length(t)
% for i=2:length(t)
    by1=by-Ay*G*x_ini-Ay*F*u_ini; % 2th column = input variable we need
    bt=[ bu; by1];
    %f= H'*(G*x_ini+F*u_ini-Yd);
    
    if obs_chk == 0
        f = H.'*(G * x_ini + F * u_ini -Yd*ones(N,1));
    else
        f = H.'*(G * x_ini + F * u_ini -(Yd+5)*ones(N,1))*1;
    end
    U=quadprog(Q,f,At,bt);
%     J=1/2*U'*Q*U+f.'*U
    x(i+1,1)=x(i,1)+delt*U(1); %steering angle, psi
    x(i+1,2)=x(i,2)+delt*v*sin(x(i,1)); %y axis position
    x_(i+1)=x_(i)+delt*v*cos(x(i,1)); % x axis position
        %////////////////////////
    p_pre=x(i,1);
    x_pre=x_(i);
    y_pre=x(i,2);
    
    for j=2:N+1
        p_pre(j)=+p_pre(j-1)+U(j-1)*delt;
        x_pre(j)=x_pre(j-1)+delt*v*cos(p_pre(j-1));
        y_pre(j)=y_pre(j-1)+delt*v*sin(p_pre(j-1));
    end
    plot(x_pre,y_pre,'.')
    %///////////////////////////////////////////
    x_ini= [x(i,1); x(i,2)];
    u_ini= U(1);
    
    

    if x_(i+1) + N * v* delt > st_point && x_(i+1) + N *v * delt <end_point
        obs_chk = 1;
    end
    if obs_chk == 1
        if x_(i+1) > end_point
            obs_chk = 0;
        end
    end
    set(sim,'Xdata',x_(i)+0.5*cos(0:0.001:2*pi),'Ydata',x(i,2)+0.5*sin(0:0.001:2*pi))
% bt_me=bt;
% Q_me=Q;
% f_me=f;
% At_me=At;
% plot(x_,x(:,2))
% H_me=H;
% G_me=G;
% x_ini_me=x_ini;
% F_me=F;
% u_ini_me=u_ini;
pause(0.1)
end
hold off

% subplot(2,1,1)
% plot(x_,x(:,2))
% hold on

%%

% 
% line([st_point end_point end_point st_point st_point],[2.0 2.0 0 0 2.0],'color','r')
% line([0 180],[6 6], 'color','k')
% line([0 180],[3 3], 'color','k')
% line([0 180],[0 0], 'color','k')
% axis([0 180 -10 10])
% title('path')
% ylabel('y(m)')
% xlabel('x(m)')
% hold off
% subplot(2,1,2)
% plot(x_,x(:,1)*180/pi)
% title('heading angle')
% ylabel('deg')
% xlabel('x(m)')
