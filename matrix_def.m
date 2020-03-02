function [A_,H_,F_,G_,D_,Q,O_,B1_,Au_,Ay_,At_,bu0_,by0_]=matrix_def(delt,N,rr)
G = [ ] ; 
H = [zeros(3,2)];
F = [ ]; 
D = eye(2*N) ; 
A = [0 0 0 0 ; 0 0 0 1; 0 0 0 0; 0 0 0 0 ] ; 
A = A*delt+eye(4);
B1 = [ 0 1; 0 0 ; 0 0; 1 0 ] *delt;
C = [0 1 0 0;0 0 1 0;0 0 0 1] ; 
O=[];
by0 = eye(3); %psi'max xmax ymax vmax / -psi'min -xim -ymin -vmin 으로 이뤄진  8x1 b를 만들기 위해 인수에 4넣음
bu0 = eye(2); %amax psi''max / amax psi''max
% H, G, F, D matrix computation
for i = 1 : N

    G = [ G ; C*A^i ] ;
    F = [ F ; C*A^(i-1)*B1 ] ;
    if i ~=1 
        final_row =[ ] ;
        for j = i :-1: 2
            final_row  = [final_row C*A^(j-2)*B1 ] ;  %C : 3 4 A = 4 4 B1 = 4 2 -> 3 2
        end
        H = [ H zeros(3*(i-1),2) ; final_row zeros(3,2)] ;
        
        by0 = [by0; eye(3)];
        bu0 = [bu0; eye(2)];
    end
    if i < N
        D(2*i-1,(2*i-1)+2) = -1 ;
        D(2*i,2*i+2)=-1;
    else
        D(2*i-1,2*i-1) = 0 ;
        D(2*i,2*i) = 0;
    end
end
Q = 1/rr*(D.'*D) + H.'*H;
Au=[eye(2*N); -eye(2*N)]; %4N,2N
Ay=[eye(3*N); -eye(3*N)]; % 6N, 3N
Ay1=Ay*H; %4Nx4M * 4Nx2N = 4Nx2N
At=[ Au; Ay1;];


A_=A;
H_=H;
F_=F;
G_=G;
D_=D;
O_=O;
B1_=B1;
Au_=Au;
Ay_=Ay;
At_=At;
bu0_=bu0;
by0_=by0;