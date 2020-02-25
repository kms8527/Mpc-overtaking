function [H_,F_,G_,D_,Q,O_,B1_,B2_,Au_,Ay_,At_,by_]=matrix_def(v,delt,N,rr)
G = [ ] ; 
H = [0; 0];
F = [ ]; 
D = eye(N) ; 
A = [ 1 0 0 ; 0 1 0; v*delt 0 1 ] ; 
B1 = [ 1; 0 ; 0 ] ;
B2 = [ 0; v*delt; 0];
C = [ 0 1 0; 0 0 1 ] ; 
O=[];
by0 = eye(2); %xmax ymax xim ymin 으로 이뤄진 b를 만들기 위해
% H, G, F, D matrix computation
for i = 1 : N
    O_row = zeros(2,3);
    
    for j=1:i
         O_row=O_row+C*A^(j-1);
    end
  
    O = [ O ;  O_row];
    G = [ G ; C*A^i ] ;
    F = [ F ; C*A^(i-1)*B1 ] ;
    if i ~=1 
        final_row =[ ] ;
        for j = i :-1: 2
            final_row  = [final_row C*A^(j-2)*B1 ] ;  %C : 2 3 A = 3 3 B1 = 3 1 -> 2 1
        end
        H = [ H zeros(2*(i-1),1) ; final_row [0; 0]] ;
        
        by0=[by0; eye(2)];
    end
    if i ~=N
        D(i,i+1) = -1 ; 
    elseif i ==N
        D(i,i) = 0 ;
    end
end

Q = 1/rr*(D.'*D) + H.'*H;
Au=[eye(N); -eye(N)]; %2N,N
Ay=[eye(2*N); -eye(2*N)];
Ay1=Ay*H; %2N,N 
At=[ Au; Ay1;];



H_=H;
F_=F;
G_=G;
D_=D;
O_=O;
B1_=B1;
B2_=B2;
Au_=Au;
Ay_=Ay;
At_=At;
by_=by0;