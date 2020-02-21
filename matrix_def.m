function [H_,F_,G_,D_]=matrix_def(v,delt,N)
G = [ ] ; 
H = 0;
F = [ ]; 
D = eye(N) ; 
A = [ 1 0  ; v*delt 1 ] ; 
B = [ delt ; v*delt^2/2 ] ; 
C = [ 0 1 ] ;
 
Au=[eye(N); -eye(N)]; %2N,N
Ay=[eye(N); -eye(N)];


G = [ ] ; 
H = 0;
F = [ ]; 
D = eye(N) ; 

% H, G, F, D matrix computation
for i = 1 : N
    G = [ G ; C*A^i ] ;
    F = [ F ; C*A^(i-1)*B ] ; 
    if i ~=1 
        final_row =[ ] ;
        for j = i :-1: 2
            final_row  = [final_row C*A^(j-2)*B ] ; 
        end
        H = [ H zeros(i-1,1) ; final_row 0] ;
    end
    if i ~=N
        D(i,i+1) = -1 ; 
    elseif i ==N
        D(i,i) = 0 ;
    end
end

Ay1=Ay*H; %2N,N



H_=H;
F_=F;
G_=G;
D_=D;