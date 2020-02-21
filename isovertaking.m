function [output]=isovertaking(v,v_front,car_dist_adj1,car_dist_adj2,r)%,a_front)
%v는 ego veihcle 속도
%v_front 는 앞차 속도
%a_fornt 는 앞차 가속도
% 추월할지 말지를 판단하는 함수


if (v>v_front &&car_dist_adj1>r)
    output=true;
else
    output=false;
end