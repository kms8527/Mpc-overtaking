function [output]=isovertaking(v,v_front,car_dist_adj1,car_dist_adj2,r)%,a_front)
%v�� ego veihcle �ӵ�
%v_front �� ���� �ӵ�
%a_fornt �� ���� ���ӵ�
% �߿����� ������ �Ǵ��ϴ� �Լ�


if (v>v_front &&car_dist_adj1>r)
    output=true;
else
    output=false;
end