function lane(d,width)
x=0:0.1:d;
y=ones(1,length(x));
y=y*width;
plot(x,y,'--b','LineWidth',2)
% rectangle('Position',[0,width-0.1,d,0.2],'FaceColor',[1 1 0],'EdgeColor','y',...
%     'LineWidth',1)

rectangle('Position',[0,-0.1,d,0.2],'FaceColor',[1 1 0],'EdgeColor','y',...
    'LineWidth',1)

rectangle('Position',[0,2*width-0.1,d,0.2],'FaceColor',[1 1 0],'EdgeColor','y',...
    'LineWidth',1)


