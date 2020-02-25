h = animatedline('MaximumNumPoints',100);
axis([0,4*pi,-1,1])

x = linspace(0,4*pi,1000);
y = sin(x);

    addpoints(h,x(1:length(x)),y(1:length(y)));

