% main function where everything runs
function outline()
    %main function 
    points=[0 1 1; 2 3 2; 1 5 8];
    pts = spline(points(:,1),points(:,2),points(:,3));
    %pts2 = interp1();
    fnplt(cscvn(points))
    hold on;
    %plot(points(1,:),points(2,:),'o'), hold off
    %p2 = 3/8*pi;
    %p3 = -1/(16*pi^2);
    %ti = 0;
    %tf=5*pi; %seconds
    %dt = 0.016;
    %t = [ti:dt:tf];
    %gamma = polyval([p3 p2 0 0], t)
    
    
    
    %a = 0.0625;
    %b = 0.25;
    %x = (b-a)*sin(t) + 3*a*sin((t/a)*(b-a));
    %z = 0.5+(b-a)*cos(t) - 3*a*cos((t/a)*(b-a));
    %plot(gamma);
    
    
    %trajectory - cubic poly to/from two points
    
    %for every point
        %call controller
        %call disturbance
    for i = 0:102
       plot(X(i),Y(i),Z(i); 
    end
    
	h = findobj(gca,'Type','line');
    x=get(h,'Xdata');
    y=get(h,'Ydata');
    z=get(h,'Zdata');
    X = x';
    Y = y';
    Z = z';
    plot3(X(:,1),Y(:,1),Z(:,1));
	
	
	

end
