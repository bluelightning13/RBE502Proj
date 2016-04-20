% main function where everything runs
function outline()
    %main function 
    points=[0 1 1; 2 3 2; 1 5 8];
    
    %makes cubic pts
    fnplt(cscvn(points))
    hold on;

    
	h = findobj(gca,'Type','line');
    x=get(h,'Xdata');
    y=get(h,'Ydata');
    z=get(h,'Zdata');
    X = x';
    Y = y';
    Z = z';
    plot3(X(:,1),Y(:,1),Z(:,1));
    % initialize quadcopter state variables
    actual = zeros(1,18);
	
    
end
