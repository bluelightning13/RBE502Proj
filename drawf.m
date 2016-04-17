%draw function takes in the next position, the x and y rotation
	%and the previous position. 
	function drawf(next, yrot, xrot, prev)
		
		hold on;
		%for previous path store dots and plot 
		plot(prev, 'LineStyle', '-', 'Color', 'r')
		
		%the next x and y values of the quadcoptor
		cX = next(1);
		cY = next(2);
		
		%determines the verticle height of each blade
		cZ1u = blade*cos(xrot);
		cZ1l = -blade*cos(xrot);
		cZ2u = blade*sin(yrot);
		cZ2l = -blade*sin(yrot);
		%plots the line
		plot3([cX cX+blade], [cY cY+blade], [cZ cZ2u], 'Color', 'b');
		plot3([cX cX+blade], [cY cY-blade], [cZ cZ2u], 'Color', 'b');
		plot3([cX cX-blade], [cY cY+blade], [cZ cZ2l], 'Color', 'b');
		plot3([cX cX-blade], [cY cY-blade], [cZ cZ2l], 'Color', 'b');
		drawnow;
	end
