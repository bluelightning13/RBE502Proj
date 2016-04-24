%M. Hiatt
%This function takes in the position that will be plotted next (next)
%the x and y rotations (yr, xr) and an array of previous points.
%Both the next and prev arrays are 3*n for x, y and z coordinates.
function drawf(next, yr, xr, prev)
		
	blade = 5;
	yrot = yr*(180/pi);
        xrot = xr*(180/pi);
	%the next x and y values of the quadcoptor
	cX = next(1);
	cZ = next(2);
	cY = next(3);
        
        invY = mod(yrot + 180,360);
        invX = mod(yrot + 180,360);
	%determines the verticle height of each blade
	cZ1u = (blade*cosd(xrot)+cZ);
	cZ1l = (blade*cosd(invX))+cZ;
	cZ2u = (blade*sind(yrot))+cZ;
	cZ2l = (blade*sind(invY))+cZ;
	%plots the line
        figure(1);
        clf;
        hold on;
        grid on;
        al = -90;
        ez = 0;
        view(al,ez);
        axis([-20 20 -20 20 -20 20]);
	%for previous path store dots and plot 
	plot3(prev(:,1),prev(:,2),prev(:,3), 'LineStyle', '-', 'Color', 'r')
    plot3([cX cX+blade], [cY cY+blade], [cZ cZ2u], 'Color', 'b');
	plot3([cX cX+blade], [cY cY-blade], [cZ cZ2u], 'Color', 'b');
	plot3([cX cX-blade], [cY cY+blade], [cZ cZ2l], 'Color', 'b');
	plot3([cX cX-blade], [cY cY-blade], [cZ cZ2l], 'Color', 'b');
	drawnow;
end
