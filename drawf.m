%M. Hiatt
%This function takes in the position that will be plotted next (next)
%the x and y rotations (yr, xr) and an array of previous points.
%Both the next and prev arrays are 3*n for x, y and z coordinates.
function drawf(current, xr, yr, desired)
		
	blade = .4;  %blade length
	yrot = yr*(180/pi);
    xrot = xr*(180/pi);
	%the next x and y values of the quadcoptor
	cX = current(1);
	cY = current(2);
	cZ = current(3);
        
    invY = mod(yrot + 180,360);
    invX = mod(xrot + 180,360);
	%determines the verticle height of each blade
	cZ1u = (blade*sind(invY))+cZ; % blade 1
	cZ1l = (blade*sind(yrot))+cZ; % blade 3
	cZ2u = (blade*sind(invX))+cZ; % blade 2
	cZ2l = (blade*sind(xrot))+cZ; % blade 4
	
    %plots the line
    figure(1);
    clf;
    hold on;
    grid on;
    al = 370;
    ez = 50;
    view(al,ez);
        
    axis([-1 6 -1 6 -2 2]);
    title('Position Actual vs Desired')
    legend('Desired Pos.','Actual Pos.')
    xlabel('X position (m)')
    ylabel('Y Position (m)')
    zlabel('Z Position (m)')
    
	%for previous path store dots and plot 
	plot3([cX desired(1)], [cY desired(2)], [cZ desired(3)], 'LineStyle', '-', 'Color', 'r')
    plot3([cX desired(1)], [cY desired(2)], [-2 desired(3)-2], 'LineStyle', '-', 'Color', 'r')
    
    plot3([cX cX+blade], [cY cY], [cZ cZ1u], 'Color', 'b');
	plot3([cX cX], [cY cY-blade], [cZ cZ2u], 'Color', 'b');
	plot3([cX cX-blade], [cY cY], [cZ cZ1l], 'Color', 'b');
	plot3([cX cX], [cY cY+blade], [cZ cZ2l], 'Color', 'b');
    
    scatter3(cX,cY,-2,'filled');
	drawnow;
end
