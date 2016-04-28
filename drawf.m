%M. Hiatt
%This function takes in the position that will be plotted next (next)
%the x and y rotations (yr, xr) and an array of previous points.
%Both the next and prev arrays are 3*n for x, y and z coordinates.
function drawf(current, xr, yr, desired)
%
%

delta_t = 0.01;
tf = 2.5;


% x1 = [];
% y1 = [];
% z1 = [];
% x2 = [];
% y2 = [];
% z2 = [];
% x3 = [];
% y3 = [];
% z3 = [];
% x4 = [];
% y4 = [];
% z4 = [];
% x5 = [];
% y5 = [];
% z5 = [];
% x6 = [];
% y6 = [];
% z6 = [];
% 
% 
% 
%     % trajectory coefficients
%     
%     for i = 0 : delta_t : tf
%     position_d_coeffs_x_1 = transpose(computeCubicParameters(0,tf,0,0,2.5,1.5));
%     desired(1) = position_d_coeffs_x_1(1) + position_d_coeffs_x_1(2)*i + position_d_coeffs_x_1(3)*(i^2) + position_d_coeffs_x_1(4)*(i^3);
%     
%     x1 = [x1,desired(1)];
%     y1 = [y1,0];
%     z1 = [z1,0];
%     end
%     
%     for i = delta_t : delta_t : tf
%     position_d_coeffs_x_2 = transpose(computeCubicParameters(0,tf,2.5,1.5,5,0)); % 5 units in 10 seconds
%     position_d_coeffs_y_2 = transpose(computeCubicParameters(0,tf,0,0,2.5,1.5)); % 5 units in 10 seconds
%     desired(1) = position_d_coeffs_x_2(1) + position_d_coeffs_x_2(2)*i + position_d_coeffs_x_2(3)*(i^2) + position_d_coeffs_x_2(4)*(i^3);
%     desired(2) = position_d_coeffs_y_2(1) + position_d_coeffs_y_2(2)*i + position_d_coeffs_y_2(3)*(i^2) + position_d_coeffs_y_2(4)*(i^3);
%     
%     x2 = [x2,desired(1)];
%     y2 = [y2,desired(2)];
%     z2 = [z2,0];
%     end
%     
%     for i = delta_t : delta_t : tf
%     position_d_coeffs_x_3 = transpose(computeCubicParameters(0,tf,5,0,2.5,-1.5)); % 5 units in 10 seconds
%     position_d_coeffs_y_3 = transpose(computeCubicParameters(0,tf,2.5,1.5,5,0)); % 5 units in 10 seconds
%     desired(1) = position_d_coeffs_x_3(1) + position_d_coeffs_x_3(2)*i + position_d_coeffs_x_3(3)*(i^2) + position_d_coeffs_x_3(4)*(i^3);
%     desired(2) = position_d_coeffs_y_3(1) + position_d_coeffs_y_3(2)*i + position_d_coeffs_y_3(3)*(i^2) + position_d_coeffs_y_3(4)*(i^3);
%         
%     x3 = [x3,desired(1)];
%     y3 = [y3,desired(2)];
%     z3 = [z3,0];
%     end
%     
%     for i = delta_t : delta_t : tf
%     position_d_coeffs_x_4 = transpose(computeCubicParameters(0,tf,2.5,-1.5,0,0)); % 5 units in 10 seconds
%     position_d_coeffs_y_4 = transpose(computeCubicParameters(0,tf,5,0,2.5,-1.5)); % 5 units in 10 seconds
%     desired(1) = position_d_coeffs_x_4(1) + position_d_coeffs_x_4(2)*i + position_d_coeffs_x_4(3)*(i^2) + position_d_coeffs_x_4(4)*(i^3);
%     desired(2) = position_d_coeffs_y_4(1) + position_d_coeffs_y_4(2)*i + position_d_coeffs_y_4(3)*(i^2) + position_d_coeffs_y_4(4)*(i^3);
%     
%     x4 = [x4,desired(1)];
%     y4 = [y4,desired(2)];
%     z4 = [z4,0];
%     end
%     
%     for i = delta_t : delta_t : tf
%     position_d_coeffs_x_5 = transpose(computeCubicParameters(0,tf,0,0,2.5,1.5)); % 5 units in 10 seconds
%     position_d_coeffs_y_5 = transpose(computeCubicParameters(0,tf,2.5,-1.5,0,0)); % 5 units in 10 seconds
%     desired(1) = position_d_coeffs_x_5(1) + position_d_coeffs_x_5(2)*i + position_d_coeffs_x_5(3)*(i^2) + position_d_coeffs_x_5(4)*(i^3);
%     desired(2) = position_d_coeffs_y_5(1) + position_d_coeffs_y_5(2)*i + position_d_coeffs_y_5(3)*(i^2) + position_d_coeffs_y_5(4)*(i^3);
%     
%     x5 = [x5,desired(1)];
%     y5 = [y5,desired(2)];
%     z5 = [z5,0];
%     end
%     
%     for i = delta_t : delta_t : tf
%     position_d_coeffs_x_6 = transpose(computeCubicParameters(0,tf,2.5,1.5,5,0)); % 5 units in 10 seconds    
%     desired(1) = position_d_coeffs_x_6(1) + position_d_coeffs_x_6(2)*i + position_d_coeffs_x_6(3)*(i^2) + position_d_coeffs_x_6(4)*(i^3);
% 
%     x6 = [x6,desired(1)];
%     y6 = [y6,0];
%     z6 = [z6,0];
%     end
    
    %
%
%
    
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

    
%     time1 = linspace(0,2.5,2.5/0.01);
%     time2 = linspace(2.5,5,2.5/0.01);
%     time3 = linspace(5,7.5,2.5/0.01);
%     time4 = linspace(7.5,10,2.5/0.01);
%     time5 = linspace(10,12.5,2.5/0.01);
%     time6 = linspace(12.5,15,2.5/0.01);
    
    
    %plots the line
    figure(1);
    clf;
    hold on;
    grid on;
    al = 370;
    ez = 50;
    view(al,ez);
        
    axis([-1 6 -1 6 -2 2]);
    
%     plot3(position_d_coeffs_x_1,zeros(1,length(position_d_coeffs_x_1)),zeros(1,length(position_d_coeffs_x_1)),'k')
%     plot3(position_d_coeffs_x_2,position_d_coeffs_y_2,zeros(1,length(position_d_coeffs_x_2)),'k')
%     plot3(position_d_coeffs_x_3,position_d_coeffs_y_3,zeros(1,length(position_d_coeffs_x_3)),'k')
%     plot3(position_d_coeffs_x_4,position_d_coeffs_y_4,zeros(1,length(position_d_coeffs_x_4)),'k')
%     plot3(position_d_coeffs_x_5,position_d_coeffs_y_5,zeros(1,length(position_d_coeffs_x_5)),'k')
%     plot3(position_d_coeffs_x_6,zeros(1,length(position_d_coeffs_x_6)),zeros(1,length(position_d_coeffs_x_6)),'k')
%     
    
    title('Position Actual vs Desired')
    legend('Desired Pos.','Actual Pos.')
    xlabel('X position (m)')
    ylabel('Y Position (m)')
    zlabel('Z Position (m)')
    
	%for previous path store dots and plot 
	%plot3([cX desired(1)], [cY desired(2)], [cZ desired(3)], 'LineStyle', '-', 'Color', 'r')
    %plot3([cX desired(1)], [cY desired(2)], [-2 desired(3)-2], 'LineStyle', '-', 'Color', 'r')
    
    plot3([cX cX+blade], [cY cY], [cZ cZ1u], 'Color', 'b');
	plot3([cX cX], [cY cY-blade], [cZ cZ2u], 'Color', 'b');
	plot3([cX cX-blade], [cY cY], [cZ cZ1l], 'Color', 'b');
	plot3([cX cX], [cY cY+blade], [cZ cZ2l], 'Color', 'b');
    
    % shadow point
    scatter3(cX,cY,-2,'filled');
    
%     plot3(x1,y1,z1,'k')
%     plot3(x2,y2,z2,'k')
%     plot3(x3,y3,z3,'k')
%     plot3(x4,y4,z4,'k')
%     plot3(x5,y5,z5,'k')
%     plot3(x6,y6,z6,'k')
    
	drawnow;
end
