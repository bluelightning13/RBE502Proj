% main function where everything runs
function main()



	% function to display
	function [x] = display()

	end 


	% function for linear acceleration
	function accel = acceleration(cInput, angles, xd, m, k, kd)
		grav = [0; 0; -9.8];
		fd = -kd * xd;
		rx = [1 0 0 ; 0 cos() -sin(); 0 sin() cos()];
		ry = [cos() 0 sin(); 0 1 0; -sin() 0 cos()];
		rz = [cos() -sin() 0; sin() cos() 0; 0 0 1];
		R = [rx ; ry ; rz ];
		th = R * thrust(cInput, k);
		accel = grav + 1/m *th + fd;
	end
	
	% function to calculate thrust
	function thf = thrust(cInput, k)
		thf = [0; 0; k * sum(cInput)];
	end

	
	% function for angular acceleration
	function angaccel = ang_acceleration(cInput, omega, I, L, b, k)
		tau = torques(cInput,L,b,k);
		angaccel = inv(I) * (tau - cross(omega, I*omega));
	end

	% compute the torques
	function tau = torques()
		tau = [L*k*(cInput(1) - cInput(3))  L*k*(cInput(2) - cInput(4))  b*(cInput(1) - cInput(2) + cInput(3) - cInput(4)) ];
	end


	% control function using cubic polynominals
	% input is a set of points (should be and next points) and actual point
	% mcontrol is cubic 
	function mControl = controller(points, actual) %Meagan
	%trajectory = cscvn(points)
		%t = 0:.1:1;
		if ~exist(phi_d)
			phi_d = 0;
			phi = 0;
		else
		if ~exist(theta_d)
			theta_d = 0;
			theta = 0;
		else
		if ~exist(psi_d)
			psi_d = 0; %heading
			psi = 0;
		else
		Kxp = 1.85;
		Kxd = .75;
		Kxdd = 1;
		Kyp = 8.55;
		Kyd = .75;
		Kydd = 1;
		Kzp = 1.85;
		Kzd = .75;
		Kzdd = 1;
		Kphi_p = 3;
		Ktheta_p = 3;
		Kpsi_p = 3;
		Kphi_d = .75;
		Ktheta_d = .75;
		Kpsi_d = .75;
		I_x = .0045;
		I_y = .0045;
		I_z = .0088;
		
		dx = Kxp*(desired(1) - actual(1)) + Kxd(desired(4) - actual(4)) + Kxdd(desired(7) - actual(7));
		dy = Kyp*(desired(2) - actual(2)) + Kyd(desired(5) - actual(5)) + Kydd(desired(8) - actual(8));
		dz = Kzp*(desired(3) - actual(3)) + Kzd(desired(6) - actual(6)) + Kzdd(desired(9) - actual(9));
		
		
		phi = arcsin((dx * sin(psi) - dy * cos(psi)) / (dx^2 + dy^2 + (dz + g)^2));
		theta = arctan((dx * cos(psi) - dy * sin(psi)) / (dz + g));
		thrust = m * (dx * (sin(theta) *cos(psi) * cos(phi) + sin(psi) * sin(phi)) + dy*(sin(theta)*sin(psi) * cos(phi) - cos(psi) * sin(phi)) + (dz + g)*cos(theta)*cos(phi));
		
		t_phi = (Kphi_p * (phi - actual(10)) + Kphi_d(phi_d - actual(10))) * I_x;
		t_theta = (Ktheta_p * (theta - actual(10)) + Ktheta_d(theta_d - actual(10))) * I_y;
		t_psi = (Kpsi_p * (psi - actual(10)) + Kpsi_d(psi_d - actual(10))) * I_z;
		
		
		phi_d = phi;
		theta_d = theta;
		psi_d = psi;
		
		
        
		drawf(traj, actual); %draw where we are now and where we should be
	end

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

	% control function using PID - for when the QC is stationary(hovering)
	function sControl = pid_controller() %John

	end
	
	% wind / disturbance function
	% same F vector with varyine magnitude
	% NOT MOMENT INDUCING
	function randomF = disturbance() %Don
	
	end
	
	

end
