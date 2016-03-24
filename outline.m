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
		trajectory = cscvn(points)
		t = 0:.1:1;
		% Thrust = 
		% PID = 
		traj = Thrust + PID;
		drawf(traj, actual);
	end

	%drawfunction
	function drawf(trajectory, actual)
		hold on;
		%for previous path store dots and plot 
		plot3D(line);
		plot3D(line);
		plot3D(circle);
		plot3D(circle);
		plot3D(circle);
		plot3D(circle);
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
