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


	% function for angular acceleration
	function angaccel = ang_acceleration()

	end


	% function to calculate thrust
	function thf = thrust(cInput, k)
		thf = [0; 0; k * sum(cInput)];
	end


	% compute the torques
	function tau = torques()

	end


	% control function using PD
	function control = pd_controller()

	end

	% wind / disturbance function
	% same F vector with varyine magnitude
	% NOT MOMENT INDUCING
	function randomF = disturbance()
	
	end

end
