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