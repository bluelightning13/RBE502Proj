
	% control function using cubic polynominals
	% input is a set of points (should be and next points) and actual point
	% mcontrol is cubic 
	function mControl = controller(points, actual) %Meagan
	%trajectory = cscvn(points)
		%t = 0:.1:1;
		if ~exist(phi_d)
			phi_d = 0;
			phi = 0;
            i = 0;%for trajectory points
            prevPts = 0;
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
        k = 2.980*10^(-6);
        l = 0.225;
        b = 1.140*10^(-7);
        delta_t = .0001; 
		
        
        % PD
		dx = Kxp*(desired(1) - actual(1)) + Kxd(desired(4) - actual(4)) + Kxdd(desired(7) - actual(7));
		dy = Kyp*(desired(2) - actual(2)) + Kyd(desired(5) - actual(5)) + Kydd(desired(8) - actual(8));
		dz = Kzp*(desired(3) - actual(3)) + Kzd(desired(6) - actual(6)) + Kzdd(desired(9) - actual(9));
		
		% new desired orientation and thrust
		phi = arcsin((dx * sin(psi) - dy * cos(psi)) / (dx^2 + dy^2 + (dz + g)^2));
		theta = arctan((dx * cos(psi) - dy * sin(psi)) / (dz + g));
		thrust = m * (dx * (sin(theta) *cos(psi) * cos(phi) + sin(psi) * sin(phi)) + dy*(sin(theta)*sin(psi) * cos(phi) - cos(psi) * sin(phi)) + (dz + g)*cos(theta)*cos(phi));
		
        % accelerations on orientation
        alpha_phi = (Kphi_p * (phi - actual(10)) + Kphi_d(phi_d - actual(10)));
        alpha_theta = (Ktheta_p * (theta - actual(10)) + Ktheta_d(theta_d - actual(10)));
        alpha_psi = (Kpsi_p * (psi - actual(10)) + Kpsi_d(psi_d - actual(10)));
        
        % desired torque on body
		t_phi = alpha_phi * I_x;
		t_theta = alpha_theta * I_y;
		t_psi = alpha_psi * I_z;      
        
        % new motor speeds to produce desired torques and thrust
        % this assumes instantaneous change to desired motor speeds - will
        % want a PID controller for this 
        w12 = (thrust / 4*k) - (t_theta / 2*k*l) - (t_psi / 4 * b);
        w22 = (thrust / 4*k) - (t_phi / 2*k*l) + (t_psi / 4 * b);
        w32 = (thrust / 4*k) + (t_theta / 2*k*l) - (t_psi / 4 * b);
        w42 = (thrust / 4*k) + (t_phi / 2*k*l) + (t_psi / 4 * b);
        
		phi_d = phi;
		theta_d = theta;
		psi_d = psi;
        
        %DON - integration
        % actual is [x y z xd yd zd xdd ydd zdd phi theta psi phid thetad
        % psid phidd thetadd psidd]
        
        for i = 1 : 100
            actual(1) = actual(1) + actual(4)*delta_t + (1/2)*actual(7)*(delta_t^2);
            actual(2) = actual(2) + actual(5)*delta_t + (1/2)*actual(8)*(delta_t^2);
            actual(3) = actual(3) + actual(6)*delta_t + (1/2)*actual(9)*(delta_t^2);
            actual(4) = actual(4) + actual(7)*delta_t;
            actual(5) = actual(5) + actual(8)*delta_t;
            actual(6) = actual(6) + actual(9)*delta_t;
            actual(7) = (thrust / m)*(cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi));
            actual(8) = (thrust / m)*;
            actual(9) = (thrust / m)*;
            actual(10) = actual(10) + actual(13)*delta_t + (1/2)*actual(16)*(delta_t^2);
            actual(11) = actual(11) + actual(14)*delta_t + (1/2)*actual(17)*(delta_t^2);
            actual(12) = actual(12) + actual(15)*delta_t + (1/2)*actual(18)*(delta_t^2);
            actual(13) = actual(13) + actual(16)*delta_t;
            actual(14) = actual(14) + actual(17)*delta_t;
            actual(15) = actual(15) + actual(18)*delta_t;
            actual(16) = alpha_phi;
            actual(17) = alpha_theta;
            actual(18) = alpha_;
        end
        
        
		drawf(traj, xrot, yrot, prevPts); %draw where we are now and where we should be
        prevPts(i) = actual;
        
        %implement disturbance
        [X Y Z] = disturbance();
        
        
        
        
        
	end