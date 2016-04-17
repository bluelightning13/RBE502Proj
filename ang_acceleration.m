% function for angular acceleration
	function angaccel = ang_acceleration(cInput, omega, I, L, b, k)
		tau = torques(cInput,L,b,k);
		angaccel = inv(I) * (tau - cross(omega, I*omega));
    end
    