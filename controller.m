% M. Hiatt, D. Bourque
% control function using cubic polynominals
% input is a set of points (should be and next points) and actual point
% mcontrol is cubic 
function [updated_actual,o_ed,p_ed] = controller(desired,actual,o_ed,p_ed)
%trajectory = cscvn(points)
%t = 0:.1:1;


psi = 0;

g = 9.8;
Kxp = 8;
Kxd = .75;
%Kxdd = 1;
Kyp = 8;
Kyd = .75;
%Kydd = 1;
Kzp = 8;
Kzd = .75;
%Kzdd = 1;
Kphi_p = 1.85;
Ktheta_p = 1.85;
Kpsi_p = 1.85;
Kphi_d = .75;
Ktheta_d = .75;
Kpsi_d = .75;
I_x = .0045;
I_y = .0045;
I_z = .0088;
m = 0.468;
k = 2.980*10^(-6);
l = 0.225;
b = 1.140*10^(-7);
delta_t = .0001; % 10000 Hz simulation time
	
% PD
dx = Kxp*(desired(1) - actual(1)) + Kzd * (p_ed(1)); %+ Kxd*(desired(4) - actual(4)) + Kxdd*(desired(7) - actual(7));
dy = Kyp*(desired(2) - actual(2)) + Kzd * (p_ed(2)); %+ Kyd*(desired(5) - actual(5)) + Kydd*(desired(8) - actual(8));
dz = Kzp*(desired(3) - actual(3)) + Kzd * (p_ed(3)) %Kzd*(desired(6) - actual(6)) + Kzdd*(desired(9) - actual(9))

% new desired orientation and thrust
phi = asin((dx * sin(psi) - dy * cos(psi)) / (dx^2 + dy^2 + (dz + g)^2));
theta = atan((dx * cos(psi) - dy * sin(psi)) / (dz + g));
thrust = m * (dx * (sin(theta) *cos(psi) * cos(phi) + sin(psi) * sin(phi)) + dy*(sin(theta)*sin(psi) * cos(phi) - cos(psi) * sin(phi)) + (dz + g)*cos(theta)*cos(phi))
	
% accelerations on orientation
alpha_phi = (Kphi_p * (phi - actual(10)) + Kphi_d * (o_ed(1)));
alpha_theta = (Ktheta_p * (theta - actual(11)) + Ktheta_d * (o_ed(2)));
alpha_psi = (Kpsi_p * (psi - actual(12)) + Kpsi_d * (o_ed(3)));

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

%DON - integration
% actual is [x y z xd yd zd xdd ydd zdd phi theta psi phid thetad
% psid phidd thetadd psidd]

for i = 1 : 100
    % update accelerations
    actual(7) = (thrust / m)*(cos(actual(12))*sin(actual(11))*cos(actual(10))+sin(actual(12))*sin(actual(10)));
    actual(8) = (thrust / m)*(sin(actual(12))*sin(actual(11))*cos(actual(10))-cos(actual(12))*sin(actual(10)));
    actual(9) = (thrust / m)*(cos(actual(11))*cos(actual(10))) - g;
    actual(16) = alpha_phi;
    actual(17) = alpha_theta;
    actual(18) = alpha_psi;
    
    % update position and velocity
    actual(1) = actual(1) + actual(4)*delta_t + (1/2)*actual(7)*(delta_t^2);
    actual(2) = actual(2) + actual(5)*delta_t + (1/2)*actual(8)*(delta_t^2);
    actual(3) = actual(3) + actual(6)*delta_t + (1/2)*actual(9)*(delta_t^2);
    actual(4) = actual(4) + actual(7)*delta_t;
    actual(5) = actual(5) + actual(8)*delta_t;
    actual(6) = actual(6) + actual(9)*delta_t;

    %update angles and angular velocities
    actual(10) = actual(10) + actual(13)*delta_t + (1/2)*actual(16)*(delta_t^2);
    actual(11) = actual(11) + actual(14)*delta_t + (1/2)*actual(17)*(delta_t^2);
    actual(12) = actual(12) + actual(15)*delta_t + (1/2)*actual(18)*(delta_t^2);
    actual(13) = actual(13) + actual(16)*delta_t;
    actual(14) = actual(14) + actual(17)*delta_t;
    actual(15) = actual(15) + actual(18)*delta_t;

end

updated_actual = actual;
o_ed(1) = (phi - actual(10)) - o_ed(1);
o_ed(2) = (theta - actual(11)) - o_ed(2);
o_ed(3) = (psi - actual(12)) - o_ed(3);
p_ed(1) = (desired(1) - actual(1)) - p_ed(1);
p_ed(2) = (desired(2) - actual(2)) - p_ed(2);
p_ed(3) = (desired(3) - actual(3)) - p_ed(3);


end

