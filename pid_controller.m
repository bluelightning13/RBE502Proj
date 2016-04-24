% M. Hiatt, D. Bourque, J. Kelly
% control function using cubic polynominals
% input is a set of points (should be and next points) and actual point
% mcontrol is cubic 
function [updated_actual,updated_phi_d,updated_theta_d,updated_psi_d] = pid_controller(desired,actual,phi_d,theta_d,psi_d)
%trajectory = cscvn(points)
%t = 0:.1:1;
if ~exist(phi_d)
    phi_d = 0;
    phi = 0;
i = 0;%for trajectory points
prevPts = 0;
elseif ~exist(theta_d)
    theta_d = 0;
    theta = 0;
elseif ~exist(psi_d)
    psi_d = 0; %heading
    psi = 0;
end

% Constants:
m = 0.468; %set mass to 0.468 kg
g = 9.8; %gravitational constant

% Proportional and derivative gain coefficients:
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

%Integral gain goefficients:
Kxi = 1;
Kyi = 1;
Kzi = 1;
Kphi_i = 1;
Ktheta_i = 1;
Kpsi_i = 1;

I_x = .0045;
I_y = .0045;
I_z = .0088;
k = 2.980*10^(-6);
l = 0.225;
b = 1.140*10^(-7);
delta_t = .0001; % in simulation
	
% PID
% actual is [x y z xd yd zd xdd ydd zdd phi theta psi phid thetad
% psid phidd thetadd psidd]

dx = Kxp*(points(1) - actual(1)) + Kxi*(points(1)-actual(1))*delta_t + Kxd*(points(4) - actual(4)) + Kxdd*(points(7) - actual(7));
dy = Kyp*(points(2) - actual(2)) + Kyi*(points(2)-actual(2))*delta_t + Kyd*(points(5) - actual(5)) + Kydd*(points(8) - actual(8));
dz = Kzp*(points(3) - actual(3)) + Kzi*(points(3)-actual(3))*delta_t + Kzd*(points(6) - actual(6)) + Kzdd*(points(9) - actual(9));

% new desired orientation and thrust
phi = asin((dx * sin(psi) - dy * cos(psi)) / (dx^2 + dy^2 + (dz + g)^2));
theta = atan((dx * cos(psi) - dy * sin(psi)) / (dz + g));
thrust = m * (dx * (sin(theta) *cos(psi) * cos(phi) + sin(psi) * sin(phi)) + dy*(sin(theta)*sin(psi) * cos(phi) - cos(psi) * sin(phi)) + (dz + g)*cos(theta)*cos(phi));
	
% accelerations on orientation
alpha_phi = Kphi_p*(phi - actual(10)) + Kphi_i*(phi - actual(10))*delta_t + Kphi_d*(phi_d - actual(10));
alpha_theta = Ktheta_p*(theta - actual(11)) + Ktheta_i*(theta - actual(11))*delta_t + Ktheta_d*(theta_d - actual(11));
alpha_psi = Kpsi_p*(psi - actual(12)) + Kpsi_i*(psi - actual(12))*delta_t + Kpsi_d*(psi_d - actual(12));

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
    % update accelerations
    actual(7) = (thrust / m)*(cos(actual(12))*sin(actual(11))*cos(actual(10))+sin(actual(12))*sin(actual(10)));
    actual(8) = (thrust / m)*(sin(actual(12))*sin(actual(11))*cos(actual(10))-cos(actual(12))*sin(actual(10)));
    actual(9) = (thrust / m)*(cos(actual(11))*cos(actual(10)));
    actual(16) = alpha_phi;
    actual(17) = alpha_theta;
    actual(18) = alpha_psi;

    actual(1) = actual(1) + actual(4)*delta_t + (1/2)*actual(7)*(delta_t^2);
    actual(2) = actual(2) + actual(5)*delta_t + (1/2)*actual(8)*(delta_t^2);
    actual(3) = actual(3) + actual(6)*delta_t + (1/2)*actual(9)*(delta_t^2);
    actual(4) = actual(4) + actual(7)*delta_t;
    actual(5) = actual(5) + actual(8)*delta_t;
    actual(6) = actual(6) + actual(9)*delta_t;

    actual(10) = actual(10) + actual(13)*delta_t + (1/2)*actual(16)*(delta_t^2);
    actual(11) = actual(11) + actual(14)*delta_t + (1/2)*actual(17)*(delta_t^2);
    actual(12) = actual(12) + actual(15)*delta_t + (1/2)*actual(18)*(delta_t^2);
    actual(13) = actual(13) + actual(16)*delta_t;
    actual(14) = actual(14) + actual(17)*delta_t;
    actual(15) = actual(15) + actual(18)*delta_t;

end


%drawf(traj, xrot, yrot, prevPts); %draw where we are now and where we should be
%prevPts(i) = actual;

%implement disturbance
%[X Y Z] = disturbance();

updated_actual = actual;
updated_phi_d = (phi_d + (phi - actual(10))) / 2;
updated_theta_d = (theta_d + (theta - actual(11))) / 2;
updated_psi_d = (psi_d + (psi - actual(12))) / 2;



end
