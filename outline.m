% main function where everything runs
function outline()
    %main function 
    points=[0 0 0 ; 0 1 0];
    cubicPoly = (cscvn(points))
    fnplt(cubicPoly)
    coAos = cubicPoly.coefs  %this is a 4*6 array (with 3pts) with the coeffs and offsets
    breaks = cubicPoly.breaks %this is the breaks in the graphs (array of 3)
    
    position_d_coeffs = [0 0 0.6 -0.08]; % 5 units in 5 seconds
    velocity_d_coeffs = [0 position_d_coeffs(2)*1 position_d_coeffs(3)*2 position_d_coeffs(4)*3];
    acceleration_d_coeffs = [0 0 velocity_d_coeffs(3)*1 velocity_d_coeffs(4)*2];
    
    actual = zeros(1,18); %initial state of robot
    desired = zeros(1,9); %desired position, velocity, acceleration at time t
    
    i = 0;%for trajectory points
    prevPts = [0;0;0;0;0;0]; %x y z roll pitch yaw
    delta_t = 0.01; %100Hz sampling frequency
    
    % initial errors
    phi_d = 0; 
    theta_d = 0;
    psi_d = 0;
    
    for i = 0 : delta_t : 5 % 5 second trajectory
        desired(2) = position_d_coeffs(1) + position_d_coeffs(2)*i + position_d_coeffs(3)*(i^2) + position_d_coeffs(4)*(i^3);
        desired(5) = velocity_d_coeffs(1) + velocity_d_coeffs(2)*i + velocity_d_coeffs(3)*(i^2) + velocity_d_coeffs(4)*(i^3);
        desired(8) = acceleration_d_coeffs(1) + acceleration_d_coeffs(2)*i + acceleration_d_coeffs(3)*(i^2) + acceleration_d_coeffs(4)*(i^3);
        
        [actual,phi_d,theta_d,psi_d] = controller(desired,actual,phi_d,theta_d,psi_d);
        prevPts = [prevPts, [actual(1); actual(2); actual(3); actual(10); actual(11); actual(12)]];
        
        % update orientation error derivatives
        phi_d = phi_d * delta_t;
        theta_d = theta_d * delta_t;
        psi_d = psi_d * delta_t;
        
        drawf([real(actual(1)),real(actual(2)),real(actual(3))], real(actual(10)), real(actual(11)), [1 1 1]); %draw where we are now and where we should be
    end
    
    for i = 0 : delta_t : 10 % 10 second trajectory to stabilize
        [actual,phi_d,theta_d,psi_d] = controller(desired,actual,phi_d,theta_d,psi_d);
        prevPts = [prevPts, [actual(1); actual(2); actual(3); actual(10); actual(11); actual(12)]];
        
        % update orientation error derivatives
        phi_d = phi_d * delta_t;
        theta_d = theta_d * delta_t;
        psi_d = psi_d * delta_t;
        
        drawf([real(actual(1)),real(actual(2)),real(actual(3))], real(actual(10)), real(actual(11)), [1 1 1]); %draw where we are now and where we should be
    end
    
    prevPts
    
end
