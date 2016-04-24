% main function where everything runs
function outline()
    %main function 
    points=[0 0 0 ; 0 1 0];
    cubicPoly = (cscvn(points))
    fnplt(cubicPoly)
    coAos = cubicPoly.coefs  %this is a 4*6 array (with 3pts) with the coeffs and offsets
    breaks = cubicPoly.breaks %this is the breaks in the graphs (array of 3)
    
    position_d_coeffs = [0 0 0.6 -0.08]; % 5 units in 5 seconds
    %velocity_d_coeffs = [0 position_d_coeffs(2)*1 position_d_coeffs(3)*2 position_d_coeffs(4)*3];
    %acceleration_d_coeffs = [0 0 velocity_d_coeffs(3)*1 velocity_d_coeffs(4)*2];
    
    actual = zeros(1,18); %initial state of robot
    desired = zeros(1,9); %desired position, velocity, acceleration at time t
    
    i = 0;%for trajectory points
    prevPts = [0;0;0;0;0;0]; %x y z roll pitch yaw
    delta_t = 0.01; %100Hz sampling frequency
    
    % initial orientation error derivatives
    o_ed = [0 0 0];
    
    % initial position error derivatives
    p_ed = [0 0 0];
    
    for i = 0 : delta_t : 5 % 5 second trajectory
        desired(2) = position_d_coeffs(1) + position_d_coeffs(2)*i + position_d_coeffs(3)*(i^2) + position_d_coeffs(4)*(i^3)
        %[desired(3),desired(6),desired(9)]
        %[actual(3),actual(6),actual(9)]
        %desired(6) = velocity_d_coeffs(1) + velocity_d_coeffs(2)*i + velocity_d_coeffs(3)*(i^2) + velocity_d_coeffs(4)*(i^3);
        %desired(9) = acceleration_d_coeffs(1) + acceleration_d_coeffs(2)*i + acceleration_d_coeffs(3)*(i^2) + acceleration_d_coeffs(4)*(i^3);
        
        [actual,o_ed,p_ed] = controller(desired,actual,o_ed,p_ed);
        prevPts = [prevPts, [actual(1); actual(2); actual(3); actual(10); actual(11); actual(12)]];
        
        % update orientation error derivatives
        o_ed = o_ed * delta_t
        p_ed = p_ed * delta_t
        
        i
        
        drawf([real(actual(1)),real(actual(2)),real(actual(3))], real(actual(10)), real(actual(11)), [1 1 1]); %draw where we are now and where we should be
        %pause
    end
    pause
    for i = 0 : delta_t : 15 % 10 second trajectory to stabilize
        [actual,o_ed,p_ed] = controller(desired,actual,o_ed,p_ed);
        prevPts = [prevPts, [actual(1); actual(2); actual(3); actual(10); actual(11); actual(12)]];
        
        % update orientation error derivatives
        o_ed = o_ed * delta_t
        p_ed = p_ed * delta_t
        
        i
        
        drawf([real(actual(1)),real(actual(2)),real(actual(3))], real(actual(10)), real(actual(11)), [1 1 1]); %draw where we are now and where we should be
        %pause
    end
    
    prevPts
    
end
