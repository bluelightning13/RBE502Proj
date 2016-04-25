% main function where everything runs
%function outline()
    %main function 
    points=[0 0 0 ; 0 1 0];
    cubicPoly = (cscvn(points))
    fnplt(cubicPoly)
    coAos = cubicPoly.coefs  %this is a 4*6 array (with 3pts) with the coeffs and offsets
    breaks = cubicPoly.breaks %this is the breaks in the graphs (array of 3)
    
    tf = 10;
    position_d_coeffs = transpose(computeCubicParameters(0,tf,0,0,5,0)) % 5 units in 10 seconds
    %pause
    %velocity_d_coeffs = [0 position_d_coeffs(2)*1 position_d_coeffs(3)*2 position_d_coeffs(4)*3];
    %acceleration_d_coeffs = [0 0 velocity_d_coeffs(3)*1 velocity_d_coeffs(4)*2];
    
    actual = zeros(1,18); %initial state of robot
    desired = zeros(1,3); %desired position, velocity, acceleration at time t
    
    i = 0;%for trajectory points
    prevPts = []; %x y z roll pitch yaw
    delta_t = 0.01; %100Hz sampling frequency
    
    % initial previous orientation error
    o_pe = [0 0 0];
    % initial previous desired orientation
    o_pdo = [0 0 0];
    
    % initial previous position error
    p_pe = [0 0 0];
    % initial previous desired position
    p_pdp = [0 0 0];
    
    p_ot = [];
    
    for i = 0 : delta_t : tf % 5 second trajectory
        desired(1) = position_d_coeffs(1) + position_d_coeffs(2)*i + position_d_coeffs(3)*(i^2) + position_d_coeffs(4)*(i^3)
        desired(2) = position_d_coeffs(1) + position_d_coeffs(2)*i + position_d_coeffs(3)*(i^2) + position_d_coeffs(4)*(i^3)

        %[desired(3),desired(6),desired(9)]
        %[actual(3),actual(6),actual(9)]
        %desired(6) = velocity_d_coeffs(1) + velocity_d_coeffs(2)*i + velocity_d_coeffs(3)*(i^2) + velocity_d_coeffs(4)*(i^3);
        %desired(9) = acceleration_d_coeffs(1) + acceleration_d_coeffs(2)*i + acceleration_d_coeffs(3)*(i^2) + acceleration_d_coeffs(4)*(i^3);
        
        [actual,o_pe,o_pdo,p_pe,p_pdp] = controller(desired,actual,o_pe,o_pdo,p_pe,p_pdp);
        prevPts = [prevPts, [actual(1); actual(2); actual(3); actual(10); actual(11); actual(12)]];
        
        i
        
        drawf([real(actual(1)),real(actual(2)),real(actual(3))], real(actual(10)), real(actual(11)), [1 1 1]); %draw where we are now and where we should be
        %pause
        
        p_ot = [p_ot, transpose(desired)];
    end
    
    %pause
    
    % converge to final position
    for i = 0 : delta_t : 10 % 10 second trajectory to stabilize
        [actual,o_pe,o_pdo,p_pe,p_pdp] = controller(desired,actual,o_pe,o_pdo,p_pe,p_pdp);
        prevPts = [prevPts, [actual(1); actual(2); actual(3); actual(10); actual(11); actual(12)]];
   
        
        i
        
        drawf([real(actual(1)),real(actual(2)),real(actual(3))], real(actual(10)), real(actual(11)), [1 1 1]); %draw where we are now and where we should be
        %pause
        
        p_ot = [p_ot, transpose(desired)];
    end
    
   time = linspace(0,(tf+10),(((tf+10)/delta_t)+2));
   
   figure()
   hold on
   plot(time,p_ot(2,:),'b')
   plot(time,prevPts(2,:),'r')
   title('Y Actual vs. Desired Position')
   legend('Desired Pos.','Actual Pos.')
   xlabel('Time (s)')
   ylabel('Y Position (m)')
   hold off
    
%end

