% main function where everything runs
%function outline()
    %main function 
    
    warning('off','all')
    
    points=[0 0 0 ; 0 1 0];
    cubicPoly = (cscvn(points))
    fnplt(cubicPoly)
    coAos = cubicPoly.coefs  %this is a 4*6 array (with 3pts) with the coeffs and offsets
    breaks = cubicPoly.breaks %this is the breaks in the graphs (array of 3)
    
    tf = 2.5;
    position_d_coeffs_x = transpose(computeCubicParameters(0,tf,0,0,2.5,1.5)) % 5 units in 10 seconds
    %pause
    %velocity_d_coeffs = [0 position_d_coeffs(2)*1 position_d_coeffs(3)*2 position_d_coeffs(4)*3]
    %acceleration_d_coeffs = [0 0 velocity_d_coeffs(3)*1 velocity_d_coeffs(4)*2];
    
    actual = zeros(1,18); %initial state of robot
    desired = zeros(1,6); %desired position, velocity, acceleration at time t
    
    i = 0;%for trajectory points
    prevPts = []; %x y z roll pitch yaw
    delta_t = 0.01; %100Hz sampling frequency
    error = []; %error per time step
    
    % initial previous orientation error
    o_pe = [0 0 0];
    % initial previous desired orientation
    o_pdo = [0 0 0];
    %list of desired orientations
    l_do = [];
    
    % initial previous position error
    p_pe = [0 0 0];
    % initial previous desired position
    p_pdp = [0 0 0];
    
    p_ot = [];
    
    % wind velocity vector
    wind = [3.89;3.89;0]; % magnitude of 5.5m/s is average windspeed
    
    % set up video:
    %nframes = 1500
    %vidObj = VideoWriter('QuadSimVideo.avi');
    %vidObj.Quality = 100;
    %vidObj.FrameRate = 30;
    %open(vidObj);
    
    
    for i = 0 : delta_t : tf % 5 second trajectory
        desired(1) = position_d_coeffs_x(1) + position_d_coeffs_x(2)*i + position_d_coeffs_x(3)*(i^2) + position_d_coeffs_x(4)*(i^3);
        %desired(2) = position_d_coeffs(1) + position_d_coeffs(2)*i + position_d_coeffs(3)*(i^2) + position_d_coeffs(4)*(i^3);

        %[desired(3),desired(6),desired(9)]
        %[actual(3),actual(6),actual(9)]
        %desired(6) = velocity_d_coeffs(1) + velocity_d_coeffs(2)*i + velocity_d_coeffs(3)*(i^2) + velocity_d_coeffs(4)*(i^3);
        %desired(9) = acceleration_d_coeffs(1) + acceleration_d_coeffs(2)*i + acceleration_d_coeffs(3)*(i^2) + acceleration_d_coeffs(4)*(i^3);
        
        error = [error, [desired(1) - actual(1); desired(2) - actual(2); desired(3) - actual(3)]];
        [actual,o_pe,o_pdo,p_pe,p_pdp] = controller(desired,actual,o_pe,o_pdo,p_pe,p_pdp,wind);
        prevPts = [prevPts, [actual(1); actual(2); actual(3); actual(10); actual(11); actual(12)]];
        l_do = [l_do, transpose(o_pdo)];
        
        i
        
        drawf([real(actual(1)),real(actual(2)),real(actual(3))], real(actual(10)), real(actual(11)), desired); %draw where we are now and where we should be
        %pause
        
        %writeVideo(vidObj, getframe);
        
        p_ot = [p_ot, transpose(desired)];
    end
    
    %pause
    
    position_d_coeffs_x = transpose(computeCubicParameters(0,tf,2.5,1.5,5,0)); % 5 units in 10 seconds
    position_d_coeffs_y = transpose(computeCubicParameters(0,tf,0,0,2.5,1.5)); % 5 units in 10 seconds

    
    for i = delta_t : delta_t : tf % 5 second trajectory
        desired(1) = position_d_coeffs_x(1) + position_d_coeffs_x(2)*i + position_d_coeffs_x(3)*(i^2) + position_d_coeffs_x(4)*(i^3);
        desired(2) = position_d_coeffs_y(1) + position_d_coeffs_y(2)*i + position_d_coeffs_y(3)*(i^2) + position_d_coeffs_y(4)*(i^3);

        %[desired(3),desired(6),desired(9)]
        %[actual(3),actual(6),actual(9)]
        %desired(6) = velocity_d_coeffs(1) + velocity_d_coeffs(2)*i + velocity_d_coeffs(3)*(i^2) + velocity_d_coeffs(4)*(i^3);
        %desired(9) = acceleration_d_coeffs(1) + acceleration_d_coeffs(2)*i + acceleration_d_coeffs(3)*(i^2) + acceleration_d_coeffs(4)*(i^3);
        
        error = [error, [desired(1) - actual(1); desired(2) - actual(2); desired(3) - actual(3)]];
        [actual,o_pe,o_pdo,p_pe,p_pdp] = controller(desired,actual,o_pe,o_pdo,p_pe,p_pdp,wind);
        prevPts = [prevPts, [actual(1); actual(2); actual(3); actual(10); actual(11); actual(12)]];
        l_do = [l_do, transpose(o_pdo)];
        
        i
        
        drawf([real(actual(1)),real(actual(2)),real(actual(3))], real(actual(10)), real(actual(11)), desired); %draw where we are now and where we should be
        %pause
        
        %writeVideo(vidObj, getframe);
        
        p_ot = [p_ot, transpose(desired)];
    end
    
    position_d_coeffs_x = transpose(computeCubicParameters(0,tf,5,0,2.5,-1.5)); % 5 units in 10 seconds
    position_d_coeffs_y = transpose(computeCubicParameters(0,tf,2.5,1.5,5,0)); % 5 units in 10 seconds
    
    for i = delta_t : delta_t : tf % 5 second trajectory
        desired(1) = position_d_coeffs_x(1) + position_d_coeffs_x(2)*i + position_d_coeffs_x(3)*(i^2) + position_d_coeffs_x(4)*(i^3);
        desired(2) = position_d_coeffs_y(1) + position_d_coeffs_y(2)*i + position_d_coeffs_y(3)*(i^2) + position_d_coeffs_y(4)*(i^3);

        %[desired(3),desired(6),desired(9)]
        %[actual(3),actual(6),actual(9)]
        %desired(6) = velocity_d_coeffs(1) + velocity_d_coeffs(2)*i + velocity_d_coeffs(3)*(i^2) + velocity_d_coeffs(4)*(i^3);
        %desired(9) = acceleration_d_coeffs(1) + acceleration_d_coeffs(2)*i + acceleration_d_coeffs(3)*(i^2) + acceleration_d_coeffs(4)*(i^3);

        error = [error, [desired(1) - actual(1); desired(2) - actual(2); desired(3) - actual(3)]];
        [actual,o_pe,o_pdo,p_pe,p_pdp] = controller(desired,actual,o_pe,o_pdo,p_pe,p_pdp,wind);
        prevPts = [prevPts, [actual(1); actual(2); actual(3); actual(10); actual(11); actual(12)]];
        l_do = [l_do, transpose(o_pdo)];
        
        i
        
        drawf([real(actual(1)),real(actual(2)),real(actual(3))], real(actual(10)), real(actual(11)), desired); %draw where we are now and where we should be
        %pause
        
        %writeVideo(vidObj, getframe);
        
        p_ot = [p_ot, transpose(desired)];
    end
    
    %pause
    
    position_d_coeffs_x = transpose(computeCubicParameters(0,tf,2.5,-1.5,0,0)); % 5 units in 10 seconds
    position_d_coeffs_y = transpose(computeCubicParameters(0,tf,5,0,2.5,-1.5)); % 5 units in 10 seconds
    
    for i = delta_t : delta_t : tf % 5 second trajectory
        desired(1) = position_d_coeffs_x(1) + position_d_coeffs_x(2)*i + position_d_coeffs_x(3)*(i^2) + position_d_coeffs_x(4)*(i^3);
        desired(2) = position_d_coeffs_y(1) + position_d_coeffs_y(2)*i + position_d_coeffs_y(3)*(i^2) + position_d_coeffs_y(4)*(i^3);

        %[desired(3),desired(6),desired(9)]
        %[actual(3),actual(6),actual(9)]
        %desired(6) = velocity_d_coeffs(1) + velocity_d_coeffs(2)*i + velocity_d_coeffs(3)*(i^2) + velocity_d_coeffs(4)*(i^3);
        %desired(9) = acceleration_d_coeffs(1) + acceleration_d_coeffs(2)*i + acceleration_d_coeffs(3)*(i^2) + acceleration_d_coeffs(4)*(i^3);
        
        error = [error, [desired(1) - actual(1); desired(2) - actual(2); desired(3) - actual(3)]];
        [actual,o_pe,o_pdo,p_pe,p_pdp] = controller(desired,actual,o_pe,o_pdo,p_pe,p_pdp,wind);
        prevPts = [prevPts, [actual(1); actual(2); actual(3); actual(10); actual(11); actual(12)]];
        l_do = [l_do, transpose(o_pdo)];
        
        i
        
        drawf([real(actual(1)),real(actual(2)),real(actual(3))], real(actual(10)), real(actual(11)), desired); %draw where we are now and where we should be
        %pause
        
        %writeVideo(vidObj, getframe);
        
        p_ot = [p_ot, transpose(desired)];
    end
    
    position_d_coeffs_x = transpose(computeCubicParameters(0,tf,0,0,2.5,1.5)); % 5 units in 10 seconds
    position_d_coeffs_y = transpose(computeCubicParameters(0,tf,2.5,-1.5,0,0)); % 5 units in 10 seconds
    
    for i = delta_t : delta_t : tf % 5 second trajectory
        desired(1) = position_d_coeffs_x(1) + position_d_coeffs_x(2)*i + position_d_coeffs_x(3)*(i^2) + position_d_coeffs_x(4)*(i^3);
        desired(2) = position_d_coeffs_y(1) + position_d_coeffs_y(2)*i + position_d_coeffs_y(3)*(i^2) + position_d_coeffs_y(4)*(i^3);

        %[desired(3),desired(6),desired(9)]
        %[actual(3),actual(6),actual(9)]
        %desired(6) = velocity_d_coeffs(1) + velocity_d_coeffs(2)*i + velocity_d_coeffs(3)*(i^2) + velocity_d_coeffs(4)*(i^3);
        %desired(9) = acceleration_d_coeffs(1) + acceleration_d_coeffs(2)*i + acceleration_d_coeffs(3)*(i^2) + acceleration_d_coeffs(4)*(i^3);
        
        error = [error, [desired(1) - actual(1); desired(2) - actual(2); desired(3) - actual(3)]];
        [actual,o_pe,o_pdo,p_pe,p_pdp] = controller(desired,actual,o_pe,o_pdo,p_pe,p_pdp,wind);
        prevPts = [prevPts, [actual(1); actual(2); actual(3); actual(10); actual(11); actual(12)]];
        l_do = [l_do, transpose(o_pdo)];
        
        i
        
        drawf([real(actual(1)),real(actual(2)),real(actual(3))], real(actual(10)), real(actual(11)), desired); %draw where we are now and where we should be
        %pause
        
        %writeVideo(vidObj, getframe);
        
        p_ot = [p_ot, transpose(desired)];
    end
    
    %pause
    
    position_d_coeffs_x = transpose(computeCubicParameters(0,tf,2.5,1.5,5,0)); % 5 units in 10 seconds
    
    for i = delta_t : delta_t : tf % 5 second trajectory
        desired(1) = position_d_coeffs_x(1) + position_d_coeffs_x(2)*i + position_d_coeffs_x(3)*(i^2) + position_d_coeffs_x(4)*(i^3);
        %desired(2) = position_d_coeffs(1) + position_d_coeffs(2)*i + position_d_coeffs(3)*(i^2) + position_d_coeffs(4)*(i^3);

        %[desired(3),desired(6),desired(9)]
        %[actual(3),actual(6),actual(9)]
        %desired(6) = velocity_d_coeffs(1) + velocity_d_coeffs(2)*i + velocity_d_coeffs(3)*(i^2) + velocity_d_coeffs(4)*(i^3);
        %desired(9) = acceleration_d_coeffs(1) + acceleration_d_coeffs(2)*i + acceleration_d_coeffs(3)*(i^2) + acceleration_d_coeffs(4)*(i^3);
        
        error = [error, [desired(1) - actual(1); desired(2) - actual(2); desired(3) - actual(3)]];
        [actual,o_pe,o_pdo,p_pe,p_pdp] = controller(desired,actual,o_pe,o_pdo,p_pe,p_pdp,wind);
        prevPts = [prevPts, [actual(1); actual(2); actual(3); actual(10); actual(11); actual(12)]];
        l_do = [l_do, transpose(o_pdo)];
        
        i
        
        drawf([real(actual(1)),real(actual(2)),real(actual(3))], real(actual(10)), real(actual(11)), desired); %draw where we are now and where we should be
        %pause
        %writeVideo(vidObj, getframe);
        p_ot = [p_ot, transpose(desired)];
    end
  
    %close(vidObj);
    
    % converge to final position
%     for i = delta_t : delta_t : 10 % 10 second trajectory to stabilize
%         [actual,o_pe,o_pdo,p_pe,p_pdp] = controller(desired,actual,o_pe,o_pdo,p_pe,p_pdp);
%         prevPts = [prevPts, [actual(1); actual(2); actual(3); actual(10); actual(11); actual(12)]];
%         l_do = [l_do, transpose(o_pdo)];
%         
%         i
%         
%         drawf([real(actual(1)),real(actual(2)),real(actual(3))], real(actual(10)), real(actual(11)), desired); %draw where we are now and where we should be
%         %pause
%         
%         p_ot = [p_ot, transpose(desired)];
%     end
    
   time = linspace(0,(6*tf),(((6*tf)/delta_t)+1));
   
   figure(2)
   hold on
   plot(time,p_ot(1,:),'b')
   plot(time,prevPts(1,:),'r')
   title('X Actual vs. Desired Position')
   legend('Desired Pos.','Actual Pos.')
   xlabel('Time (s)')
   ylabel('X Position (m)')
   hold off
   
   figure(3)
   hold on
   plot(time,p_ot(2,:),'b')
   plot(time,prevPts(2,:),'r')
   title('Y Actual vs. Desired Position')
   legend('Desired Pos.','Actual Pos.')
   xlabel('Time (s)')
   ylabel('Y Position (m)')
   hold off
   
   figure(4)
   hold on
   plot(time,p_ot(3,:),'b')
   plot(time,prevPts(3,:),'r')
   title('Z Actual vs. Desired Position')
   legend('Desired Pos.','Actual Pos.')
   xlabel('Time (s)')
   ylabel('Z Position (m)')
   hold off
    
   figure(5)
   hold on
   plot(time,l_do(1,:),'b')
   plot(time,prevPts(4,:),'r')
   title('Phi Actual vs. Desired Phi')
   legend('Desired Phi','Actual Phi')
   xlabel('Time (s)')
   ylabel('Phi Position (rads)')
   hold off
   
   figure(6)
   hold on
   plot(time,l_do(2,:),'b')
   plot(time,prevPts(5,:),'r')
   title('Theta Actual vs. Desired Theta')
   legend('Desired Theta','Actual Theta')
   xlabel('Time (s)')
   ylabel('Theta Position (rads)')
   hold off
   
   figure(7)
   hold on
   plot(time,l_do(3,:),'b')
   plot(time,prevPts(6,:),'r')
   title('Psi Actual vs. Desired Psi')
   legend('Desired Psi','Actual Psi')
   xlabel('Time (s)')
   ylabel('Psi Position (rads)')
   hold off
   
   figure(8)
   hold on
   plot(p_ot(1,:),p_ot(2,:),'b')
   plot(prevPts(1,:),prevPts(2,:),'r')
   title('XY Actual and XY Desired')
   legend('Desired Pos.','Actual Pos.')
   xlabel('X position (m)')
   ylabel('Y Position (m)')
   hold off
   
   figure(9)
   hold on
   plot(time,error(1,:),'b')
   title('X Error Over Time')
   xlabel('Time (s)')
   ylabel('X Position Error (m)')
   hold off
   
   figure(10)
   hold on
   plot(time,error(2,:),'b')
   title('Y Error Over Time')
   xlabel('Time (s)')
   ylabel('Y Position Error (m)')
   hold off
   
   figure(11)
   hold on
   plot(time,error(3,:),'b')
   title('Z Error Over Time')
   xlabel('Time (s)')
   ylabel('Z Position Error (m)')
   hold off
   
   % print average errors
   avg_errors = sum(abs(transpose(error)))/1501
  
   % 3d plot code for later:
%    figure()
%    hold on
%    plot3(p_ot(1,:),p_ot(2,:),p_ot(3,:),'b')
%    plot3(prevPts(1,:),prevPts(2,:),prevPts(3,:),'r')
%    title('XY Actual and XY Desired')
%    legend('Desired Pos.','Actual Pos.')
%    xlabel('X position (m)')
%    ylabel('Y Position (m)')
%    zlabel('Z Position (m)')
%    hold off
   
%end

