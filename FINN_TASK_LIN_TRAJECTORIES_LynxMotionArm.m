%% linear motion (straight lines)
clear all %#ok<*CLALL> %#ok<*SAGROW> 
close all
clc
% link lengths
d1 = 0.1 ;
l1 = 0.1 ;
l2 = 0.1 ;
l3 = 0.1 ;

% assuming 10 HZ robot, performing each line in 1 second
%% TRYING LINEAR FIRST
%1. get start, end point and line eqn between.
%2. sample function for 10 points between each start and end point
%3. apply IK and FK

%% 1 & 2 -> find eqn for each line and add 10 points to the trajectory.
% function at bottom to do this.

points_per_line = 10 ;
points_to_plot = [[0.0 0.0 0.2]; [0.15 0.0 0.2]; [0.15 0.0 0.1]; [0.0 0.0 0.1]; [0.15 0.15 0.2]; [0.15 0.15 0.1]; [0.0 0.15 0.1]; [0.0 0.15 0.2]] ;
trajectories = zeros((size(points_to_plot,1) - 1)*points_per_line, 3) ;

% Use our function to get the gradient and constant, filter for lines along
% each axis.
for i = 1:(size(points_to_plot,1) - 1)
    [grad, const, r_dir, r] = find_line_eqn(points_to_plot(i,1),points_to_plot(i+1,1),points_to_plot(i,2),points_to_plot(i+1,2),points_to_plot(i,3),points_to_plot(i+1,3)) ; 
    for j = 1:points_per_line
   
        if r_dir == 10
            if i == 1 && j == 1
                trajectories(( (i-1)*10+j) ,3) = points_to_plot(i,3) + (points_to_plot(i+1,3)-points_to_plot(i,3))*(j/points_per_line)  ;
                continue
            end
            trajectories(( (i-1)*10+j) ,1) = trajectories(( (i-1)*10+j-1) ,1);
            trajectories(( (i-1)*10+j) ,2) = trajectories(( (i-1)*10+j-1) ,2);
            trajectories(( (i-1)*10+j) ,3) = points_to_plot(i,3) + (points_to_plot(i+1,3)-points_to_plot(i,3))*(j/points_per_line)  ;
            continue
        end
        x = points_to_plot(i,1) + r*cos(r_dir) * (j/points_per_line) ;
        y = points_to_plot(i,2) + r*sin(r_dir) * (j/points_per_line) ;
        r_temp = sqrt(x^2 + y^2) ;
        z =  grad*r_temp + const ;
        trajectories(( (i-1)*10+j) ,1) = x ;
        trajectories(( (i-1)*10+j) ,2) = y ;
        trajectories(( (i-1)*10+j) ,3) = z ;

    end
end
disp(trajectories)



%% 3
for j = 1:size(trajectories,1)
    px = trajectories(j,1) ;
    py = trajectories(j,2) ;
    pz = trajectories(j,3) ;
    
    % mu and psi angles for orientation
    mu = 0; % mu is rotation of end effector relative to "wrist"
    psi = (0:pi/20:2*pi) ;
    for i = 1: length(psi)
        psi(i) = psi(i) - pi/2; 
    end 
    End_Effector = [ px py pz ]' ;
    if norm(End_Effector) > l1+l2+l3+d1  % workspace is above ground
        error('desired position is out of the workspace')
    end
    
    
    %% %%%%%%%%%%%% Inverse Kinematics of LynxMotion Arm %%%%%%%%%%%%%%%%%
    % For details on calculations, view notes in ReadMe.
    % We must find values for the 5 joint angles below.
    sym q1 ;  % single value
    q2 = [] ; % potentially many values
    q3 = [] ; % potentially many values
    q4 = [] ; % potentially many values
    sym q5 ;  % single value
    
    %% Simple angles to find: 
    
    % Find q1
    q1 = atan2(py,px) ;
    % Find q5
    q5 = mu ;
    
    %% Define extra variables to help us find q2,3,4
    
    % define (r,z) plane
    r = sqrt(px^2 + py^2) ; % r is hypotenues in x-y plane.
    
    % For each value of psi, there is a value of rw, zw and D
    r_w = (1:length(psi)) ;
    z_w = (1:length(psi)) ;
    D = (1:length(psi)) ;
    for i = 1:length(psi)
    % define position of wrist in (r,z) plane
        r_w(i) = r - l3*cos(psi(i)) ; %r_w and z_w will be real as cos or sin of a real number is a real number
        
        z_w(i) = pz - d1 -l3*sin(psi(i)) ;
        
        % define D - a placeholder variable for a large combination we derived in
        % notes
        
        D(i) = - (r_w(i)^2 + z_w(i)^2 - l1^2 - l2^2) / (2*l1*l2) ;
        D(i) = round(D(i), 7)  ;% rounding D avoids fake imaginary numbers due to rounding
                        % errors.
    end
    %% get only real values
    
    psi_real = [] ;
    r_w_real = [] ;
    z_w_real = [] ;
    D_real = [] ;
    for i = 1:length(psi)
        if imag(sqrt(1-D(i)^2)) == 0  % for real values, append to real lists.
            psi_real(end + 1) = psi(i);%round(psi(i),7) ; % round each list to get zeros rather than e^-10 or something.
            r_w_real(end + 1) = r_w(i);%round(r_w(i),7) ;
            z_w_real(end + 1) = z_w(i);%round(z_w(i),7) ;
            D_real(end + 1)= D(i);%round(D(i),7) ;
        end
    end
    
    % we can see these four lists MUST have same length.
    %  disp(psi_real)
    %  disp(D_real)
    %  disp(z_w_real)
    %  disp(r_w_real)
    
    %% Find q3 possibilities
    % there are two posibilities for each value of D depending on
    for i = 1:length(D_real)
    
        q3(end+1) = atan2( sqrt(1-D_real(i)^2), -D_real(i)) ;
        q3(end+1) = atan2(-sqrt(1-D_real(i)^2), -D_real(i)) ;
    end
    
    %  disp("q3")    
    %  disp(q3)
    %% Explaining list order. 
    % matlab indexes from 1 not 0.
    % if D has 3 values, we can see q3 will have 6, where 
    % D(1) corresponds to q3(1 & 2)
    % D(2) corresponds to q3(3 & 4)
    % D(3) corresponds to q3(5 & 6)
    % we can see D(i) corresponds to values
    % q3(2*i -1) and q3(2*i)
    % we need to ensure this is *consistant* across all q lists below.
    %% Find q2
    
    % two options based on q3
    for i = 1:length(D_real)
        % for each r_w,z_w value, find first q2 value using first q3 value.
        % Switching order here corrects angles... why?
        q2(end+1) = atan2(z_w_real(i),r_w_real(i)) + atan2(l2*sin(q3(2*i)), l1+l2*cos(q3(2*i))) ;
        % add +1 to 
        q2(end+1) = atan2(z_w_real(i),r_w_real(i)) + atan2(l2*sin(q3(2*i-1)), l1+l2*cos(q3(2*i-1))) ;
    end
    %  disp("q2")
    %  disp(q2)
    %% Find q4 
    
    % two options based on two sets of q2,q3
    % use psi = q2 + q3 + q4
    for i = 1:length(D_real)
        q4(end+1) = psi_real(i) - q2(2*i -1) - q3(2*i -1) +pi/2 ;
        q4(end+1) = psi_real(i) - q2(2*i) - q3(2*i) +pi/2 ;
    end
    
    %  disp("q4")
    %  disp(q4)
    %% Remove duplicate values
    % make a matrix out of the lists where one row is one solution
    
    % we know matrix will have initial size: no. of items in q2,3 or 4 x 5
    % angles
    Solution_Matrix = zeros(length(q2), 5);  
    for i = 1:length(q2)
        Solution_Matrix(i,1) = q1 ;
        Solution_Matrix(i,2) = q2(i) ;
        Solution_Matrix(i,3) = q3(i) ;
        Solution_Matrix(i,4) = q4(i) ; 
        Solution_Matrix(i,5) = q5 ;
    end
    % disp(Solution_Matrix)
    % remove duplicate rows
    Unique_Solutions = unique(Solution_Matrix,"rows","stable") ;
    % stable prevents order being changed.
    
    
    %% we also know, q4 can't be more than 360 degrees. filter these out.
    valid_Solutions = [] ;
    for i = 1:size(Unique_Solutions,1)
        if abs(Unique_Solutions(i,4)) < 2*pi
            valid_Solutions = [valid_Solutions; Unique_Solutions(i,:)] ; 
        end
    end
    % stable prevents order being changed.
        for i = 1:5
            IK_OUTPUT(j,i) = valid_Solutions(1,i) ;
    
        end
end
disp("Using IK: Array of q1-5 values:")
%disp(IK_OUTPUT);



%% 3. use FK to plot the arm trajectory in 3d space 
syms q1 q2 q3 q4 q5; % angle variables

% Show our 5 tranformation matrices
T_01 =[
[cos(q1), 0,  sin(q1),  0]
[sin(q1), 0, -cos(q1),  0]
[      0, 1,        0, d1]
[      0, 0,        0,  1]];

T_12 =[
[cos(q2), -sin(q2), 0, l1*cos(q2)]
[sin(q2),  cos(q2), 0, l1*sin(q2)]
[      0,        0, 1,          0]
[      0,        0, 0,          1]];

T_23 = [
[cos(q3), -sin(q3), 0, l2*cos(q3)]
[sin(q3),  cos(q3), 0, l2*sin(q3)]
[      0,        0, 1,          0]
[      0,        0, 0,          1]];

T_34 = [     
[cos(q4), 0,  sin(q4),          0]
[sin(q4), 0, -cos(q4),          0]
[      0, 1,        0,          0]
[      0, 0,        0,          1]];

T_45 = [
[cos(q5), -sin(q5), 0,  0]
[sin(q5),  cos(q5), 0,  0]
[      0,        0, 1,  l3]
[      0,        0, 0,  1]];

% Get overall forward kinematics
T_e = T_01*T_12*T_23*T_34*T_45;

% extract the x,y,z elements of these
xt_element = T_e(1,4); % the x transformation is the 4th element of the 1st row of our general transformation matrix, T_e. 
yt_element = T_e(2,4); % the y transformation is the 4th element of the 2nd row of our general transformation matrix, T_e. 
zt_element = T_e(3,4); % the z transformation is the 4th element of the 3rd row of our general transformation matrix, T_e. 

% Define all our angles in their own arrays
q1_set = [] ;
q2_set = [] ;
q3_set = [] ;
q4_set = [] ;
q5_set = [] ;
for i = 1:size(IK_OUTPUT,1) % i in range number of rows
    % number of columns always 5 for q1-5
    q1_set(i) = IK_OUTPUT(i,1) ;
    q2_set(i) = IK_OUTPUT(i,2) ;
    q3_set(i) = IK_OUTPUT(i,3) ;
    q4_set(i) = IK_OUTPUT(i,4) ;
    q5_set(i) = IK_OUTPUT(i,5) ;
end


%
%Now simply plot using our FK


% define variable for number of points
NoP = size(IK_OUTPUT,1) ; % same as number of sets of q values.

%base is at origin
base = 0 ;


% next joint given by T01 translations (movement from base to joint 1)
% need to make into a 4 by 3 array.
Tj1_x = zeros(NoP,1) ; % x translation is zero
Tj1_y = zeros(NoP,1) ; % y translation is zero
Tj1_z = subs(zeros(NoP,1), 0, d1) ;  % z translation is d1

% next joint given by T_01T_12 translations - movement from base to joint 2
% make our general elements for each array
movement = T_01*T_12 ;
xt_element = movement(1,4) ;
yt_element = movement(2,4) ;
zt_element = movement(3,4) ;
% fill the joint 2 4x1 arrays
Tj2_x = [] ;
Tj2_y = [] ;
Tj2_z = [] ;
for i = 1:length(q1_set)
    xt_value = subs(xt_element, [ q1 q2], [q1_set(i) q2_set(i)]) ;
    yt_value = subs(yt_element, [ q1 q2], [q1_set(i) q2_set(i)]) ;
    zt_value = subs(zt_element, [ q1 q2], [q1_set(i) q2_set(i)]) ;
    Tj2_x(i,1) = xt_value ;
    Tj2_y(i,1) = yt_value ;
    Tj2_z(i,1) = zt_value ;
end
% above worked, now continue for the rest.


% next joint given by T_01T_12T_23 translations - movement from base to joint 3
% make our general elements for each array
movement = T_01*T_12*T_23 ;
xt_element = movement(1,4) ;
yt_element = movement(2,4) ;
zt_element = movement(3,4) ;
% fill the joint 2 4x1 arrays
Tj3_x = [] ;
Tj3_y = [] ;
Tj3_z = [] ;
for i = 1:length(q1_set)
    xt_value = subs(xt_element,[q1 q2 q3], [q1_set(i) q2_set(i) q3_set(i)]) ;
    yt_value = subs(yt_element,[q1 q2 q3], [q1_set(i) q2_set(i) q3_set(i)]) ;
    zt_value = subs(zt_element,[q1 q2 q3], [q1_set(i) q2_set(i) q3_set(i)]) ;
    Tj3_x(i,1) = xt_value ;
    Tj3_y(i,1) = yt_value ;
    Tj3_z(i,1) = zt_value ;
end


% next joint given by T_01T_12T_23T_34 translations - movement from base to joint 4
% note there's no translation in this joint so no change from above.
% make our general elements for each array
movement = T_01*T_12*T_23*T_34 ;
xt_element = movement(1,4) ;
yt_element = movement(2,4) ;
zt_element = movement(3,4) ;
% fill the joint 2 4x1 arrays
Tj4_x = [] ;
Tj4_y = [] ;
Tj4_z = [] ;
for i = 1:length(q1_set)
    xt_value = subs(xt_element,[q1 q2 q3 q4], [q1_set(i) q2_set(i) q3_set(i) q4_set(i)]) ;
    yt_value = subs(yt_element,[q1 q2 q3 q4], [q1_set(i) q2_set(i) q3_set(i) q4_set(i)]) ;
    zt_value = subs(zt_element,[q1 q2 q3 q4], [q1_set(i) q2_set(i) q3_set(i) q4_set(i)]) ;
    Tj4_x(i,1) = xt_value ;
    Tj4_y(i,1) = yt_value ;
    Tj4_z(i,1) = zt_value ;
end

% final joint given by T_01T_12T_23T_34T_45 translations - movement from base to joint 4
% note there's no translation in this joint so no change from above.
% make our general elements for each array
movement = T_01*T_12*T_23*T_34*T_45 ;
xt_element = movement(1,4) ;
yt_element = movement(2,4) ;
zt_element = movement(3,4) ;
% fill the joint 2 4x1 arrays
EE = [] ; % End Effector Positions list
for i = 1:length(q1_set)
    xt_value = subs(xt_element,[q1 q2 q3 q4], [q1_set(i) q2_set(i) q3_set(i) q4_set(i)]) ;
    yt_value = subs(yt_element,[q1 q2 q3 q4], [q1_set(i) q2_set(i) q3_set(i) q4_set(i)]) ;
    zt_value = subs(zt_element,[q1 q2 q3 q4], [q1_set(i) q2_set(i) q3_set(i) q4_set(i)]) ;
    EE(i,1) = xt_value ; % x values in column 1
    EE(i,2) = yt_value ; % y values in column 2
    EE(i,3) = zt_value ; % z values in column 3
end

% Check if end effector positions match points to plot.
disp("EE values (should match points to plot): ")
disp(EE)
disp("do they match? 0 = no, 1 = yes: ")
isequal(round(trajectories,4,"decimals"),round(EE,4,"decimals"))

figure (2) 

    % generate the graph. Note: only 4 joints will be visible as there is
    % no spatial distinction between joints three and 4.
for i = 1:NoP % zeros is the base position, doesn't change.
    xx = [base; Tj1_x(i); Tj2_x(i); Tj3_x(i); Tj4_x(i); EE(i,1) ] ;
    yy = [base; Tj1_y(i); Tj2_y(i); Tj3_y(i); Tj4_y(i); EE(i,2) ] ;
    zz = [base; Tj1_z(i); Tj2_z(i); Tj3_z(i); Tj4_z(i); EE(i,3) ] ;
    axis([ 0 0.2 0 0.2 0 0.3 ])
    plot3(xx,yy,zz,'ko-','Linewidth',2)
    hold on % CHANGE TO HOLD ON TO SEE ALL LINES AT ONCE.
    pause(0.3)

    % label axes, start point, end point.
    text(EE(1,1) + 0.002,EE(1,2) + 0.002,EE(1,3) + 0.002,'Start') ;
    text(EE(end,1) + 0.002,EE(end,2) + 0.002,EE(end,3) + 0.002,'End') ;

    % label end effector points
    text(EE(i,1),EE(i,2),EE(i,3), 'x')
end
title('Actual Points') ; xlabel('x (m)') ; ylabel('y (m)') ; zlabel('z (m)');


figure (3) % end points only
for i = 1:NoP 
    plot3(EE(i,1),EE(i,2),EE(i,3),'ko-','Linewidth',2)
    hold on
end
title('Actual End Points Only') ; xlabel('x (m)') ; ylabel('y (m)') ; zlabel('z (m)');
axis([ 0 0.2 0 0.2 0 0.3 ])




%% Function to find gradient and constant of line equation
function [grad, const, r_dir, r] = find_line_eqn(x1, x2, y1, y2, z1, z2)

    % find which vector r = in x,y plane magnitude and direction
    % r dir is angle from x axis along r, at the point of x1,y1 in the x,y
    % plane.

    % check for lines along  each axis first
    % line along z 
    if (x2 - x1) == 0 && (y2 - y1) == 0
        grad = "null" ;
        const = "null" ;
        r_dir = 10 ; % max value of atan2 is < pi/2 therefore < 10
        r2 = 0 ;
        r1 = 0 ;
        r = 0 ;
        return
    % line along y
    elseif (x2 - x1) == 0
        r1 = y1 ;
        r2 = y2 ;
        r = r2 -r1 ;
        r_dir = pi/2 ;
    % line along x
    elseif (y2 - y1) == 0
        r1 = x1 ;
        r2 = x2 ;
        r = r2 -r1 ;
        r_dir = 0 ;
    else
        r1 = sqrt(x1^2 + y1^2) ;
        r2 = sqrt(x2^2 + y2^2) ;
        r = sqrt((y2-y1)^2 + (x2-x1)^2) ;
        r_dir = atan2(y2-y1, x2-x1) ;
        if r_dir < 0
            r_dir = pi - r_dir;
        end
    end
    
    grad = (z2-z1)/(r) ;
    const = (z1*r2 - z2*r1)/ (r) ;
end
