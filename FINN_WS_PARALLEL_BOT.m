%% Parallel Robot WorkSpace Plotter.
clear all %#ok<*CLALL> %#ok<*SAGROW> 
close all
clc


%% STEPS - FULL NOTES ON DRIVE
% 1. SPECIFY ROBOT PARAMETERS
% 2. SPECIFY RANGE OF DESIRED END POSITIONS (x_c, y_c, a)
% 4. SOLVE IK -> find (j1, j2, j3) for each arm AT EACH POSITION.
% using q for arm 1 angles.
% using s for arm 2 angles (skipping r to avoid confusing with radius
% values.
% using t for arm 3 angles.
% IGNORE IMAGINARY POSITIONS, PLOT REAL POSITIONS TO GET AN OUTLINE OF WS
%% 1. SPECIFY ROBOT PARAMETERS
% these values are taken from coursework appendix
S_A = 0.17 ; %  j1 to j2 link length
L = 0.13 ; % j2 to j3 link length
r_e = 0.13 ; % radius of circle formed by end effector joints
r_b = 0.29 ; % radius of circle formed by base joints



%% 2. SPECIFY DESIRED END POSITION (x_c, y_c, a)
x_c = -0.1:0.01:0.6 ; % test ranges for x and y
y_c = -0.1:0.01:0.6 ;
a = pi/12; % constant orientation - interesting values for plot: 0, pi/12, pi/6, pi/2.3

% pre-make lists for the elbow angles
q1 = zeros(length(x_c)*length(y_c),2) ; % 2 angles at each position for elbow angles 1 and 2
q2 = zeros(length(x_c)*length(y_c),2) ;

s1 = zeros(length(x_c)*length(y_c),2) ; % Repeat for s angles
s2 = zeros(length(x_c)*length(y_c),2) ;


t1 = zeros(length(x_c)*length(y_c),2) ; % Repeat for t angles
t2 = zeros(length(x_c)*length(y_c),2) ;

%% arm 1 - joint angles given by q
q3 = a + pi/6 ;
count = 1 ;
% two possibilities for q1, q2 as they form an "elbow"
for i = 1: length(x_c)
    for j = 1: length(y_c)
        [q1(count,1), q1(count,2), q2(count,1), q2(count,2)] = find_line_eqn(S_A, L, r_e, x_c(i), y_c(j), q3) ;
        count = count + 1;
    end
end

%% arm 2 - joint angles given by s

% Find relative position of end effector in this arms frame.
x_c2 = x_c - sqrt(3)*r_b ;
y_c2 = y_c ;

% Find angles s1, s2, s3 
s3 = a + 5*pi/6 ; % q3 + 120 degrees
count = 1 ;
% two possibilities for s1, s2 as they form an "elbow"
for i = 1: length(x_c)
    for j = 1: length(y_c)
        [s1(count,1), s1(count,2), s2(count,1), s2(count,2)] = find_line_eqn(S_A, L, r_e, x_c2(i), y_c2(j), s3) ;
        count = count + 1;
    end
end
%% arm 3 - joint angles given by t


% Find relative position of end effector in this arms frame.
x_c3 = x_c - sqrt(3)*r_b/2 ;
y_c3 = y_c - 3*r_b/2 ;

% Find angles t1, t2, t3
t3 = a + 3*pi/2 ; % s3 + 120 degrees, q3 + 240 degrees
count = 1 ;
% two possibilities for t1, t2 as they form an "elbow"
for i = 1: length(x_c)
    for j = 1: length(y_c)
        [t1(count,1), t1(count,2), t2(count,1), t2(count,2)] = find_line_eqn(S_A, L, r_e, x_c3(i), y_c3(j), t3) ;
        count = count + 1;
    end
end

%% Generate array of only real solutions
in_workspace = [] ;
count = 1 ;
for i = 1: length(x_c)
    for j = 1:length(y_c)
        angles_array_elbow1 = [x_c(i), y_c(j), q1(count,1), q2(count,1), q3, s1(count,1), s2(count,1), s3, t1(count,1), t2(count,1), t3] ;
        angles_array_elbow2 = [x_c(i), y_c(j), q1(count,2), q2(count,2), q3, s1(count,2), s2(count,2), s3, t1(count,2), t2(count,2), t3] ;
        if isreal(angles_array_elbow1) % boolean check for whether array contains imaginary numbers 
           in_workspace = [in_workspace; angles_array_elbow1] ; % if passed, append the real row
        end
        if isreal(angles_array_elbow2)% boolean check for whether array contains imaginary numbers 
           in_workspace = [in_workspace; angles_array_elbow2] ; % if passed, append the real row
        end 
        count = count + 1;
    end
end

disp(in_workspace)

% print statement to let user know if no real solutions
if isempty(in_workspace)
    disp("Chosen value of *a* has no real workspace!")
end


% get only the position values from these.
workspace_positions = [in_workspace(:,1) , in_workspace(:,2) ] ;


%% Display Solutions on Graph
figure (1) 
% plot the outer triangle
% bottom left, bottom right, top
tri_x = [0, sqrt(3)*r_b, (1/2)*sqrt(3)*r_b, 0] ;
tri_y = [0, 0, (3/2)*r_b, 0] ;
plot(tri_x,tri_y,'ro-','Linewidth',2)
hold on

% Plot the individual points
plot(workspace_positions(:,1), workspace_positions(:,2), "kx")
hold on


% Plot the workspace outline
outline = convhull(workspace_positions);
plot(workspace_positions(outline,1), workspace_positions(outline,2), "c" )

legend("Robot Base", "End Effector Positions", "Workspace Area")
axis([ -0.1 0.6 -0.1 0.6])
title('Parallel Robot WorkSpace for a = ', num2str(a)) ; xlabel('x (m)') ; ylabel('y (m)');

%% Function to find angles 1 & 2 using angle 3 and coords of end effector
% arm base frame

function [angle1_a, angle1_b, angle2_a, angle2_b] = find_line_eqn(S_A, L, r_e, x_ee, y_ee, angle3 )

% find x3, y3 the position of the joint at angle3
x3 = x_ee - r_e*cos(angle3) ;
y3 = y_ee - r_e*sin(angle3) ;

% calculate both angle1 values
angle1_a = atan2(y3,x3) + acos((S_A^2 + x3^2 + y3^2 - L^2) / (2*S_A*sqrt(x3^2 + y3^2))) ;
angle1_b = atan2(y3,x3) - acos((S_A^2 + x3^2 + y3^2 - L^2) / (2*S_A*sqrt(x3^2 + y3^2))) ;

% calculate both angle2 values

% for angle1_a, 
angle2_a = -(pi - angle1_a - acos((S_A^2 +L^2 - x3^2 -y3^2)/(2*S_A*L))) ;
% for angle1_b,
angle2_b = +(pi + angle1_b - acos((S_A^2 +L^2 - x3^2 -y3^2)/(2*S_A*L))) ;

end