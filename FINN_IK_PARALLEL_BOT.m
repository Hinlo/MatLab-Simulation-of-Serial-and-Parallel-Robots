%% Inverse Kinematics Calculator for LynxMotion Arm
clear all %#ok<*CLALL> %#ok<*SAGROW> 
close all
clc


%% STEPS - FULL NOTES ON DRIVE
% 1. SPECIFY ROBOT PARAMETERS
% 2. SPECIFY DESIRED END POSITION (x_c, y_c, a)
% 3. SOLVE IK -> find (j1, j2, j3) for each arm.
% 4. PRINT THE 8 POSSIBLE ORIENTATIONS, DISPLAY 2 OF THESE IN FIGURES ONE
% AND TWO.
% using q for arm 1 angles.
% using s for arm 2 angles (skipping r to avoid confusing with radius
% values.
% using t for arm 3 angles.
%% 1. SPECIFY ROBOT PARAMETERS
% these values are taken from coursework appendix
S_A = 0.17 ; %  j1 to j2 link length
L = 0.13 ; % j2 to j3 link length
r_e = 0.13 ; % radius of circle formed by end effector joints
r_b = 0.29 ; % radius of circle formed by base joints



%% 2. SPECIFY DESIRED END POSITION (x_c, y_c, a)
x_c = 0.25 ;
y_c = 0.21 ;
a = pi/6 ;


%% arm 1 - joint angles given by q
q3 = a + pi/6 ;
% two possibilities for q1, q2 as they form an "elbow"
q1 = [] ;
q2 = [] ;
[q1(1), q1(2), q2(1), q2(2)] = find_line_eqn(S_A, L, r_e, x_c, y_c, q3) ;


%% arm 2 - joint angles given by s

% Find relative position of end effector in this arms frame.
x_c2 = x_c - sqrt(3)*r_b ;
y_c2 = y_c ;

% Find angles s1, s2, s3 
s3 = a + 5*pi/6 ; % q3 + 120 degrees
% two possibilities for s1, s2 as they form an "elbow"
s1 = [] ;
s2 = [] ;
[s1(1), s1(2), s2(1), s2(2)] = find_line_eqn(S_A, L, r_e, x_c2, y_c2, s3) ;


%% arm 3 - joint angles given by t


% Find relative position of end effector in this arms frame.
x_c3 = x_c - sqrt(3)*r_b/2 ;
y_c3 = y_c - (3/2)*r_b ;

% Find angles t1, t2, t3
t3 = a + 3*pi/2 ; % s3 + 120 degrees, q3 + 240 degrees
% two possibilities for t1, t2 as they form an "elbow"
t1 = [] ;
t2 = [] ;
[t1(1), t1(2), t2(1), t2(2)] = find_line_eqn(S_A, L, r_e, x_c3, y_c3, t3) ;



%% Check that solutions are real
in_workspace = false ;
angles_array = [q1(1), q2(1), q1(2) q2(2), q3, s1(1), s2(1), s1(2), s2(2), s3, t1(1), t2(1), t1(2), t2(2), t3] ;

if imag(angles_array) == zeros(1,15)
   in_workspace = true ;
end        
%% Display Solutions
% there are 2 solutions for each of the 3 arms 
% therefore 2^3 total orientations = 8 total
if in_workspace == true
    disp(" For a given position, there are 8 possible orientations: ")
    disp(" ")
%     pause(2)

    %Referring as arms being in orientation 1 or 2, we have: 
    disp("solution 1: ") 
    disp("orientation: 111")
    disp("***********")
    disp("Arm 1 (q1, q2, q3): ")
    disp( q1(1) *180/pi+ ", " + q2(1) *180/pi + ", " + q3 *180/pi)
    disp(" ")
    disp("Arm 2 (s1, s2, s3): ")
    disp( s1(1) *180/pi+ ", " + s2(1) *180/pi + ", " + s3 *180/pi)
    disp(" ")
    disp("Arm 3 (t1, t2, t3): ")
    disp( t1(1) *180/pi+ ", " + t2(1) *180/pi + ", " + t3 *180/pi)
    disp("***********")
    disp(" ")
%     pause(2)
    
    
    disp("solution 2: ") 
    disp("orientation: 222")
    disp("***********")
    disp("Arm 1 (q1, q2, q3): ")
    disp( q1(2) *180/pi+ ", " + q2(2) *180/pi + ", " + q3 *180/pi)
    disp(" ")
    disp("Arm 2 (s1, s2, s3): ")
    disp( s1(2) *180/pi+ ", " + s2(2) *180/pi + ", " + s3 *180/pi)
    disp(" ")
    disp("Arm 3 (t1, t2, t3): ")
    disp( t1(2) *180/pi+ ", " + t2(2) *180/pi + ", " + t3 *180/pi)
    disp("***********")
    disp(" ")
%     pause(2)
    
    
    disp("solution 3: ") 
    disp("orientation: 121")
    disp("***********")
    disp("Arm 1 (q1, q2, q3): ")
    disp( q1(1) *180/pi+ ", " + q2(1) *180/pi + ", " + q3 *180/pi)
    disp(" ")
    disp("Arm 2 (s1, s2, s3): ")
    disp( s1(2) *180/pi+ ", " + s2(2) *180/pi + ", " + s3 *180/pi)
    disp(" ")
    disp("Arm 3 (t1, t2, t3): ")
    disp( t1(1) *180/pi+ ", " + t2(1) *180/pi + ", " + t3 *180/pi)
    disp("***********")
    disp(" ")
%     pause(2)
    
    disp("solution 4: ") 
    disp("orientation: 112")
    disp("***********")
    disp("Arm 1 (q1, q2, q3): ")
    disp( q1(1) *180/pi+ ", " + q2(1) *180/pi + ", " + q3 *180/pi)
    disp(" ")
    disp("Arm 2 (s1, s2, s3): ")
    disp( s1(1) *180/pi+ ", " + s2(1) *180/pi + ", " + s3 *180/pi)
    disp(" ")
    disp("Arm 3 (t1, t2, t3): ")
    disp( t1(2) *180/pi+ ", " + t2(2) *180/pi + ", " + t3 *180/pi)
    disp("***********")
    disp(" ")
%     pause(2)
    
    disp("solution 5: ") 
    disp("orientation: 211")
    disp("***********")
    disp("Arm 1 (q1, q2, q3): ")
    disp( q1(2) *180/pi+ ", " + q2(2) *180/pi + ", " + q3 *180/pi)
    disp(" ")
    disp("Arm 2 (s1, s2, s3): ")
    disp( s1(1) *180/pi+ ", " + s2(1) *180/pi + ", " + s3 *180/pi)
    disp(" ")
    disp("Arm 3 (t1, t2, t3): ")
    disp( t1(1) *180/pi+ ", " + t2(1) *180/pi + ", " + t3 *180/pi)
    disp("***********")
    disp(" ")
%     pause(2)
    
    disp("solution 6: ") 
    disp("orientation: 221")
    disp("***********")
    disp("Arm 1 (q1, q2, q3): ")
    disp( q1(2) *180/pi+ ", " + q2(2) *180/pi + ", " + q3 *180/pi)
    disp(" ")
    disp("Arm 2 (s1, s2, s3): ")
    disp( s1(2) *180/pi+ ", " + s2(2) *180/pi + ", " + s3 *180/pi)
    disp(" ")
    disp("Arm 3 (t1, t2, t3): ")
    disp( t1(1) *180/pi+ ", " + t2(1) *180/pi + ", " + t3 *180/pi)
    disp("***********")
    disp(" ")
%     pause(2)
    
    disp("solution 7: ") 
    disp("orientation: 212")
    disp("***********")
    disp("Arm 1 (q1, q2, q3): ")
    disp( q1(2) *180/pi+ ", " + q2(2) *180/pi + ", " + q3 *180/pi)
    disp(" ")
    disp("Arm 2 (s1, s2, s3): ")
    disp( s1(1) *180/pi+ ", " + s2(1) *180/pi + ", " + s3 *180/pi)
    disp(" ")
    disp("Arm 3 (t1, t2, t3): ")
    disp( t1(2) *180/pi+ ", " + t2(2) *180/pi + ", " + t3 *180/pi)
    disp("***********")
    disp(" ")
%     pause(2)
    
    disp("solution 8: ") 
    disp("orientation: 122")
    disp("***********")
    disp("Arm 1 (q1, q2, q3): ")
    disp( q1(1) *180/pi+ ", " + q2(1) *180/pi + ", " + q3 *180/pi)
    disp(" ")
    disp("Arm 2 (s1, s2, s3): ")
    disp( s1(2) *180/pi+ ", " + s2(2) *180/pi + ", " + s3 *180/pi)
    disp(" ")
    disp("Arm 3 (t1, t2, t3): ")
    disp( t1(2) *180/pi+ ", " + t2(2) *180/pi + ", " + t3 *180/pi)
    disp("***********")
    disp(" ")

else 
    disp("Chosen Position is outside of robot workspace.")
end


%% Plot the position of the robot

figure(1) % orientation 1
% plot the outer triangle
% bottom left, bottom right, top
tri_x = [0, sqrt(3)*r_b, (1/2)*sqrt(3)*r_b, 0] ;
tri_y = [0, 0, (3/2)*r_b, 0] ;
plot(tri_x,tri_y,'ro-','Linewidth',2)
hold on


% Plot arm 1
xj1 = 0 ;
xj2 = xj1 + S_A*cos(q1(1)) ;
xj3 = xj2 + L*cos(q2(1)) ;
arm1_x = [xj1, xj2, xj3] ;

yj1 = 0 ;
yj2 = yj1 + S_A*sin(q1(1)) ;
yj3 = yj2 + L*sin(q2(1)) ;
arm1_y = [yj1, yj2, yj3] ;

plot(arm1_x, arm1_y, 'co-','Linewidth',2)
hold on


% Plot arm 2
xj1 = sqrt(3)*r_b ;
xj2 = xj1 + S_A*cos(s1(1)) ;
xj3 = xj2 + L*cos(s2(1)) ;
arm2_x = [xj1, xj2, xj3] ;

yj1 = 0 ;
yj2 = yj1 + S_A*sin(s1(1)) ;
yj3 = yj2 + L*sin(s2(1)) ;
arm2_y = [yj1, yj2, yj3] ;

plot(arm2_x, arm2_y, 'co-','Linewidth',2)
hold on

% Plot arm 3
xj1 = (1/2)*sqrt(3)*r_b ;
xj2 = xj1 + S_A*cos(t1(1)) ;
xj3 = xj2 + L*cos(t2(1)) ;
arm3_x = [xj1, xj2, xj3] ;

yj1 = (3/2)*r_b ;
yj2 = yj1 + S_A*sin(t1(1)) ;
yj3 = yj2 + L*sin(t2(1)) ;
arm3_y = [yj1, yj2, yj3] ;

plot(arm3_x, arm3_y, 'co-','Linewidth',2)
hold on

%plot inner triangle
inner_triangle_x = [arm1_x(3), arm2_x(3), arm3_x(3), arm1_x(3)] ;
inner_triangle_y = [arm1_y(3), arm2_y(3), arm3_y(3), arm1_y(3)] ;
plot(inner_triangle_x, inner_triangle_y, 'bo-','Linewidth',2)
hold on

axis([ -0.1 0.6 -0.1 0.6])
title('Parallel Robot First Orientation for a = pi/6') ; xlabel('x (m)') ; ylabel('y (m)');
subtitle('End Effector at x = 0.25, y = 0.21');



%**************************************************
figure(2) % orientation 2
% plot the outer triangle
% bottom left, bottom right, top
tri_x = [0, sqrt(3)*r_b, (1/2)*sqrt(3)*r_b, 0] ;
tri_y = [0, 0, (3/2)*r_b, 0] ;
plot(tri_x,tri_y,'ro-','Linewidth',2)
hold on


% Plot arm 1
xj1 = 0 ;
xj2 = xj1 + S_A*cos(q1(2)) ;
xj3 = xj2 + L*cos(q2(2)) ;
arm1_x = [xj1, xj2, xj3] ;

yj1 = 0 ;
yj2 = yj1 + S_A*sin(q1(2)) ;
yj3 = yj2 + L*sin(q2(2)) ;
arm1_y = [yj1, yj2, yj3] ;

plot(arm1_x, arm1_y, 'co-','Linewidth',2)
hold on


% Plot arm 2
xj1 = sqrt(3)*r_b ;
xj2 = xj1 + S_A*cos(s1(2)) ;
xj3 = xj2 + L*cos(s2(2)) ;
arm2_x = [xj1, xj2, xj3] ;

yj1 = 0 ;
yj2 = yj1 + S_A*sin(s1(2)) ;
yj3 = yj2 + L*sin(s2(2)) ;
arm2_y = [yj1, yj2, yj3] ;

plot(arm2_x, arm2_y, 'co-','Linewidth',2)
hold on

% Plot arm 3
xj1 = (1/2)*sqrt(3)*r_b ;
xj2 = xj1 + S_A*cos(t1(2)) ;
xj3 = xj2 + L*cos(t2(2)) ;
arm3_x = [xj1, xj2, xj3] ;

yj1 = (3/2)*r_b ;
yj2 = yj1 + S_A*sin(t1(2)) ;
yj3 = yj2 + L*sin(t2(2)) ;
arm3_y = [yj1, yj2, yj3] ;

plot(arm3_x, arm3_y, 'co-','Linewidth',2)
hold on

%plot inner triangle
inner_triangle_x = [arm1_x(3), arm2_x(3), arm3_x(3), arm1_x(3)] ;
inner_triangle_y = [arm1_y(3), arm2_y(3), arm3_y(3), arm1_y(3)] ;
plot(inner_triangle_x, inner_triangle_y, 'bo-','Linewidth',2)
hold on

axis([ -0.1 0.6 -0.1 0.6])
title('Parallel Robot Second Orientation for a = pi/6') ; xlabel('x (m)') ; ylabel('y (m)');
subtitle('End Effector at x = 0.25, y = 0.21');
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