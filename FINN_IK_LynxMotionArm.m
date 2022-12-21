%% Inverse Kinematics Calculator for LynxMotion Arm
% key is to maintain list order across lists of each angle.
clear all %#ok<*CLALL> %#ok<*SAGROW> 
close all
clc

%% Links Length
d1 = 0.1 ;
l1 = 0.1 ;
l2 = 0.1 ;
l3 = 0.1 ;

%% Desired position of end-effector - SET ME

% Cartesian Coords for x,y,z values.
px = 0.17;
py = 0.17 ;
pz = 0.17;

% mu and psi angles for orientation
mu = 0; % mu is rotation of end effector relative to "wrist"

% list of all possible psi values. they will be narrowed down later to only
% real values. technically could go negative angle but it just ends up
% repeating. e.g +90 is same as -270 degrees logically.
psi = 0:pi/6:2*pi; % RADIANS 
psi = psi - pi/2; % angular correction to line up with zero point of FK.

% psi is angle between end effector and negative-z-axis, based on FK.
% psi = q2 + q3 + q4
% End effector desired position and orientation given by array End_Effector
End_Effector = [ px py pz ]' ;
% disp('Desired position =')
% disp("x,y,z")
% disp(End_Effector)
% disp("mu")
% disp(mu)
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
    D(i) = round(D(i), 7) ; % rounding D avoids fake imaginary numbers due to rounding
                    % errors.
end
%% MAKE LISTS FOR EACH ANGLE AND ENSURE ALL THE SAME LENGTH. IF ONE OF THE ANGLES IN A SET OF 5 IS IMAGINARY, REMOVE THAT INDEX FOR EVERY ANGLE. SHOULD START WITH MANY BUT REDUCE TO ONLY A FEW.
%% get only real values

psi_real = [] ;
r_w_real = [] ;
z_w_real = [] ;
D_real = [] ;
for i = 1:length(psi)
    if imag(sqrt(1-D(i)^2)) == 0  % for real values, append to real lists.
        psi_real(end + 1) = round(psi(i),7) ; % round each list to get zeros rather than e^-10 or something.
        r_w_real(end + 1) = round(r_w(i),7) ;
        z_w_real(end + 1) = round(z_w(i),7) ;
        D_real(end + 1)= round(D(i),7) ;
    end
end

% we can see these four lists MUST have same length.
% disp(psi_real)
% disp(D_real)
% disp(z_w_real)
% disp(r_w_real)

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
    % for each r_w,z_w value, find first q2 value using first q3 value
    q2(end+1) = atan2(z_w_real(i),r_w_real(i)) + atan2(l2*sin(q3(2*i)), l1+l2*cos(q3(2*i))) ;
    % add +1 to 
    q2(end+1) = atan2(z_w_real(i),r_w_real(i)) + atan2(l2*sin(q3(2*i-1)), l1+l2*cos(q3(2*i-1))) ;
end
%  disp("q2")
%  disp(q2)
%% Find q4 

% two options based on two sets of q2,q3
% use psi = q2 + q3 + q4
% +pi/2 is the geometry correction for zero point of q4
for i = 1:length(D_real) % round to remove zero discrepancies.
    q4(end+1) = psi_real(i) - q2(2*i -1) - q3(2*i -1) +pi/2 ;
    q4(end+1) = psi_real(i) - q2(2*i) - q3(2*i) +pi/2 ;
end


%  disp("q4")
%  disp(q4)
%% Remove duplicate values
% make a matrix out of the lists where one row is one solution

% we know matrix will have initial size: no. of items in q2, 3 or 4 x 5
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
Unique_Solutions = unique(Solution_Matrix,"rows","stable");
% stable prevents order being changed.


%% we also know, q4 can't be more than 360 degrees. filter these out.
valid_Solutions = [] ;
for i = 1:size(Unique_Solutions,1)
    if abs(Unique_Solutions(i,4)) < 2*pi
        valid_Solutions = [valid_Solutions; Unique_Solutions(i,:)] ; 
    end
end
%% Display Solutions

for i = 1:size(valid_Solutions,1) % for no. of rows aka no. of solutions
    
    disp("solution " + num2str(i) + ": ")
    disp(round(valid_Solutions(i,:)*180/pi,2,"decimals"))

end
