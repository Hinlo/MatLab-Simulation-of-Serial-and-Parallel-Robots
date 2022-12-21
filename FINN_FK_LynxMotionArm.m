%% lynxmotion arm has 5 dof!
% NOTE: Defined angles start from x = 0, rotating +ve or -ve direction, about the z
% axis always!!
%#ok<*NOPTS>  do not pester me about semi colons
%#ok<*SAGROW> do not pester me about lists
% click line numbers to enforce a pause at that line during run. 
% remember, cos(x) is radians, cosd(x) would be degrees. 
% remember, arrays count from 1 not 0 in matlab.
% clear the output
clear all %#ok<*CLALL> 
close all
clc

%% DH TABLE
% a    alpha   d   theta 
% 0    90      d1  q1
% l1   0       0   q2
% l2   0       0   q3
% 0    90      0   q4
% 0    0       l3  q5
%% Set our variables
syms d1 l1 l2 l3; % length variables
syms q1 q2 q3 q4 q5; % angle variables
%% Show our 5 tranformation matrices
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

%% Multiply them in the required order (right to left)
% You can show or comment out the components.

% uncomment to see step by step multiplication of T matrices.
% T_34*T_45
% T_23*T_34*T_45;
% T_12*T_23*T_34*T_45;
T_e = T_01*T_12*T_23*T_34*T_45;

% Show our final transformation matrix
T_e 

% now get a specified result, for example: 

T_e_Specified = subs(T_e, [q1, q2, q3, q4, q5, l1, l2, l3, d1], [0*(pi/180),90*(pi/180),0*(pi/180), 0*(pi/180), 0*(pi/180), 1, 1, 1, 1])

%% The following code simulates the forward kinematics of a the lynxmotion 
% arm using my forward kinematics calculated above.
% Figure 2 shows its movement from start to end
% position, Figure 1 shows the location of its end effector at points of
% its trajectory and Figure 3 shows the maximum potential workspace of the
% arm's end effector.

disp('The following code simulates the forward kinematics of a simple 2DOF')
disp('serial manipulator. Figure 2 shows ts movement from start to end position,')
disp('Figure 1 shows the location of its end effector at points of its trajectory')
disp('and Figure 3 shows the maximum potential workspace of its end effector')

%% A series of joint angles
% The following variables are defined in the form of column-vectors with
% 4 rows each. Each row represents a different position (angle) of the joint.
% e.g. inititally we hae 60 degrees for q1 and -30 for q2.
% USING _set TO MARK WHEN WE HAVE DEFINED OUR SYMBOLIC VARIABLES.
q1_set = [ 0 20 40 45 ]'*pi/180 ; % base can spin full 360 degrees
q2_set = [ 0 30 60 90]'*pi/180 ; % first p joint can move 0 to 180
q3_set = [ 0 10 20 30 ]'*pi/180 ; % second p joint can move -180 to 180
q4_set = [ 0 -10 -20 -30]'*pi/180 ; % third p joint can move -180 to 180
q5_set = [ 0 90 180 30 ]'*pi/180 ; % claw can spin full 360 degrees


%% Links Lengths
d1_set = 0.1 ; % example lengths, start with all as 10cm
l1_set = 0.1 ;
l2_set = 0.1 ;
l3_set = 0.1 ;

%% Trigonometric abbreviations
% not required at the moment but see orginal uwe script if you want to add
% them.


%% Tip position
% These equations are taken from our general T_e matrix above.

% CHECK: I think these are going to be the x,y,z translation values from
% T_E and ignore the rot values for orientation of the end effector.
xt_element = T_e(1,4); % the x transformation is the 4th element of the 1st row of our general transformation matrix, T_e. 
yt_element = T_e(2,4); % the y transformation is the 4th element of the 2nd row of our general transformation matrix, T_e. 
zt_element = T_e(3,4); % the z transformation is the 4th element of the 3rd row of our general transformation matrix, T_e. 

% to add the angles and lengths, we will use a quick loop.
% lengths are just one value, but angles have an array, so we'll loop
% through the angle arrays.
xt = [];
for i = 1:length(q1_set)
    xt_value = subs(xt_element,[d1 l1 l2 l3 q1 q2 q3 q4 q5], [d1_set l1_set l2_set l3_set q1_set(i) q2_set(i) q3_set(i) q4_set(i) q5_set(i)]); % substitute in the set values.
    xt(i,1) = xt_value ;
end



% repeat for y
yt = [];
for i = 1:length(q1_set)
    yt_value = subs(yt_element,[d1 l1 l2 l3 q1 q2 q3 q4 q5], [d1_set l1_set l2_set l3_set q1_set(i) q2_set(i) q3_set(i) q4_set(i) q5_set(i)]); % substitute in the set values.
    yt(i,1) = yt_value ;
end



% repeat for z
zt = [];
for i = 1:length(q1_set)
    zt_value = subs(zt_element,[d1 l1 l2 l3 q1 q2 q3 q4 q5], [d1_set l1_set l2_set l3_set q1_set(i) q2_set(i) q3_set(i) q4_set(i) q5_set(i)]); % substitute in the set values.
    zt(i,1) = zt_value ;
end

% pt is a 4 by 3 double as required for the formatting below. 
pt = [ xt yt zt ] ;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plot the trajectory of the end-effector
figure (1)
%line below just dictates where the image appears on the screen in matlab
%desktop
set(1,'position',[680 558 560 420])



% IN 3D...
plot3(pt(1,1), pt(1,2), pt(1,3), 'ro')       % plot the first position of the robot's end effector
hold on
plot3(pt(2:4,1),pt(2:4,2),pt(2:4,3), 'o')       % plot the 3 following positions of the robot's end effector
title('Tip Trajectory') ; xlabel('x (m)') ; ylabel('y (m)') ; zlabel('z (m)');


%% Plot the robotic arm, in 4 different positions
% this currently just moves q1, the base. Do we want to rotate more than
% one joint? We defo don't want to rotate just the base.
figure (2) 
%line below just dictates where the image appears on the screen in matlab
%desktop
set(2,'position',[116 190 560 420])

%base is at origin
base = 0 ;


% next joint given by T01 translations (movement from base to joint 1)
% need to make into a 4 by 3 array.
Tj1_x = zeros(4,1) ; % x translation is zero
Tj1_y = zeros(4,1) ; % y translation is zero
Tj1_z = subs(zeros(4,1), 0, d1_set);  % z translation is d1

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
    xt_value = subs(xt_element, [l1 d1 q1 q2], [l1_set d1_set q1_set(i) q2_set(i)]) ;
    yt_value = subs(yt_element, [l1 d1 q1 q2], [l1_set d1_set q1_set(i) q2_set(i)]) ;
    zt_value = subs(zt_element, [l1 d1 q1 q2], [l1_set d1_set q1_set(i) q2_set(i)]) ;
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
    xt_value = subs(xt_element,[d1 l1 l2 q1 q2 q3], [d1_set l1_set l2_set q1_set(i) q2_set(i) q3_set(i)]) ;
    yt_value = subs(yt_element,[d1 l1 l2 q1 q2 q3], [d1_set l1_set l2_set q1_set(i) q2_set(i) q3_set(i)]) ;
    zt_value = subs(zt_element,[d1 l1 l2 q1 q2 q3], [d1_set l1_set l2_set q1_set(i) q2_set(i) q3_set(i)]) ;
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
    xt_value = subs(xt_element,[d1 l1 l2 l3 q1 q2 q3 q4], [d1_set l1_set l2_set l3_set q1_set(i) q2_set(i) q3_set(i) q4_set(i)]) ;
    yt_value = subs(yt_element,[d1 l1 l2 l3 q1 q2 q3 q4], [d1_set l1_set l2_set l3_set q1_set(i) q2_set(i) q3_set(i) q4_set(i)]) ;
    zt_value = subs(zt_element,[d1 l1 l2 l3 q1 q2 q3 q4], [d1_set l1_set l2_set l3_set q1_set(i) q2_set(i) q3_set(i) q4_set(i)]) ;
    Tj4_x(i,1) = xt_value ;
    Tj4_y(i,1) = yt_value ;
    Tj4_z(i,1) = zt_value ;
end

% We already calculated our end effector positions - that's the pt array.





    % generate the graph. Note: only 4 joints will be visible as there is
    % no spatial distinction between joins three and 4.
for i = 1:4 % zeros is the base position, doesn't change.
    xx = [base; Tj1_x(i); Tj2_x(i); Tj3_x(i); Tj4_x(i); pt(i,1) ] ;
    yy = [base; Tj1_y(i); Tj2_y(i); Tj3_y(i); Tj4_y(i); pt(i,2) ] ;
    zz = [base; Tj1_z(i); Tj2_z(i); Tj3_z(i); Tj4_z(i); pt(i,3) ] ;
    
    pause(1) % pause long enough that we see the first position
    plot3(xx,yy,zz,'ko-','Linewidth',2)
    axis equal
    hold on
    
    % label axes, start point, end point.
    xlabel('x (m)') ; ylabel('y (m)') ; zlabel('z (m)') 
    text(pt(1,1),pt(1,2),pt(1,3), 'x') ; text(pt(1,1) + 0.002,pt(1,2) + 0.002,pt(1,3) + 0.002,'ptStart') ;
    text(pt(4,1),pt(4,2),pt(4,3), 'x') ; text(pt(4,1) + 0.002,pt(4,2) + 0.002,pt(4,3) + 0.002,'ptEnd') ;
    axis([ -0.1 0.4 -0.1 0.4 0 0.4 ])
    hold off % CHANGE TO HOLD ON TO SEE ALL LINES AT ONCE.
    pause(2)
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Workspace - once working, set to finer increments to get workspace plot.

% Here we must extend the range of the set angles to encompass all possible
% angles.

% SETTING ARRAYS: start:interval:end. x = 1:2:7 -> x = [1,3,5,7]
% to be able to plot, keep intervals consistant.
q1_set = (0:30:360)*pi/180  ;% base angle can go 0 to 360, using intervals of 5 atm.
q2_set = (-90:30:90)*pi/180 ; % joint has range -90 to 90 
q3_set = (-180:30:180)*pi/180 ; % joint has range -180 to 180
q4_set = (-180:30:180)*pi/180 ; % joint has range -180 to 180
q5_set = 0; % rot of end effector has range 0 to 360 but this doesn't 
% affect workspace.
% we don't care about q5 as workspace is about how far the robot
% can reach, the final orientation of the end effector doesn't matter
% therefore, set q5 as a constant.

%% Angles Full Range of motion
% Not realistic but worth knowing

% q1_set = 0:30:360 ; % base angle can go 0 to 360, using intervals of 5 atm.
% q2_set = -90:30:90 ; % joint has range -90 to 90 
% q3_set = -180:30:180 ; % joint has range -180 to 180
% q4_set = -180:30:180 ; % joint has range -180 to 180
% q5_set = 0; % rot of end effector has range 0 to 360 but this doesn't 
%% Plot the workspace of the robot
figure (3)
%line below just dictates where the image appears on the screen in matlab
%desktop
set(3,'position',[1243 190 560 420])


for i = 1:length(q1_set)	% for q1
    for j = 1:length(q2_set)  % for q2
        for k = 1:length(q3_set) % for q3
            for l = 1:length(q4_set) % for q4
                % define our x y z values for the given q 1-4 values
                xwork = subs(xt_element,[d1 l1 l2 l3 q1 q2 q3 q4 q5], [d1_set l1_set l2_set l3_set q1_set(1,i) q2_set(1,j) q3_set(1,k) q4_set(1,l) q5_set]); % substitute in the set values.
                ywork = subs(yt_element,[d1 l1 l2 l3 q1 q2 q3 q4 q5], [d1_set l1_set l2_set l3_set q1_set(1,i) q2_set(1,j) q3_set(1,k) q4_set(1,l) q5_set]); % we still want our end effector positions
                zwork = subs(zt_element,[d1 l1 l2 l3 q1 q2 q3 q4 q5], [d1_set l1_set l2_set l3_set q1_set(1,i) q2_set(1,j) q3_set(1,k) q4_set(1,l) q5_set]); % given by subbing into the element eqns.
                % plot this point!
                if zwork < 0 || abs( q2_set(1,j) + q3_set(1,k) + q4_set(1,l) ) > 2*pi % skip plotting this point if the z value is less than zero, e.g arm goes into the table. or if psi greater than 360
                   continue
                end
%                 disp("position set: " + num2str(i)  + ", point : " + num2str(j) + num2str(k) + num2str(l) )
%                 disp("x: " + double(xwork) + ", y: " +double(ywork) + ", z: " + double(zwork) + ", psi: " + round((q2_set(1,j) + q3_set(1,k) + q4_set(1,l)),2,"decimals") + ", mu: " + q5_set*180/pi)
                plot3(xwork,ywork,zwork,'x');
                hold on
            end
        end
    end
        % this can take some time, so display current percentage
        % done while you wait.
        percent_done = (i*j*k*l)/(length(q1_set)*length(q2_set)*length(q3_set)*length(q4_set))*100
end

title('Workspace') ; xlabel('x (m)') ; ylabel('y (m)') ; zlabel('z (m)');