%% General Information:
%{ 
Dobot Simulation with Geometric approach and the DH Apporach based on the
simplified model.

Created as part of:
Malla, O. and Shanmugavel, M. (2024), 
"Simplified model to study the kinematics of manipulators with parallelogram linkages", 
Industrial Robot, Vol. 51 No. 5, pp. 704-714. 
https://doi.org/10.1108/IR-01-2024-0046

The code requires the use of Peter Corke's Robotics Toolbox for Matlab:
https://petercorke.com/toolboxes/robotics-toolbox/

THe code contains:
1) Robot Specs as per the real robot.
2) Geometric Kinematic solution.
3) Simplified model with DH parametrization.
4) Comparison to the vlaues captured from Dobot Studio for
joints rotations across the workspace.
5) Test cases captured from the robot's hardware.
6) Error and Workspace Plotting.
7) Functions to calculate Inverse Kinematics analytically and numerically.
%}
%% Robot Specs (Dimensions are in mm):

% L1 = 133; % length of link 1
L1 = 0; % To match the Dobot Studio L1 = 0.
L2 = 135; % length of link 2
L3 = 147; % length of link 3
L4 = 59.7;  % length of link 4. L4 = 61 To match with RoboDk Model.

fprintf('Dobot Magician: \n\n');
fprintf('Link lengths " mm ":\n %1.2f , %2.2f , %3.2f , %4.2f: \n \n', L1, L2, L3, L4);

% Joints' limits:
qmin =  [-90 , 0 , -10 , -90];
qmax =  [+90 , 85, +95 , +90];

% Joint Limits Radians:
theta1_min = deg2rad(qmin(1)); % minimum joint angle 1
theta1_max = deg2rad(qmax(1)); % maximum joint angle 1
theta2_min = deg2rad(qmin(2)); % minimum joint angle 2
theta2_max = deg2rad(qmax(2)); % maximum joint angle 2
theta3_min = deg2rad(qmin(3)); % minimum joint angle 3
theta3_max = deg2rad(qmax(3)); % maximum joint angle 3
theta4_min = deg2rad(qmin(4)); % minimum joint angle 4
theta4_max = deg2rad(qmax(4)); % maximum joint angle 4

% Defining robot base relative to workspace origin point "Camera":
Tbase1 = [1 0 0 0; 0 1 0 0; 0 0 1 -L1; 0 0 0 1];
fprintf('Robot 1 base transformation from workspace base point: \n');
disp(Tbase1);

% Discretizing the joint rotations:
SampleRate = 0.1;
% for n1: 180/SampelRate = 1800 points + 1 to consider boundery points
n1 = ((qmax(1)-qmin(1))/SampleRate)+1; 
n2 = ((qmax(2)-qmin(2))/SampleRate)+1;
n3 = ((qmax(3)-qmin(3))/SampleRate)+1;
n3 = 1001;
n4 = ((qmax(4)-qmin(4))/SampleRate)+1;

q1 = linspace(theta1_min,theta1_max,n1);
q2 = linspace(theta2_min,theta2_max,n2);
q3 = linspace(theta3_min,theta3_max,n3);
q4 = linspace(theta4_min,theta4_max,n4);

%% Another Sample rate for two joints rotation:
SampleRate2 = 1;
n2_2 = ((qmax(2)-qmin(2))/SampleRate2)+1;
n3_2 = ((qmax(3)-qmin(3))/SampleRate2)+1;

q2_2 = linspace(theta2_min,theta2_max,n2_2);
q3_2 = linspace(theta3_min,theta2_max,n3_2);

q2_2deg = linspace(qmin(2),qmax(2),n2_2);
q3_2deg = linspace(qmin(3),qmax(3),n3_2);
%% THE GEOMETRIC APPROACH:
% Plotting X, Y, Z with possible Joint 1 rotations

clear t1 t2 t3 theta1 theta2 theta3 i ii

t2 = 0;
t3 = 0;
theta2 = deg2rad(t2);
theta3 = deg2rad(t3);
Poses1 = zeros(n1,3);
ii = 0;

for i = qmin(1):SampleRate:qmax(1)
    ii = ii+1;

    t1 = i;
    theta1 = deg2rad(t1);
    r= L2 * sin(theta2) + L3 * cos(theta3) + L4;
    Poses1(ii,1) = cos(theta1) * r;
    Poses1(ii,2) = sin(theta1) * r;
    Poses1(ii,3) = L1 + L2 * cos(theta2) - L3 * sin(theta3);
end

% Plotting the results for joint 1:
figure;
hold on;
h1 = plot( rad2deg(q1),Poses1(1:n1,1) , '-*', 'color' , [0.8 0.5 0.4],'LineWidth',1.2, 'MarkerIndices', 1:200:length(q1));
h2 = plot( rad2deg(q1),Poses1(1:n1,2) , '-o', 'color' , [0.4 0.8 0.4],'LineWidth',1.2, 'MarkerIndices', 1:200:length(q1));
h3 = plot( rad2deg(q1),Poses1(1:n1,3) , '-s', 'color' , [0.5 0.4 0.8],'LineWidth',1.2, 'MarkerIndices', 1:200:length(q1));
% Add legends to the plot:
legend([h1, h2, h3], {'X', 'Y', 'Z'},'Location', 'southeast');
title('Plot of X, Y, and Z vs. Joint1 - Geometric');
xlabel('Joint1');
ylabel('X, Y, and Z');

% Plotting X, Y, Z with possible Joints variables

clear t1 t2 t3 theta1 theta2 theta3 i ii

t1 = 0;
t3 = 50;
theta1 = deg2rad(t1);
theta3 = deg2rad(t3);
Poses2 = zeros(n2,3);
ii = 0;

for i = qmin(2):SampleRate:qmax(2)
    ii = ii+1;

    t2 = i;
    theta2 = deg2rad(t2);
    r= L2 * sin(theta2) + L3 * cos(theta3) + L4;
    Poses2(ii,1) = cos(theta1) * r;
    Poses2(ii,2) = sin(theta1) * r;
    Poses2(ii,3) = L1 + L2 * cos(theta2) - L3 * sin(theta3);
end

% Plotting the results for joint 2:
figure;
hold on;
h1 = plot( rad2deg(q2),Poses2(1:n2,1) , '-*', 'color' , [0.8 0.5 0.4],'LineWidth',1.2, 'MarkerIndices', 1:200:length(q2));
h2 = plot( rad2deg(q2),Poses2(1:n2,2) , '-o', 'color' , [0.4 0.8 0.4],'LineWidth',1.2, 'MarkerIndices', 1:200:length(q2));
h3 = plot( rad2deg(q2),Poses2(1:n2,3) , '-s', 'color' , [0.5 0.4 0.8],'LineWidth',1.2, 'MarkerIndices', 1:200:length(q2));
legend([h1, h2, h3], {'X', 'Y', 'Z'},'Location', 'southeast');
title('Plot of X, Y, and Z vs. Joint2 - Geometric');
xlabel('Joint2');
ylabel('X, Y, and Z');


% Plotting X, Y, Z with possible Joint 3 rotations

clear t1 t2 t3 theta1 theta2 theta3 i ii

t1 = 0;
t2 = 45;
theta1 = deg2rad(t1);
theta2 = deg2rad(t2);
Poses3 = zeros(n3,3);
ii = 0;

for i = qmin(3):SampleRate:qmax(3)
    ii = ii+1;

    t3 = i;
    theta3 = deg2rad(t3);
    r= L2 * sin(theta2) + L3 * cos(theta3) + L4;
    Poses3(ii,1) = cos(theta1) * r;
    Poses3(ii,2) = sin(theta1) * r;
    Poses3(ii,3) = L1 + L2 * cos(theta2) - L3 * sin(theta3);
end

% Plotting the results for joint 3:
figure;
hold on;
h1 = plot( rad2deg(q3),Poses3(1:n3,1) , '-*', 'color' , [0.8 0.5 0.4],'LineWidth',1.2, 'MarkerIndices', 1:200:length(q3));
h2 = plot( rad2deg(q3),Poses3(1:n3,2) , '-o', 'color' , [0.4 0.8 0.4],'LineWidth',1.2, 'MarkerIndices', 1:200:length(q3));
h3 = plot( rad2deg(q3),Poses3(1:n3,3) , '-s', 'color' , [0.5 0.4 0.8],'LineWidth',1.2, 'MarkerIndices', 1:200:length(q3));
% Add legends to the plot:
legend([h1, h2, h3], {'X', 'Y', 'Z'},'Location', 'southeast');
title('Plot of X, Y, and Z vs. Joint3 - Geometric');
xlabel('Joint3');
ylabel('X, Y, and Z');


%% Orientation Calculation for the Geomtric Approach:
t1 = 0;
t4 = 0;
theta1 = deg2rad(t1);
theta4 = deg2rad(t4);
Rbase2end = trotz(theta4) * trotz(theta1) * trotx(pi) * Tbase1;
display(Rbase2end);


%% Geometric WorkSpace: computing end-effector positions for all joint angle combinations:
n = 30;
% Discretize joint and object "as link" angles:
theta1_range = linspace(theta1_min, theta1_max, n);
theta2_range = linspace(theta2_min, theta2_max, n);
theta3_range = linspace(theta3_min, theta3_max, n);

fprintf('Calculating end-effectors reachable points... \n \n');
X1 = zeros(n^3, 1); % Three Links >> n^3
Y1 = zeros(n^3, 1);
Z1 = zeros(n^3, 1);

for i = 1:n
    for j = 1:n
        for k = 1:n

            % Robot 1 Angles of Rotation:
            theta1 = theta1_range(i);
            theta2 = theta2_range(j);
            theta3 = theta3_range(k);
            
            R = L2 * sin(theta2) + L3 * cos(theta3) + L4;
            % Calculate the end points for Robot 1:
            X1((i-1)*n^2+(j-1)*n+k) = cos(theta1) * R;
            Y1((i-1)*n^2+(j-1)*n+k) = sin(theta1) * R;
            Z1((i-1)*n^2+(j-1)*n+k) = L1 + L2 * cos(theta2) - L3 * sin(theta3);

        end
    end
end

A = [X1,Y1,Z1];

% Modifying the base of the robot to make the centroid of the workspace 
% in the center of the workspace (0,0,0)
% K-Means clsutering to find the centroid for the points of Robot 1:
[~, centroid1] = kmeans(A , 1);

Tbase1mod = [1 0 0 Tbase1(1,4)-centroid1(1); 0 1 0 Tbase1(2,4)-centroid1(2); 0 0 1 Tbase1(3,4)-centroid1(3); 0 0 0 1];
Tbase1 = Tbase1mod;

endEffector1_Points = [A, ones(size(X1))];
A = (Tbase1 * endEffector1_Points')';
A = [A(:,1),A(:,2),A(:,3)];

Base_Location1 = [Tbase1(1,4),Tbase1(2,4),Tbase1(3,4)];
fprintf('Calculating end-effectors reachable points finished! \n');

% K-Means clsutering to find the center of mass for the points of Robot 1:
[~, centroid1] = kmeans(A , 1);

% Boundary
% boundary constructs an alphaShape from the specified points 
% and then uses boundaryFacets to determine which points lie on the boundary.
Shrink_Factor = 0.8;
[k_A,vol_A] = boundary(A,Shrink_Factor);
display(vol_A);

%% Plotting the Workspace
% Plot with the modified base for manipulatpor 1:

f = figure;
% Create the slider

plot3(centroid1(1),centroid1(2),centroid1(3),"+","MarkerSize",15,"Color","black","LineWidth",2);
text(Base_Location1(1)-0.2,Base_Location1(2),Base_Location1(3),'<<Rob 1','Color',[0.3 0.5 0.6]);

hold on;

% scatter3(A(:,1),A(:,2),A(:,3),'filled','MarkerEdgeColor',[0.7 .5 .5],'MarkerFaceColor',[0.5 0.5 .7]);
% FaceAlpha = 0.9;

trisurf(k_A, A(:,1), A(:,2), A(:,3), 'FaceColor', [0.4 0.6 0.7], 'FaceAlpha', 0.5);
Shrink_Factor = 1;

slider = uicontrol('Parent', f, 'Style', 'slider', 'Min', 0, 'Max', 1, 'Value', Shrink_Factor, ...
    'Position', [20 20 200 20], 'Callback', {@sliderCallback, A});

% Create the text label for ShrinkFactor
slider_text = uicontrol('Parent', f, 'Style', 'text','String', 'Sf', ...
    'Position', [0 40 200 20],'BackgroundColor', get(f, 'Color'));

xlabel('X'); ylabel('Y'); zlabel('Z');
title("Dobot Magician Workspace with AlphaShape: Sf =  " + (Shrink_Factor));

axis equal;
hold off;


%% DH Method with PSEUDO JOINT:

% Link (theta, D: joint extension, a: joint offset, alpha: joint twist)
DobotM(1)= Link([0 , L1  , 0  , -pi/2]);
DobotM(2)= Link([0 ,  0  , L2 , 0    ]);
DobotM(3)= Link([0 ,  0  , L3 , 0    ]);
DobotM(4)= Link([0 ,  0  , L4 , -pi/2]);
DobotM(5)= Link([0 ,  0 , 0  , 0    ]);

DobotM(1).qlim = [theta1_min,theta1_max];
DobotM(2).offset = -pi/2;
DobotM(2).qlim = [theta2_min,theta2_max];
DobotM(3).offset = pi/2 - DobotM(2).theta;
DobotM(3).qlim = [-pi,pi/2];
DobotM(4).qlim = [-pi/2,pi]; % pseudo-joint
DobotM(5).qlim = [theta4_min,theta4_max];
DobotM(5).offset = pi;

Rob1 = SerialLink(DobotM,'name','Dobot');

figure;
Q = [0,deg2rad(0),deg2rad(0),deg2rad(0),0];
Rob1.plot(Q,'view',[40 20],'wrist','jaxes','arrow','jointcolor',[0.3,0.3,0.3],'linkcolor',[1,1,1],'tile1color',[0.9,0.9,0.9],'tile2color',[0.9,0.9,0.9]);
set(gca,'color',[0.5,0.6,0.7]);
%
Rob1.teach('approach');
Rob1.plot(Q,'view',[40 20],'wrist','jointcolor',[0.3,0.3,0.3],'linkcolor',[1,1,1],'tile1color',[0.9,0.9,0.9],'tile2color',[0.9,0.9,0.9]);

%% Plotting X, Y , Z vs Joints' Variables

ii1 = 1; ii2 = 1; ii3 = 1;

TR1 = zeros(4,4,n1);
TR2 = zeros(4,4,n2);
TR3 = zeros(4,4,n3);

for i = 1:length(q1)
   Q = [q1(i),0,0,0,deg2rad(-26.3737)];
   TR1(:,:,ii1) = Rob1.fkine(Q);
   ii1 = ii1+1;
end

for i = 1:length(q2)
   Q = [0,q2(i),deg2rad(50)-q2(i),-deg2rad(50),deg2rad(-0.0518)];
   TR2(:,:,ii2) = Rob1.fkine(Q);
   ii2 = ii2+1;
end

for i = 1:length(q3)
   Q = [0,deg2rad(45),q3(i)-deg2rad(45),-q3(i),deg2rad(-0.0518)];
   TR3(:,:,ii3) = Rob1.fkine(Q);
   ii3 = ii3+1;
end

A1 = round(transl(TR1),4);
A2 = round(transl(TR2),4);
A3 = round(transl(TR3),4);

% Plot for first Joint
figure;
hold on;
h1 = plot( rad2deg(q1),A1(1:n1,1) , '-*', 'color' , [0.8 0.5 0.4],'LineWidth',1.2, 'MarkerIndices', 1:200:length(A1(1:n1,1)));
h2 = plot( rad2deg(q1),A1(1:n1,2) , '-^', 'color' , [0.4 0.8 0.4],'LineWidth',1.2, 'MarkerIndices', 1:200:length(A1(1:n1,2)));
h3 = plot( rad2deg(q1),A1(1:n1,3) , '-s', 'color' , [0.5 0.4 0.8],'LineWidth',1.2, 'MarkerIndices', 1:200:length(A1(1:n1,3)));
h4 = plot( rad2deg(q1),(rad2deg(q1)-26.3737), '-o', 'color', [0.5 0.5 0.5], 'LineWidth',1.2, 'MarkerIndices', 1:200:length(A1(1:n1,3)));

% Add legends to the plot:
legend([h1, h2, h3, h4], {'X', 'Y', 'Z', 'O'},'Location', 'southeast');
title('Plot of X,Y,Z and R vs. Joint1 - Model');
xlabel('Joint1 (deg)');
ylabel('X, Y, Z (mm), and R (deg)');

% Plot for Second Joint
figure;
hold on;
h1 = plot( rad2deg(q2),A2(1:n2,1) , '-*', 'color' , [0.8 0.5 0.4],'LineWidth',1.2, 'MarkerIndices', 1:200:length(A2(1:n2,1)));
h2 = plot( rad2deg(q2),A2(1:n2,2) , '-^', 'color' , [0.4 0.8 0.4],'LineWidth',1.2, 'MarkerIndices', 1:200:length(A2(1:n2,2)));
h3 = plot( rad2deg(q2),A2(1:n2,3) , '-s', 'color' , [0.5 0.4 0.8],'LineWidth',1.2, 'MarkerIndices', 1:200:length(A2(1:n2,3)));
%h4 = plot( rad2deg(q2),-0.05, '-o', 'color', [0.4 0.7 0.6], 'LineWidth',1.2, 'MarkerIndices', 1:200:length(A1(1:1800,3)));

% Add legends to the plot:
legend([h1, h2, h3], {'X', 'Y', 'Z'},'Location', 'southeast');
title('Plot of X, Y, and Z vs. Joint2 - Model');
xlabel('Joint2 (deg)');
ylabel('X, Y, and Z (mm)');

% Plot for Third Joint
figure;
hold on;
h1 = plot( rad2deg(q3),A3(1:n3,1) , '-*', 'color' , [0.8 0.5 0.4],'LineWidth',1.2, 'MarkerIndices', 1:200:length(A3(1:n3,1)));
h2 = plot( rad2deg(q3),A3(1:n3,2) , '-^', 'color' , [0.4 0.8 0.4],'LineWidth',1.2, 'MarkerIndices', 1:200:length(A3(1:n3,2)));
h3 = plot( rad2deg(q3),A3(1:n3,3) , '-s', 'color' , [0.5 0.4 0.8],'LineWidth',1.2, 'MarkerIndices', 1:200:length(A3(1:n3,3)));

% Add legends to the plot:
legend([h1, h2, h3], {'X', 'Y', 'Z'},'Location', 'southeast');
title('Plot of X, Y, and Z vs. Joint3 - Model');
xlabel('Joint3 (deg)');
ylabel('X, Y, and Z (mm)');

%% Error Plotting

% Plot for first Joint
q1m = q1(2:length(q1));
q2m = q2(2:length(q2));
q3m = q3(2:length(q3));

%% Calculate Error Across all points of the simulation:

data1 = readtable('PosesJoint1.csv');
column_names = data1.Properties.VariableNames;
for i = 1:length(column_names)
    column_name = column_names{i};
    column_data = data1(:,column_name);
    assignin('base',column_name,column_data);
end

data2 = readtable('PosesJoint2.csv');
column_names = data2.Properties.VariableNames;
for i = 1:length(column_names)
    column_name = column_names{i};
    column_data = data2(:,column_name);
    assignin('base',column_name,column_data);
end

data3 = readtable('PosesJoint3.csv');
column_names = data3.Properties.VariableNames;
for i = 1:length(column_names)
    column_name = column_names{i};
    column_data = data3(:,column_name);
    assignin('base',column_name,column_data);
end
Data = [data1;data2;data3];

%% Adding columns for values calculated from the simulation:
x = 1;
Orientation = zeros(height(Data),1);

for i = 1:height(Data)
    Q = [deg2rad(Data{i,1}),deg2rad(Data{i,2}),deg2rad(Data{i,3})-deg2rad(Data{i,2}),deg2rad(-Data{i,3}),deg2rad(Data{i,4})];
    TR2(:,:,x) = Rob1.fkine(Q);
    Orientation(x,1) = Data{i,1}+Data{i,4};
    x = x+1;
end

% A2 = round(transl(TR2),4);

A2 = transl(TR2);

Data.Xdh = A2(:,1);
Data.Ydh = A2(:,2);
Data.Zdh = A2(:,3);
Data.Orientation = Orientation(:,1);
Data.Error = sqrt((Data.X-Data.Xdh).^2+(Data.Y-Data.Ydh).^2+(Data.Z-Data.Zdh).^2);
Data.ErrorX = Data.X-Data.Xdh;
Data.ErrorY = Data.Y-Data.Ydh;
Data.ErrorZ = Data.Z-Data.Zdh;
Data.OrientError = Data.R - Data.Orientation;

fprintf("Mean Position Error is %f \n",mean(Data.Error));
fprintf("Max Position Error is %f \n",max(Data.Error));
fprintf("Mean R Error is %f \n",mean(Data.OrientError));
fprintf("Max R Error is %f \n",max(Data.OrientError));

tester = Rob1.fkine(deg2rad([-44.5000000000000,0,0,0,-26.3737000000000]));

%% Error Histograms:
figure;
histfit(Data.Error(1:length(q1m)),12);
xtickformat('%.2f');
title('Positional Error and its Frequency for joint 1');
xlabel('Error Value (mm)');

figure;
histfit(Data.Error(1:length(q2m)),12);
xtickformat('%.2f');
title('Positional Error and its Frequency for joint 2');
xlabel('Error Value (mm)');

figure;
histfit(Data.Error(1:length(q3m)),12);
xtickformat('%.2f');
title('Positional Error and its Frequency for joint 3');
xlabel('Error Value (mm)');
%% ERROR PLOTTING:
figure;
h5 = plot( rad2deg(q1m), Data.Error(1:1800) , 'color' , [0.8 0.5 0.4],'LineWidth',1.2);
hold on
h6 = plot( rad2deg(q1m), Data.OrientError(1:1800) , 'color' , [0.4 0.8 0.4],'LineWidth',1.2);
ylim([0,0.001]);

figure;
h5 = plot( rad2deg(q2m), Data.Error(length(q1m)+1:length(q1m)+length(q2m)) , 'color' , [0.8 0.5 0.4],'LineWidth',1.2);
hold on
h6 = plot( rad2deg(q2m),Data.OrientError(length(q1m)+1:length(q1m)+length(q2m)) , 'color' , [0.4 0.8 0.4],'LineWidth',1.2);

figure;
h5 = plot( rad2deg(q3m), Data.Error(length(q1m)+length(q2m)+1:length(q1m)+length(q2m)+length(q3m)) , 'color' , [0.8 0.5 0.4],'LineWidth',1.2);
hold on
h6 = plot( rad2deg(q3m),Data.OrientError(length(q1m)+length(q2m)+1:length(q1m)+length(q2m)+length(q3m)) , 'color' , [0.4 0.8 0.4],'LineWidth',1.2);


%% TESTING Box Plots:
% First Joint:

figure;
hold on;
h1 = plot( rad2deg(q1m),data1.X , '-*', 'color' , [0.8 0.5 0.4],'LineWidth',1.2, 'MarkerIndices', 1:200:n1-1);
h2 = plot( rad2deg(q1m),data1.Y , '-^', 'color' , [0.4 0.8 0.4],'LineWidth',1.2, 'MarkerIndices', 1:200:n1-1);
h3 = plot( rad2deg(q1m),data1.Z , '-s', 'color' , [0.5 0.4 0.8],'LineWidth',1.2, 'MarkerIndices', 1:200:n1-1);
h4 = plot( rad2deg(q1m),(rad2deg(q1m)-26.3737), '-o', 'color', [0.5 0.5 0.5], 'LineWidth',1.2, 'MarkerIndices', 1:200:n1-1);
h5 = plot( rad2deg(q1m), Data.Error(1:1800) , 'color' , [0.8 0.5 0.4],'LineWidth',1.2);
h6 = plot( rad2deg(q1m),Data.OrientError(1:1800) , 'color' , [0.4 0.8 0.4],'LineWidth',1.2);

% Add legends to the plot:
legend([h1, h2, h3, h4, h5, h6], {'X', 'Y', 'Z', 'O', 'Pe' , 'Oe'},'Location', 'southeast','AutoUpdate','off');
title('Plot of X,Y,Z and R (and errors) vs. Joint1 ');
xlabel('Joint1 (deg)');
ylabel('X, Y, Z, Pe (mm), and R, Oe (deg)');

ZoomX = [20, 21];
ZoomY = [-0.001, 0.001];
plot([mean(ZoomX), 47], [0, 100], '--k');
axes('Position',[0.7 0.7 0.2 0.2]);
box on 
hold on
h5 = plot( rad2deg(q1m), Data.Error(1:1800) , 'color' , [0.8 0.5 0.4],'LineWidth',1.2);
h6 = plot( rad2deg(q1m),Data.OrientError(1:1800) , 'color' , [0.4 0.8 0.4],'LineWidth',1.2);
% Set the limits for the portion you want to zoom in on
xlim(ZoomX); % Set the x-axis limits from 20 to 21
ylim(ZoomY); % Set the y-axis limits from -1 to 1
% Add text to the box
text(20.5, 0.0005, 'Error', 'FontSize', 10, 'FontWeight', 'bold');

% Plot for Second Joint
figure;
hold on;
h1 = plot( rad2deg(q2m),data2.X , '-*', 'color' , [0.8 0.5 0.4],'LineWidth',1.2, 'MarkerIndices', 1:200:n2-1);
h2 = plot( rad2deg(q2m),data2.Y , '-^', 'color' , [0.4 0.8 0.4],'LineWidth',1.2, 'MarkerIndices', 1:200:n2-1);
h3 = plot( rad2deg(q2m),data2.Z , '-s', 'color' , [0.5 0.4 0.8],'LineWidth',1.2, 'MarkerIndices', 1:200:n2-1);
h4 = plot( rad2deg(q2m),(rad2deg(q2m)-0.05), '-o', 'color', [0.5 0.5 0.5], 'LineWidth',1.2, 'MarkerIndices', 1:200:n2-1);
h5 = plot( rad2deg(q2m), Data.Error(length(q1m)+1:length(q1m)+length(q2m)) , 'color' , [0.8 0.5 0.4],'LineWidth',1.2);
h6 = plot( rad2deg(q2m),Data.OrientError(length(q1m)+1:length(q1m)+length(q2m)) , 'color' , [0.4 0.8 0.4],'LineWidth',1.2);
% Add legends to the plot:
legend([h1, h2, h3, h4, h5, h6], {'X', 'Y', 'Z', 'O', 'Pe' , 'Oe'},'Location', 'southeast','AutoUpdate','off');
title('Plot of X,Y,Z and R (and errors) vs. Joint2 ');
xlabel('Joint2 (deg)');
ylabel('X, Y, Z, Pe (mm), and R, Oe (deg)');
ZoomX = [50, 51];
ZoomY = [-0.001, 0.001];
plot([mean(ZoomX), 68], [0, 170], '--k');
axes('Position',[.7 .7 .2 .2])
box on 
hold on
h5 = plot( rad2deg(q2m), Data.Error(length(q1m)+1:length(q1m)+length(q2m)) , 'color' , [0.8 0.5 0.4],'LineWidth',1.2);
h6 = plot( rad2deg(q2m),Data.OrientError(length(q1m)+1:length(q1m)+length(q2m)) , 'color' , [0.4 0.8 0.4],'LineWidth',1.2);
% Set the limits for the portion you want to zoom in on
xlim(ZoomX); % Set the x-axis limits from 20 to 21
ylim(ZoomY); % Set the y-axis limits from -1 to 1
% Add text to the box
text(mean(ZoomX), 0.0005, 'Error', 'FontSize', 10, 'FontWeight', 'bold');

% Plotting for Third joint:
figure;
hold on;
h1 = plot( rad2deg(q3m),data3.X , '-*', 'color' , [0.8 0.5 0.4],'LineWidth',1.2, 'MarkerIndices', 1:200:n3-1);
h2 = plot( rad2deg(q3m),data3.Y , '-^', 'color' , [0.4 0.8 0.4],'LineWidth',1.2, 'MarkerIndices', 1:200:n3-1);
h3 = plot( rad2deg(q3m),data3.Z , '-s', 'color' , [0.5 0.4 0.8],'LineWidth',1.2, 'MarkerIndices', 1:200:n3-1);
h4 = plot( rad2deg(q3m),(rad2deg(q3m)-0.05), '-o', 'color', [0.5 0.5 0.5], 'LineWidth',1.2, 'MarkerIndices', 1:200:n3-1);
h5 = plot( rad2deg(q3m), Data.Error(length(q1m)+length(q2m)+1:length(q1m)+length(q2m)+length(q3m)) , 'color' , [0.8 0.5 0.4],'LineWidth',1.2);
h6 = plot( rad2deg(q3m),Data.OrientError(length(q1m)+length(q2m)+1:length(q1m)+length(q2m)+length(q3m)) , 'color' , [0.4 0.8 0.4],'LineWidth',1.2);

% Add legends to the plot:
legend([h1, h2, h3, h4, h5, h6], {'X', 'Y', 'Z', 'O', 'Pe' , 'Oe'},'Location', 'southeast','AutoUpdate','off');
title('Plot of X,Y,Z and R (and errors) vs. Joint3 ');
xlabel('Joint3 (deg)');
ylabel('X, Y, Z, Pe (mm), and R, Oe (deg)');

ZoomX = [65, 66];
ZoomY = [-0.001, 0.001];
plot([mean(ZoomX), 80], [0, 250], '--k');
axes('Position',[.7 .7 .2 .2])
box on 
hold on
h5 = plot( rad2deg(q2m), Data.Error(length(q1m)+1:length(q1m)+length(q2m)) , 'color' , [0.8 0.5 0.4],'LineWidth',1.2);
h6 = plot( rad2deg(q2m),Data.OrientError(length(q1m)+1:length(q1m)+length(q2m)) , 'color' , [0.4 0.8 0.4],'LineWidth',1.2);
% Set the limits for the portion you want to zoom in on
xlim(ZoomX); % Set the x-axis limits from 20 to 21
ylim(ZoomY); % Set the y-axis limits from -1 to 1
% Add text to the box
text(mean(ZoomX), 0.0005, 'Error', 'FontSize', 10, 'FontWeight', 'bold');

%% X Y Z O Plotting with Errors:

figure;
hold on;
h1 = plot( rad2deg(q1m),data1.X , '-*', 'color' , [0.8 0.5 0.4],'LineWidth',1.2, 'MarkerIndices', 1:200:n1-1);
h2 = plot( rad2deg(q1m),data1.Y , '-^', 'color' , [0.4 0.8 0.4],'LineWidth',1.2, 'MarkerIndices', 1:200:n1-1);
h3 = plot( rad2deg(q1m),data1.Z , '-s', 'color' , [0.5 0.4 0.8],'LineWidth',1.2, 'MarkerIndices', 1:200:n1-1);
h4 = plot( rad2deg(q1m),(rad2deg(q1m)-26.3737), '-o', 'color', [0.5 0.5 0.5], 'LineWidth',1.2, 'MarkerIndices', 1:200:n1-1);

h5 = plot( rad2deg(q1m), Data.Error(1:1800) , 'color' , [0.8 0.5 0.4],'LineWidth',1.2);
h6 = plot( rad2deg(q1m),Data.OrientError(1:1800) , 'color' , [0.4 0.8 0.4],'LineWidth',1.2);

% Add legends to the plot:
legend([h1, h2, h3, h4], {'X', 'Y', 'Z', 'O', 'Pe' , 'Oe'},'Location', 'southeast');
title('Plot of X,Y,Z and R (and errors) vs. Joint1 ');
xlabel('Joint1 (deg)');
ylabel('X, Y, Z, Pe (mm), and R, Or (deg)');

% Plot Error for first Joint
% Assuming you already have your plot
% If not, replace this with your plotting code

% Set the limits for the portion you want to zoom in on
xlim([20, 21]); % Set the x-axis limits from 20 to 21
ylim([-0.001, 0.001]); % Set the y-axis limits from -1 to 1

%% Downsampled error plotting:

figure;
% Define the desired number of points to plot
desired_num_points = 100; % Adjust this value as needed

% Downsample the data
downsampled_indices = round(linspace(1, n1, desired_num_points));
downsampled_q1 = q1(downsampled_indices);
downsampled_error = Data.Error(downsampled_indices);

% Plot the downsampled data
h1 = plot(rad2deg(downsampled_q1), downsampled_error, '-*', 'color', [0.8 0.5 0.4], 'LineWidth', 1.2, 'MarkerIndices', 1:25:length(downsampled_error));


%% Saving Data to Excel
% % Define the filename for the Excel file
% filename = 'Data.xlsx';
% 
% %q1_table = array2table(q1.');
% % Combine the data into a matrix
% %data_matrix = [q1_table, data1];
% 
% % Write the data to an Excel file
% writetable(Data, filename);
% Add legends to the plot:
legend([h1, h2, h3, h4], {'X', 'Y', 'Z', 'O'},'Location', 'southeast');
title('Plot of X,Y,Z and R vs. Joint1 - Model');
xlabel('Joint1 (deg)');
ylabel('X, Y, Z (mm), and R (deg)');

% Plot for Second Joint
figure;
hold on;
h1 = plot( rad2deg(q2),A2(1:n2,1) , '-*', 'color' , [0.8 0.5 0.4],'LineWidth',1.2, 'MarkerIndices', 1:200:length(A2(1:n2,1)));
h2 = plot( rad2deg(q2),A2(1:n2,2) , '-^', 'color' , [0.4 0.8 0.4],'LineWidth',1.2, 'MarkerIndices', 1:200:length(A2(1:n2,2)));
h3 = plot( rad2deg(q2),A2(1:n2,3) , '-s', 'color' , [0.5 0.4 0.8],'LineWidth',1.2, 'MarkerIndices', 1:200:length(A2(1:n2,3)));
%h4 = plot( rad2deg(q2),-0.05, '-o', 'color', [0.4 0.7 0.6], 'LineWidth',1.2, 'MarkerIndices', 1:200:length(A1(1:1800,3)));

% Add legends to the plot:
legend([h1, h2, h3], {'X', 'Y', 'Z'},'Location', 'southeast');
title('Plot of X, Y, and Z vs. Joint2 - Model');
xlabel('Joint2 (deg)');
ylabel('X, Y, and Z (mm)');

% Plot for Third Joint
figure;
hold on;
h1 = plot( rad2deg(q3),A3(1:n3,1) , '-*', 'color' , [0.8 0.5 0.4],'LineWidth',1.2, 'MarkerIndices', 1:200:length(A3(1:n3,1)));
h2 = plot( rad2deg(q3),A3(1:n3,2) , '-^', 'color' , [0.4 0.8 0.4],'LineWidth',1.2, 'MarkerIndices', 1:200:length(A3(1:n3,2)));
h3 = plot( rad2deg(q3),A3(1:n3,3) , '-s', 'color' , [0.5 0.4 0.8],'LineWidth',1.2, 'MarkerIndices', 1:200:length(A3(1:n3,3)));
% Add legends to the plot:
legend([h1, h2, h3], {'X', 'Y', 'Z'},'Location', 'southeast');
title('Plot of X, Y, and Z vs. Joint3 - Model');
xlabel('Joint3 (deg)');
ylabel('X, Y, and Z (mm)');

%% Plotting for 2 joints rotations (Changing the Sample Rate to 1):
% % Commented for not being very informative:
% 
% TR4 = zeros(4,4,n2_2*n3_2); % for joints 2 and three rotations
% A23 = zeros(n2_2*n3_2,5);
% ii23 = 1;
% 
% for i = 1:length(q2_2)
%    for j = 1:length(q3_2)
%        Q = [0,q2(i),q3(j)-q2(i),-q3(j),0];
%        TR4(:,:,ii23) = Rob1.fkine(Q);
%        A23(ii23,4) = rad2deg(q2_2(i));
%        A23(ii23,5) = rad2deg(q3_2(j));
%        ii23 = ii23+1;
%    end
% end
% 
% A23(:,1:3) = transl(TR4);
% 
% % Plot for Two Joints
% figure;
% plot3(A23(:,4), A23(:,5), A23(:,3));

%% Calculating the Workspace from the DH-Method:
% Calculating the Workspace: 
x   = 1;
% N   = 40;
N = 25;
q1 = (linspace(deg2rad(-90) ,deg2rad(90)    ,N))';
q2 = (linspace(deg2rad(0)   ,deg2rad(90)    ,N))';
q3 = (linspace(deg2rad(-10) ,deg2rad(90)    ,N))';
% For faster results q4 will be considered as 0
% q4 = (linspace(deg2rad(-90) ,deg2rad(90)    ,N))'; 
% TR1 = [zeros(4,4,N^4)];

TR1 = zeros(4,4,N^3);

for i = 1:length(q1)
    for j=1:length(q2)
        for ii = 1:length(q3)
            % for jj = 1:length(q4)  
               % TR1(:,:,x) = Rob1.fkine([q1(i),q2(j),q3(ii)-q2(j),-q3(ii),q4(jj)]);
               % for faster results q4 is considered 0 as it will not have
               % effects on the workspace with the system studied
               Q = [q1(i),q2(j),q3(ii)-q2(j),-q3(ii),0];
               TR1(:,:,x) = Rob1.fkine(Q);
               x = x+1;
            % end
        end
    end
end

A1 = transl(TR1);

%% Centroid and Alpha Shape
% K-Means clsutering to find the center of mass for the points of Robot 2:
num_clusters1 = 1;
[~, centroid1] = kmeans(A1 , num_clusters1);

Shrink_Factor = 0.8;
[k_A,vol_A] = boundary(A1,Shrink_Factor);

%% Plotting the workspace of the Robot:
% Alpha-Shape
figure;
trisurf(k_A, A1(:,1), A1(:,2), A1(:,3), 'FaceColor', [0.4 0.6 0.7], 'FaceAlpha', 0.7);
hold on
plot3(0,0,0,"+","MarkerSize",10,"Color","white","LineWidth",2);
xlabel('X'); ylabel('Y'); zlabel('Z');
title("Workspace of Dobot with AlphaShape: Sf =  " + (Shrink_Factor));
axis equal;
hold off;

% Plotting with a color map representation:
map = [0 0 0.2; 0 0 0.3; 0 0 0.4; 0 0 0.45; 0.1 0.1 0.5; 0.1 0.1 0.55; 
    0.15 0.15 0.6; 0.2 0.2 0.65; 0.25 0.25 0.7; 0.3 0.3 0.8; 0.4 0.4 0.9];
figure;
custom_colormap = map;
min_cdata = min(A1(:,1));
max_cdata = max(A1(:,1));
normalized_cdata = (A1(:,1) - min_cdata) / (max_cdata - min_cdata);
colormap(custom_colormap);
scatter3(A1(:,1), A1(:,2), A1(:,3), 'filled', 'CData', normalized_cdata,'MarkerEdgeAlpha',0.1,'MarkerFaceAlpha',0.4);
hold on
plot3(0,0,0,"+","MarkerSize",10,"Color","white","LineWidth",2);
xlabel('X'); ylabel('Y'); zlabel('Z');
title("Point-cloud of Dobot's Workspace: ");
axis equal;
hold off;

%
% figure;
% %Rob1.plot([0,0,0,0,0]);
% %c = [0.2 0.2 0.6];
% c=  [0.4 0.4 0.7];
% % scatter3(A1(:,1),A1(:,2),A1(:,3),100,'filled','MarkerEdgeColor','b','MarkerFaceColor',c,'MarkerEdgeAlpha',0,'MarkerFaceAlpha',0.2)
% scatter3(A1(:,1),A1(:,2),A1(:,3),'filled','CData', A1(:,2));
% hold on
% plot3(0,0,0,"+","MarkerSize",10,"Color","white","LineWidth",2);
% xlabel('X'); ylabel('Y'); zlabel('Z');
% title("Point-cloud of Dobot's Workspace: ");
% axis equal;
% hold off;

% Convex Hull
k_convex = convhull(A1);
figure;
trisurf(k_convex, A1(:,1), A1(:,2), A1(:,3), 'FaceColor', [0.4 0.6 0.7], 'FaceAlpha', 0.7);
hold on
plot3(0,0,0,"+","MarkerSize",10,"Color","white","LineWidth",2);
xlabel('X'); ylabel('Y'); zlabel('Z');
title("Workspace of Dobot - using Convex Hull");
axis equal;
hold off;

% Delaunay Triangulation
DT = delaunay(A1);
figure;
trisurf(DT, A1(:,1), A1(:,2), A1(:,3), 'FaceAlpha', 0.3, 'FaceColor', [0.4 0.6 0.7]);
hold on;
plot3(0,0,0,"+","MarkerSize",10,"Color","white","LineWidth",2);
xlabel('X'); ylabel('Y'); zlabel('Z');
title("Workspace of Dobot - using Delaunay Triangulation");
axis equal;
hold off;

%% 
trimesh(DT,A1(:,1), A1(:,2), A1(:,3),'FaceAlpha', 0.3, 'FaceColor', [0.4 0.6 0.7]);
title("Workspace of Dobot - using Delaunay Triangulation");
axis equal;
hold off;
%% Different Specified Test Cases:
% Cases saved from Dobot Studio at random points

% Case = [j1, j2, j3, j4, X, Y, Z, R];

Case1 = [31.3050,25.0260,40.3646,-2.3200,195.5018,118.8902,27.1213,28.9850];
Case2 = [31.3314,38.6018,46.3190,-3.0400,209.6582,127.6318,-0.8074,28.2914];
Case3 = [9.6556,38.4255,21.0770,69.2999,276.7899,47.0917,52.8967,78.9555];
Case4 = [9.7050,22.9353,65.6058,34.3000,170.5448,29.1670,-9.5490,44.0049];
Case5 = [32.7187,30.1405,59.2217,47.5263,170.5462,109.5672,-9.5481,80.2449];
Case6 = [-60.0450,40.6527,40.5472,5.0463,129.4983,-224.7053,6.8597,-54.9987];
Case7 = [-27.6715,40.5099,54.1403,-6.8337,206.7988, -108.4406,-16.4971,-34.5052];
Case8 = [-25.5274,59.3305,34.8951,-17.8737,267.4528,-127.7254,-15.2335,-43.4011];
Case9 = [-57.9539,40.7509,78.8324,19.4463,93.5425,-149.4314,-41.9467,-38.5076];
Case10 = [-26.9568,52.3085,61.9307,25.9263,210.0880,-106.8457,-47.1694,-1.0305];
Case11 = [57.1138,60.0392,49.9026,9.4863,147.3304,227.8584,-45.0277,66.6001];
Case12 = [36.9432,50.9156,77.7919,-0.8337,156.3093,117.5448,-58.5631,36.1095];
Case13 = [11.7961,60.0068,52.8343,-0.5137,259.8219,54.2614,-49.6568,11.2824];
Case14 = [8.9638,51.5333,54.7904,64.2861,247.1014,38.9770,-36.1281,73.2499];

%% Test Case Forward Kinematics (Geometric):

clear t1 t2 t3 theta1 theta2 theta3

t1 = 0; theta1 = deg2rad(t1);
t2 = -9.3080 ; theta2 = deg2rad(t2);
t3 = 64.9956; theta3 = deg2rad(t3);
t4 = 0; theta4 = deg2rad(t4);

r = L2 * sin(theta2) + L3 * cos(theta3) + L4;

% Calculate the end points for Dobot:
x1 = cos(theta1) * r;
y1 = sin(theta1) * r;
z1 = L1 + L2 * cos(theta2) - L3 * sin(theta3);

% Orientation relative to base:
Rbase2end = trotz(theta4) * trotz(theta1) * trotx(pi) * Tbase1;
% To match with Robot DK model:
% z1 = L1 + L2 * cos(theta2) - L3 * sin(theta3) + 133;

fprintf("The Dobot End effector is at:" + ...
    "\n X = %1.5f , Y = %1.5f , Z = %1.5f \n", x1 , y1, z1);


%% Test Case: Forward kinematics (DH) Random:

t1 = round((qmax(1)-qmin(1))*rand() - ((qmax(1)-qmin(1))/2),4); theta1 = deg2rad(t1);
t2 = round((qmax(2)-qmin(2))*rand() - ((qmax(2)-qmin(2))/2),4); theta2 = deg2rad(t2);
t3 = round((qmax(3)-qmin(3))*rand() - ((qmax(3)-qmin(3))/2),4); theta3 = deg2rad(t3);
t4 = round((qmax(4)-qmin(4))*rand() - ((qmax(4)-qmin(4))/2),4); theta4 = deg2rad(t4);

T(1) = t1; T(2) = t2; T(3) = t3; T(4) = t4; %#ok<NASGU> 
Q = [theta1,theta2,theta3-theta2,-theta3,theta4];
Qdisp = [t1,t2,t3,t4];

TP = Rob1.fkine(Q).t;

% Error = sqrt((TP(1)-F(1))^2+(TP(2)-F(2))^2+(TP(3)-F(3))^2);
format short;
disp('Joints: ');
disp(Qdisp);
%format long;
disp('End Effecto Position');
disp(TP);

figure;
Rob1.plot(Q,'view',[40 20],'wrist','jaxes','arrow','jointcolor',[0.3,0.3,0.3],'linkcolor',[1,1,1],'tile1color',[0.9,0.9,0.9],'tile2color',[0.9,0.9,0.9]);
set(gca,'color',[0.5,0.6,0.7]);


%% Test Case: Forward kinematics (DH) with Values:

CaseForward = Case1;
Q = deg2rad([CaseForward(1),CaseForward(2),CaseForward(3)-CaseForward(2),-CaseForward(3),CaseForward(4)]);
Qdisp = [CaseForward(1),CaseForward(2),CaseForward(3),CaseForward(4)];
F = [CaseForward(5),CaseForward(6),CaseForward(7)];
RF = CaseForward(8);

TP = Rob1.fkine(Q).t;

F_Error = sqrt((TP(1)-F(1))^2+(TP(2)-F(2))^2+(TP(3)-F(3))^2);
F_OrientationError = abs(RF - (CaseForward(1)+CaseForward(4)));

%format Long;
disp('Joints: ');
disp(Qdisp);
format long;
disp('End Effecto Position');
disp(TP);
format short;
disp('Error Value: ');
disp(F_Error);
disp('Orientation Error Value: ');
disp(F_OrientationError );
figure;
Rob1.plot(Q,'view',[40 20],'wrist','jaxes','arrow','jointcolor',[0.3,0.3,0.3],'linkcolor',[1,1,1],'tile1color',[0.9,0.9,0.9],'tile2color',[0.9,0.9,0.9]);
set(gca,'color',[0.5,0.6,0.7]);

%% Test Case: Inverse kinematics (Random):

t1 = (qmax(1)-qmin(1))*rand() - ((qmax(1)-qmin(1))/2); theta1 = deg2rad(t1);
t2 = (qmax(2)-qmin(2))*rand() - ((qmax(2)-qmin(2))/2); theta2 = deg2rad(t2);
t3 = (qmax(3)-qmin(3))*rand() - ((qmax(3)-qmin(3))/2); theta3 = deg2rad(t3);
t4 = (qmax(4)-qmin(4))*rand() - ((qmax(4)-qmin(4))/2); theta4 = deg2rad(t4);
T = zeros(1,4);

T(1) = t1; T(2) = t2; T(3) = t3; T(4) = t4;
Q = [theta1,theta2,theta3-theta2,-theta3,theta4];
TP = Rob1.fkine(Q).t; 
R = theta1+theta4;

tic;
%[Q,iterations] = DobotInverse(TestPoint,IntGuess);
[Qsol,iterations] = DobotInverse(TP,R); % Without Initial Guess!
toc;
t = toc;
% Mapping Q to Robot's joints:
Q2 = [Qsol(1),Qsol(2),-Qsol(4),Qsol(5)];
F = Rob1.fkine(Qsol).t;
Error = sqrt((TP(1)-F(1))^2+(TP(2)-F(2))^2+(TP(3)-F(3))^2);
format long;
% Display the Q values
% disp('Real angles Values:');
% disp(T);
disp('Q values: ');
disp(rad2deg(Q2));
disp('The Euclidean distance between the solution and the original point is: ');
disp(Error);
format short;
disp('Number of iterations: ');
disp(iterations);
disp('Time taken: ');
disp(t);
disp('End Effecto Position');
disp(TP);
figure;
Rob1.plot(Q,'view',[40 20],'wrist','jaxes','arrow','jointcolor',[0.3,0.3,0.3],'linkcolor',[1,1,1],'tile1color',[0.9,0.9,0.9],'tile2color',[0.9,0.9,0.9]);
set(gca,'color',[0.5,0.6,0.7]);

%% Test Case: Inverse Kinematics (Analytical - Specified):
% TP = [206.7, 0, 135];
% R = rad2deg(0);
Case = Case12;
TP = Case(5:7);
R = Case(8);
tic;
%[Q,iterations] = DobotInverse(TestPoint,IntGuess);
[Qsol,iterations] = DobotInverseAnalytical(TP,R); % Without Initial Guess!
toc;
t = toc;
% Mapping Q to Robot's joints:
Q_model = [Qsol(1),Qsol(2),Qsol(3)-Qsol(2),-Qsol(3),Qsol(4)];
F = Rob1.fkine(deg2rad(Q_model)).t;
Error = sqrt((TP(1)-F(1))^2+(TP(2)-F(2))^2+(TP(3)-F(3))^2);
format long;
% Display the Q values
% disp('Real angles Values:');
% disp(T);
disp('Q values: ');
disp(Q_model);
disp('The Euclidean distance between the solution and the original point is: ');
disp(Error);
format short;
disp('Time taken: ');
disp(t);
disp('End Effecto Position');
disp(TP);
figure;
Rob1.plot(Q,'view',[40 20],'wrist','jaxes','arrow','jointcolor',[0.3,0.3,0.3],'linkcolor',[1,1,1],'tile1color',[0.9,0.9,0.9],'tile2color',[0.9,0.9,0.9]);
set(gca,'color',[0.5,0.6,0.7]);

%% Test Case: Inverse kinematics (Specified):
CaseInverse = Case12;
%IntGuess = [-0.3,0,0,0];
tic;
%[Q,iterations] = DobotInverse(TestPoint,R,IntGuess);
[Qsol,iterations] = DobotInverse([CaseInverse(5),CaseInverse(6),CaseInverse(7)],CaseInverse(8)); % Without Initial Guess!
toc;
t = toc;

% Mapping Q to Robot's joints:
Q2 = [Qsol(1),Qsol(2),-Qsol(4),Qsol(5)];

% Re-forward to calculate error:
F = Rob1.fkine(Qsol).t;
Error = sqrt((CaseInverse(5)-F(1))^2+(CaseInverse(6)-F(2))^2+(CaseInverse(7)-F(3))^2);
OrientError = (Qsol(1)+Qsol(5)) - deg2rad(CaseInverse(8));

format short;
% Display the Q values
% disp('Real angles Values:');
% disp(T);
disp('Q values: ');
disp(rad2deg(Q2));
disp('The Euclidean distance between the solution and the original point is: ');
disp(Error);
disp("The orientation error is: ");
disp(OrientError);
format short;
disp('Number of iterations: ');
disp(iterations);
disp('Time taken: ');
disp(t);
format Long;
disp('End Effecto Position');
disp(F);

figure;
Rob1.plot(Qsol,'view',[40 20],'wrist','jaxes','arrow','jointcolor',[0.3,0.3,0.3],'linkcolor',[1,1,1],'tile1color',[0.9,0.9,0.9],'tile2color',[0.9,0.9,0.9]);
set(gca,'color',[0.5,0.6,0.7]);

%% Calculate Inverse Kinematics for some of the plotted points:
x = 1;
%Breakfactor = 1;
Breakfactor = 1;

nR = height(Data)/Breakfactor;
TRI = zeros(4,4,nR);
nIterations = zeros(nR,1);
errorOrientation = zeros(nR,1);
timeI = zeros(nR,1);
OrientationR = zeros(nR,1);
QQ1 = zeros(nR,4);
% Errori = zeros(nR,1);

for i = 1:nR
    ii = i * Breakfactor;
    QQ1(i,1) = Data{ii,5};
    QQ1(i,2) = Data{ii,6};
    QQ1(i,3) = Data{ii,7};
    QQ1(i,4) = Data{ii,8};
    [QI,nI] = DobotInverseAnalytical([Data{ii,5},Data{ii,6},Data{ii,7}],Data{ii,8});
    Q_in = [QI(1),QI(2),QI(3)-QI(2),-QI(3),QI(4)];
    TRI(:,:,x) = Rob1.fkine(deg2rad(Q_in));
    nIterations(i) = nI;
    % timeI(i) = tI;
    OrientationR(i) = Data{ii,8};
    errorOrientation(i) = Q_in(5)+Q_in(1)-OrientationR(i);
    x = x+1;
end

AR1 = transl(TRI);

IData = table(QQ1(:,1),QQ1(:,2),QQ1(:,3),QQ1(:,4),AR1(:,1), AR1(:,2), AR1(:,3), 'VariableNames', {'X', 'Y', 'Z','R','Xdh','Ydh','Zdh'});
IData.iterations = nIterations(:,1);
IData.time = timeI(:,1);
IData.Errori = sqrt((IData.X-IData.Xdh).^2+(IData.Y-IData.Ydh).^2+(IData.Z-IData.Zdh).^2);
IData.errorOrientation = errorOrientation(:,1);

format long
fprintf("Mean Position Error is %f \n",mean(IData.Errori));
fprintf("Max Position Error is %f \n",max(IData.Errori));
fprintf("Mean R Error is %f \n",mean(IData.errorOrientation));
fprintf("Max R Error is %f \n",max(IData.errorOrientation));
fprintf("Mean number of iterations is %f \n",mean(IData.iterations));
fprintf("Max number of iterations is %f \n",max(IData.iterations));
fprintf("Mean time to find solution is %f \n",mean(IData.time));
fprintf("Max time to find solution is %f \n",max(IData.time));
%% Test Case: Inverse kinematics with Parpool:
% Commented for taking too much time for this case
%
% TestPoint = [200, -60, 135];
% 
% IntGuess = [-0.3,0,0,0];
% tic;
% 
% [Q,iterations] = DobotInverseParpool(TestPoint,100);
% 
% % [Q,iterations] = DobotInverse(TestPoint); % Without Initial Guess!
% 
% 
% toc;
% t = toc;
% 
% % Display the Q values
% disp('Q values: ');
% disp(Q);
% disp('Number of iterations: ');
% disp(iterations);
% disp('Time taken: ');
% disp(t);
% 
% figure;
% Rob1.plot(Q,'view',[40 20],'wrist','jaxes','arrow','jointcolor',[0.3,0.3,0.3],'linkcolor',[1,1,1],'tile1color',[0.9,0.9,0.9],'tile2color',[0.9,0.9,0.9]);
% set(gca,'color',[0.5,0.6,0.7]);

%% FUNCTIONS:
% 1. Symbolic Equations for Geomtric and DH approach:

syms q1v q2v q3v q4v q5v L1v L2v L3v L4v

TeqGeometric = trotz(q4v) * trotz(q1v) * troty(pi) * Tbase1;
display(simplify(TeqGeometric));
DobotM(1)= Link([0 , L1v  , 0  , -pi/2]);
DobotM(2)= Link([0 ,  0  , L2v , 0    ]);
DobotM(3)= Link([0 ,  0  , L3v , 0    ]);
DobotM(4)= Link([0 ,  0  , L4v , -pi/2]);
DobotM(5)= Link([0 ,  0 , 0  , 0   ]);
DobotM(2).offset = -pi/2;
DobotM(3).offset = pi/2 - DobotM(2).theta;
DobotM(5).offset = pi;
Robeq = SerialLink(DobotM,'name','Dobot');
% Compute the end-effector pose in terms of the symbolic joint angles
Q = [q1v, q2v, q3v-q2v, -q3v, q4v];
TeqDH = Robeq.fkine(Q);
Xeq = TeqDH.t(1);
Yeq = TeqDH.t(2);
Zeq = TeqDH.t(3);
disp("The final transformation from base to EE");
display(simplify(TeqDH));

%% Analytical Solver:

syms X Y Z
Xq = Xeq - X == 0;
Yq = Yeq - Y == 0;
Zq = Zeq - Z == 0;
q1v = atan(Y/X);
solution = solve([Xq Yq Zq],[q2v q3v])

%% Inverse Analytical 
syms L1 L2 L3 L4 X Y Z q1 q2 q3;

% % Given equations for X, Y, and Z
% eq1 = X - cos(q1)*(L4 + L3*cos(q3) + L2*sin(q2)) == 0;
% eq2 = Y - sin(q1)*(L4 + L3*cos(q3) + L2*sin(q2)) == 0;
% eq3 = Z - (L1 + L2*cos(q2) - L3*sin(q3)) == 0;
% %eq4 = atan(Y/X) - q1 == 0;
% 
% % Solve for L2v and L3v symbolically
% [so] = solve([eq1, eq2, eq3], [q2, q3]);
% 
% Q2 = simplify(so.q2(1,1))
% Q3 = simplify(so.q3(2,1))

syms L1 L2 L3 L4 X Y Z q1 q2 q3;

% Given equations for X, Y, and Z:
eq1 = X - cos(atan(Y/X))*(L4 + L3*cos(q3) + L2*sin(q2)) == 0;
eq2 = Y - sin(atan(Y/X))*(L4 + L3*cos(q3) + L2*sin(q2)) == 0;
eq3 = Z - (L2*cos(q2) - L3*sin(q3)) == 0;

% Solve for q2 and q3 symbolically:
so = solve([eq1, eq2, eq3],[q2, q3]);

%% Extract solutions
Q2 = simplify(so.q2(1,1));
Q3 = simplify(so.q3(1,1));
Q4 = simplify(so.q2(2,1));
Q5 = simplify(so.q3(2,1));

%% Test Case

Case15 = [0,0,0,0,305.258,0,117.932];
X = Case15(5);
Y = Case15(6);
Z = Case15(7);

L1 = 0; % To match the Dobot Studio L1 = 0.
L2 = 135; % length of link 2
L3 = 147; % length of link 3
L4 = 59.7;  % length of link 4. L4 = 61 To match with RoboDk Model.

r = sqrt(X^2 + Y^2)- L4;
h = sqrt(r^2 + Z^2);

% Elbow Up:
solQ2 = rad2deg(atan2(r,Z)) - acosd((L2^2 - L3^2 + h^2)/(2*L2*h))
solQ3 = solQ2 - 90 + acosd((h^2 - L2^2 - L3^2) /(2 * L2 * L3))
% Elbow Down:
solQ2_2 = rad2deg(atan2(r,Z)) + acosd((L2^2 - L3^2 + h^2)/(2*L2*h))
solQ3_2 = solQ2_2 - 90 - acosd((h^2 - L2^2 - L3^2) /(2 * L2 * L3))


% t2 = vpa(rad2deg(subs(Q2)))
% t3 = vpa(rad2deg(subs(Q3)))
% t2_2 = vpa(rad2deg(subs(Q4)))
% t3_2 = vpa(rad2deg(subs(Q5)))

% J = jacobian([Xeq;Yeq;Zeq],[q1v,q2v,q3v]);
J = subs(J, [L1v, L2v, L3v, L4v], [L1, L2, L3, L4]);
display(simplify(J));
detJ = det(J);
solutions = solve(detJ,[q1v,q2v,q3v]);
disp(simplify(detJ));
disp(solutions);

% A Funciton to calculate the svd of iterations of the Jacobian Matrix 
% across the range of the joints:
SampleRate = 50; 
n1 = ((qmax(1)-qmin(1))/SampleRate)+1; % 180/SampelRate = 1800 points
n2 = ((qmax(2)-qmin(2))/SampleRate)+1;
n3 = ((qmax(3)-qmin(3))/SampleRate)+1;

q1 = linspace(theta1_min,theta1_max,n1);
q2 = linspace(theta2_min,theta2_max,n2);
q3 = linspace(theta3_min,theta2_max,n3);

% Jn = subs(J, [q1v, q2v, q3v], [q1(i), q2(j), q3(k)]);
% S = svd(Jn);

for i = 1:length(q1)
    for j = 1:length(q2)
        for k = 1:length(q3) 
            Jn = subs(J, [q1v, q2v, q3v], [q1(i), q2(j), q3(k)]); 
            [U, S, V] = svd(Jn);
            if diag(S) == 0
                singular_values_cell{i, j, k} = 0; %#ok<SAGROW> 
            else
                singular_values_cell{i, j, k} = 1; %#ok<SAGROW> 
            end
%             % Check for singular values close to zero
%             tolerance = 1e-6; % Adjust as needed
%             near_singularities = find(singular_values < tolerance);
        end
    end
end

%% 2. Inverse Kinematics Function:
% Call DobotInverse to calculate first 3 joints' variables
% If currentPose is provided, it will be used as an initial guess
% Otherwise, a default initial guess [0,42.5,52.5,0] is used
% 'interior-point' handles large, sparse problems, as well as small dense problems.
% The algorithm satisfies bounds at all iterations, and can recover from NaN or Inf results. 
% How to Use: [endEffector, currentPose] = End_effector_position;

function [Q, numIterations, timetaken] = DobotInverse(endEffector,R,currentPose)
    % Robot Link Lengths
    L1 = 0; % To match the Dobot Studio L1 = 0.
    L2 = 135; % length of link 2
    L3 = 147; % length of link 3
    L4 = 59.7;  % length of link 4. L4 = 61 To match with RoboDk Model.
    % Define the objective function (error to minimize)
    objective = @(q) norm([(cos(q(1)) * (L4 + L3 * cos(q(3)) + L2 * sin(q(2)))) - endEffector(1);
                           (sin(q(1)) * (L4 + L3 * cos(q(3)) + L2 * sin(q(2)))) - endEffector(2);
                           (L1 + L2 * cos(q(2)) - L3 * sin(q(3))) - endEffector(3);]);
    % Initial guess:     initial_guess = [0, 42.5, 52.5, 0];
    tic;
    % Set initial_guess to currentPose if it's supplied, otherwise use [0, 0, 0, 0]
    if nargin < 3
        initial_guess = [0, deg2rad(42.5), deg2rad(52.5), 0];
    else
        initial_guess = currentPose;
    end
    % No constraints for now
    qmin =  [-90 , 0 , -10 , -90];
    qmax =  [+90 , 85, +95 , +90];
    % Define lower and upper bounds (optional, if needed)
    lb = [deg2rad(qmin(1)), deg2rad(qmin(2)), deg2rad(qmin(3)), deg2rad(qmin(4))];
    ub = [deg2rad(qmax(1)), deg2rad(qmax(2)), deg2rad(qmax(3)), deg2rad(qmax(4))];
    % Call fmincon
    options = optimoptions('fmincon','Algorithm','interior-point','Display', 'iter','MaxIterations', 200,'TolFun', 10^-8 ,'TolX', 10^-8);
    %options = optimoptions('fmincon','Algorithm','sqp', 'Display', 'iter','MaxIterations', 200,'TolFun', 0 ,'TolX', 0);
    [q_optimal, ~, ~, output] = fmincon(objective, initial_guess, [], [], [], [], lb, ub, [], options);
    toc;
    timetaken = toc;
    
    % Access the number of iterations from the output structure
    numIterations = output.iterations;
    % Calculate Q values using DH parameters
    R = deg2rad(R);
    Q = [q_optimal(1), q_optimal(2), q_optimal(3) - q_optimal(2), -q_optimal(3), R-q_optimal(1)]; 
end

%% 2.2 Inverse with parpool:
function [Q, numIterations] = DobotInverseParpool(endEffector,numRuns)

    % Robot Link Lengths
    L1 = 0; % To match the Dobot Studio L1 = 0.
    L2 = 135; % length of link 2
    L3 = 147; % length of link 3
    L4 = 59.7;  % length of link 4. L4 = 61 To match with RoboDk Model.

    % Define the objective function (error to minimize)
    objective = @(q) norm([cos(q(1)) * (L4 + L3 * cos(q(3)) + L2 * sin(q(2))) - endEffector(1);
                           sin(q(1)) * (L4 + L3 * cos(q(3)) + L2 * sin(q(2))) - endEffector(2);
                           L1 + L2 * cos(q(2)) - L3 * sin(q(3)) - endEffector(3)]);

    % Define the common initial guess for all runs
    common_initial_guess = [0, 0, 0, 0];

    % Create a cell array of identical initial guesses
    initial_guesses = repmat({common_initial_guess}, 1, numRuns);
    
    qmin =  [-90 , 0 , -10 , -90];
    qmax =  [+90 , 85, +95 , +90];

    % Define lower and upper bounds (optional, if needed)
    lb = [deg2rad(qmin(1)), deg2rad(qmin(2)), deg2rad(qmin(3)), deg2rad(qmin(4))];
    ub = [deg2rad(qmax(1)), deg2rad(qmax(2)), deg2rad(qmax(3)), deg2rad(qmax(4))];

    % Initialize parallel pool (if not already done)
    if isempty(gcp('nocreate'))
        parpool(); % Initialize a parallel pool
    end

    % Parallelize the optimization runs
    results = cell(1, numel(initial_guesses));

    parfor i = 1:numel(initial_guesses)
        % Set up a separate set of options for each parallel run if needed
        options = optimoptions('fmincon', 'Display', 'off', 'MaxIterations', 200, 'TolX', 1e-6);

        % Run fmincon with the current initial guess
        [q_optimal, ~, ~, output] = fmincon(objective, initial_guesses{i}, [], [], [], [], lb, ub, [], options);

        % Store the results, including the number of iterations
        results{i} = struct('Q', q_optimal, 'Iterations', output.iterations);
    end

%     % Close the parallel pool
%     if ~isempty(gcp('nocreate'))
%         delete(gcp); % Close the parallel pool
%     end

    % Find the best solution based on the minimum number of iterations
    best_solution = results{1};
    best_iterations = best_solution.Iterations;

    for i = 2:numel(initial_guesses)
        current_iterations = results{i}.Iterations;
        if current_iterations < best_iterations
            best_iterations = current_iterations;
            best_solution = results{i};
        end
    end

    % Retrieve the final Q values and number of iterations
    Q = best_solution.Q;
    numIterations = best_solution.Iterations;
    Q = [Q(1), Q(2), Q(3) - Q(2), -Q(3), Q(4)];
   
end
%% 2.3 Inverse Analytical:

function [Q, timetaken] = DobotInverseAnalytical(endEffector,R,elbow)

    L1 = 0; % To match the Dobot Studio L1 = 0.
    L2 = 135; % length of link 2
    L3 = 147; % length of link 3
    L4 = 59.7;  % length of link 4. L4 = 61 To match with RoboDk Model.

    % Start timing
    tic;

    % Calculate inverse kinematics
    X = endEffector(1);
    Y = endEffector(2);
    Z = endEffector(3);

    r = sqrt(X^2 + Y^2) - L4;
    h = sqrt(r^2 + Z^2);
    
    solQ1 = atand(Y/X);

    if nargin < 3
        % Elbow Up:
        solQ2 = rad2deg(atan2(r, Z)) - acosd((L2^2 - L3^2 + h^2) / (2 * L2 * h));
        solQ3 = solQ2 - 90 + acosd((h^2 - L2^2 - L3^2) / (2 * L2 * L3));
    else
        % Elbow Down:
        solQ2 = rad2deg(atan2(r, Z)) + acosd((L2^2 - L3^2 + h^2) / (2 * L2 * h));
        solQ3 = solQ2_2 - 90 - acosd((h^2 - L2^2 - L3^2) / (2 * L2 * L3));
    end

    solQ4 = R - solQ1;
    
    % Calculate time taken
    timetaken = toc;

    % Output joint angles
    Q = [solQ1, solQ2, solQ3, solQ4];
    
end
%% 3. Function to update the plot when the slider value changes:
function sliderCallback(source, ~ , A)
    [init_azimuth, init_elevation] = view;
    Shrink_Factor = source.Value;     % Gets the current value of the slider
    
    [k_A, ~] = boundary(A, Shrink_Factor); % Re-calculate and Re-plot
    [~, centroid1] = kmeans(A , 1);
    plot3(centroid1(1), centroid1(2), centroid1(3), "+", "MarkerSize", 15, "Color", "black", "LineWidth", 2);
    trisurf(k_A, A(:,1), A(:,2), A(:,3), 'FaceColor', [0.4 0.6 0.7], 'FaceAlpha', 0.5);
    view(init_azimuth, init_elevation);
    axis equal;
    hold off;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title("Dobot Magician Workspace with AlphaShape: Sf = " + Shrink_Factor);

    drawnow; % Re-plot
end

%% 4. Function used to plot  variables Captured from Dobot-Studio (.csv)
function csv_to_variables(filename) %#ok<DEFNU> 
    % Reads a CSV file into MATLAB and converts each column into a variable.
    % Read the CSV file into a MATLAB table.
    data = readtable(filename);
    % Get the column names of the table.
    column_names = data.Properties.VariableNames;
    % Create a variable for each column in the table.
    for i = 1:length(column_names)
        column_name = column_names{i};
        column_data = data(:,column_name);
        assignin('base',column_name,column_data);
    end
end

%% 5. Function to convert real manipulator variables into simulation:
function Qout = R2S(Qin) 
    Qout = [Qin(1),Qin(2),Qin(3)-Qin(2),-Qin(3),Qin(4)];
end

%% 6. Function to convert simulation variables into real manipulator:
function qout = S2R(qin) %#ok<DEFNU> 
    qout = [qin(1),qin(2),-qin(4),qin(5)];
end
