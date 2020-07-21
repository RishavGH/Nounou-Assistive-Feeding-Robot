%% Inverse Kinematics function for the IRB120 robot

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Made by:
% Rishav Guha
% Paawan Garg
% Greg Huh
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function config = IRB120IK_3(pose, pastConfig)
%% Forming the configuration struct vector
config(1) = 0;
config(2) = 0;
config(3) = 0;
config(4) = 0;
config(5) = 0;
config(6) = 0;

%% Extracting the joint angles from previous configuration
pTheta1 = pastConfig(1).JointPosition;
pTheta2 = pastConfig(2).JointPosition;
pTheta3 = pastConfig(3).JointPosition;
pTheta4 = pastConfig(4).JointPosition;
pTheta5 = pastConfig(5).JointPosition;
pTheta6 = pastConfig(6).JointPosition;

%% Setting joint limits

jointLimits = [-2.87979,2.87979;
    -1.91986,1.91986;
    -1.91986,1.22173;
    -2.79253,2.79253;
    -2.094395,2.094395;
    -6.98132,6.98132];

%% Setting DH parameters
a2 = 0.27;
a3 = 0.07;
d4 = 0.302;
a6 = 0.046;
dT = 0.2069;

%% Setting the coordinates and two helper variables(r and r2)
rotm = eul2rotm(pose(4:6), 'XYZ');
a_end = rotm(:,3);
n_end = rotm(:,1);
x = pose(1) - dT * a_end(1) - a6 * n_end(1);
y = pose(2) - dT * a_end(2) - a6 * n_end(2);
z = pose(3) - dT * a_end(3) - a6 * n_end(3) - 0.29;

r = sqrt(x^2 + y^2 + z^2);
r2 = sqrt(d4^2 +a3^2);
r1 = sqrt(x^2 + y^2);

%% Joint 1
theta1 = atan2(y,x);

% Checking if theta1 is within range
if theta1 < jointLimits(1,1) || theta1 > jointLimits(1,2)
    theta1 = Inf;   % Solution will be rejected
end

%% Joint 3
theta3a = atan2(sqrt(a3^2 + d4^2 - ((r^2 - a2^2 - a3^2 - d4^2)/(2*a2))^2), ...
    (r^2 - a2^2 - a3^2 - d4^2)/(2*a2)) - atan2(d4,a3);
theta3b = atan2(-sqrt(a3^2 + d4^2 - ((r^2 - a2^2 - a3^2 - d4^2)/(2*a2))^2), ...
    (r^2 - a2^2 - a3^2 - d4^2)/(2*a2)) - atan2(d4,a3);

% Checking if theta3 is within range
if theta3a < jointLimits(3,1) || theta3a > jointLimits(3,2)
    theta3a = Inf;   % Solution will be rejected
end
if theta3b < jointLimits(3,1) || theta3b > jointLimits(3,2)
    theta3b = Inf;   % Solution will be rejected
end

%% Joint 2
alpha = atan2(z/r, r1/r);
betaa = atan2(sqrt(1 - ((a2^2 + r^2 - r2^2)/(2*a2*r))^2), (a2^2 + r^2 - r2^2)/(2*a2*r));
betab = atan2(-sqrt(1 - ((a2^2 + r^2 - r2^2)/(2*a2*r))^2), (a2^2 + r^2 - r2^2)/(2*a2*r));

theta2a = pi/2 - alpha - betaa;
theta2b = pi/2 - alpha - betab;

% Checking if theta2 is within range
if theta2a < jointLimits(2,1) || theta2a > jointLimits(2,2)
    theta2a = Inf;   % Solution will be rejected
end
if theta2b < jointLimits(2,1) || theta2b > jointLimits(2,2)
    theta2b = Inf;   % Solution will be rejected
end

%% Compiling solutions for further filtering
solnmat = [
    theta1 theta2a theta3a;
    theta1 theta2a theta3b;
    theta1 theta2b theta3a;
    theta1 theta2b theta3b];

%% Selecting the solutions that is closest to the previous configuration

% No. of solutions remaining
% nrows = size(matout, 1);

minrow = 1;
minval = Inf;
nrows = 4;
for row = 1:nrows
    if sum(isinf(solnmat(row, :))) == 0
        val = sqrt((pTheta1 - solnmat(row, 1)) ^2 + ...
            (pTheta2 - solnmat(row, 2))^2 + (pTheta3 - solnmat(row, 3))^2);

        if val < minval
            minrow = row;
            minval = val;
        end
    end
end

%% Assigning the joint angles of the chosen solution

config(1) = solnmat(minrow,1);
config(2) = solnmat(minrow,2);
config(3) = solnmat(minrow,3);

theta1 = solnmat(minrow,1);
theta2 = solnmat(minrow,2);
theta3 = solnmat(minrow,3);

%% Rotation

s5sq = (rotm(3,3)*cos(theta2)*cos(theta3) - rotm(3,3)*sin(theta2)*sin(theta3) + rotm(1,3)*cos(theta1)*cos(theta2)*sin(theta3) + rotm(1,3)*cos(theta1)*cos(theta3)*sin(theta2) + rotm(2,3)*cos(theta2)*sin(theta1)*sin(theta3) + rotm(2,3)*cos(theta3)*sin(theta1)*sin(theta2))^2 + (rotm(2,3)*cos(theta1) - rotm(1,3)*sin(theta1))^2;
at5a = sqrt(s5sq);
at5b = rotm(1,3)*cos(theta1)*cos(theta2)*cos(theta3) - rotm(3,3)*cos(theta3)*sin(theta2) - rotm(3,3)*cos(theta2)*sin(theta3) + rotm(2,3)*cos(theta2)*cos(theta3)*sin(theta1) - rotm(1,3)*cos(theta1)*sin(theta2)*sin(theta3) - rotm(2,3)*sin(theta1)*sin(theta2)*sin(theta3);

theta5a = atan2(at5a, at5b);
theta5b = atan2(-at5a, at5b);

at4b = -(rotm(3,3)*cos(theta2)*cos(theta3) - rotm(3,3)*sin(theta2)*sin(theta3) + rotm(1,3)*cos(theta1)*cos(theta2)*sin(theta3) + rotm(1,3)*cos(theta1)*cos(theta3)*sin(theta2) + rotm(2,3)*cos(theta2)*sin(theta1)*sin(theta3) + rotm(2,3)*cos(theta3)*sin(theta1)*sin(theta2));
at4a = rotm(2,3)*cos(theta1) - rotm(1,3)*sin(theta1);

if (abs(at4a) < 1e-11 && abs(at4b) < 1e-11)
    theta4a = 0;
    theta4b = 0;
else
    theta4a = atan2(at4a/sin(theta5a), at4b/sin(theta5a));
    theta4b = atan2(at4a/sin(theta5b), at4b/sin(theta5b));
end

at6a = (rotm(1,2)*cos(theta1)*cos(theta2)*cos(theta3) - rotm(3,2)*cos(theta3)*sin(theta2) - rotm(3,2)*cos(theta2)*sin(theta3) + rotm(2,2)*cos(theta2)*cos(theta3)*sin(theta1) - rotm(1,2)*cos(theta1)*sin(theta2)*sin(theta3) - rotm(2,2)*sin(theta1)*sin(theta2)*sin(theta3));
at6b = -(rotm(1,1)*cos(theta1)*cos(theta2)*cos(theta3) - rotm(3,1)*cos(theta3)*sin(theta2) - rotm(3,1)*cos(theta2)*sin(theta3) + rotm(2,1)*cos(theta2)*cos(theta3)*sin(theta1) - rotm(1,1)*cos(theta1)*sin(theta2)*sin(theta3) - rotm(2,1)*sin(theta1)*sin(theta2)*sin(theta3));

if (abs(at6a) < 1e-11 && abs(at6b) < 1e-11)
    theta6a = 0;
    theta6b = 0;
else
    theta6a = atan2(at6a/sin(theta5a), at6b/sin(theta5a));
    theta6b = atan2(at6a/sin(theta5b), at6b/sin(theta5b));
end

ormat = [theta4a theta5a theta6a;
    theta4b theta5b theta6b];

minrow = 1;
minval = Inf;
for row = 1:2
    if sum(isinf(ormat(row, :))) == 0
        val = sqrt((pTheta4 - ormat(row, 1)) ^2 + ...
            (pTheta5 - ormat(row, 2))^2 + (pTheta6 - ormat(row, 3))^2);

        if val < minval
            minrow = row;
            minval = val;
        end
    end
end

config(4) = ormat(minrow,1);
config(5) = ormat(minrow,2);
config(6) = ormat(minrow,3);
end