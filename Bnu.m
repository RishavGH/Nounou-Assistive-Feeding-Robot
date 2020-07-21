%% Dynamics function for the IRB120 robot

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Made by:
% Rishav Guha
% Paawan Garg
% Greg Huh
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function tau = Bnu(q, qdot, qddot)
tau = zeros(1,6);
w = zeros(3,8);
wdot = zeros(3,8);
vdot = zeros(3,8);
vcdot = zeros(3,8);
f = zeros(3,8);
n = zeros(3,8);
F = zeros(3,8);
N = zeros(3,8);

%% Constant DH parameters
a2 = 0.27;
a3 = 0.07;
d1 = 0.29;
d4 = 0.302;
a6 = 0.046;
dT = 0.2069;
%% Tranformation Matrices
t1 = q(1);
t2 = q(2);
t3 = q(3);
t4 = q(4);
t5 = q(5);
t6 = q(6);

T = zeros(4,4,8);

T(:,:,2) = [
    cos(t1) -sin(t1) 0 0;
    sin(t1) cos(t1) 0 0;
    0 0 1 d1;
    0 0 0 1];

T(:,:,3) = [
    cos(t2-pi/2) -sin(t2-pi/2) 0 0;
    0 0 1 0;
    -sin(t2-pi/2) -cos(t2-pi/2) 0 0;
    0 0 0 1];

T(:,:,4) = [
    cos(t3) -sin(t3) 0 a2;
    sin(t3) cos(t3) 0 0;
    0 0 1 0;
    0 0 0 1];

T(:,:,5) = [
    cos(t4) -sin(t4) 0 a3;
    0 0 1 d4;
    -sin(t4) -cos(t4) 0 0;
    0 0 0 1];

T(:,:,6) = [
    cos(t5) -sin(t5) 0 0;
    0 0 -1 0;
    sin(t5) cos(t5) 0 0;
    0 0 0 1];

T(:,:,7) = [
    cos(t6+pi) -sin(t6+pi) 0 0;
    0 0 1 0;
    -sin(t6+pi) -cos(t6+pi) 0 0;
    0 0 0 1];

T(:,:,8) = [1 0 0 a6 ; 0 1 0 0 ; 0 0 1 dT ; 0 0 0 1];

%% Mass and inertia parameters
%% link 1 parameters

m1 = 3.067;
% inertia
% Ic1x = 0.010453063;
% Ic1y = 0.014216733;
% Ic1z = 0.014405108;
Ic1x = 0.0223798;
Ic1y = 0.0225665;
Ic1z = 0.0104534;
Ic1xy = -0.0000129;
Ic1xz = -0.0000386;
Ic1zy = 0.0000382;

% position of center mass
lc1x = 0.0000977;
lc1y = -0.0001193;
lc1z = -0.0515882;

%% link 2 parameters

m2 = 3.90867;
% inertia
% Ic2x = 0.025938441;
% Ic2y = 0.041585297;
% Ic2z = 0.060311211;
Ic2x = 0.0259893;
Ic2y = 0.1004078;
Ic2z = 0.0816359;
Ic2xy = 0.0003652;
Ic2xz = -0.0016395;
Ic2zy = 0.0000011;

% position of center mass
lc2x = 0.1012432;
lc2y = 0.0007783;
lc2z = -0.0028670;

%% link 3 parameters
m3 =2.94372;

% inertia
% Ic3x = 0.007926185;
% Ic3y = 0.013119995;
% Ic3z = 0.016721225;
Ic3x = 0.0142330;
Ic3y = 0.0182316;
Ic3z = 0.0281164;
Ic3xy = 0.0053169;
Ic3xz = -0.0000008;
Ic3zy = -0.0000087;

% position of center mass
lc3x = 0.0579106;
lc3y = 0.0228077;
lc3z = 0.0010644;

%% link 4 parameters
m4 = 1.32509;

% inertia
% Ic4x = 0.002839056;
% Ic4y = 0.004004123;
% Ic4z = 0.005244937;
Ic4x = 0.0131807;
Ic4y = 0.0119401;
Ic4z = 0.0028397;
Ic4xy = -0.0000132;
Ic4xz = -0.0000581;
Ic4zy = 0.0000402;

% position of center mass
lc4x = 0.0004064;
lc4y = -0.0002253;
lc4z = -0.0773884;

%% link 5 parameters
m5 = 0.54663;
% inertia
% Ic5x = 0.000404884;
% Ic5y = 0.00081547;
% Ic5z = 0.00089283;
Ic5x = 0.0008161;
Ic5y = 0.0004049;
Ic5z = 0.0008935;
Ic5xy = 0.0000008;
Ic5xz = 0;
Ic5zy = 0.0000016;

% position of center mass
lc5x = 0.0000622;
lc5y = -0.0010948;
lc5z = 0.0000369;

%% link 6 parameters
m6 = 0.01368;
% inertia
% Ic6x = 0.000001658;
% Ic6y = 0.000001694;
% Ic6z = 0.000002977;
Ic6x = 0.0000594;
Ic6y = 0.0000593;
Ic6z = 0.0000030;
Ic6xy = 0;
Ic6xz = -0.0000002;
Ic6zy = 0;

% position of center mass
lc6x = -0.0001696;
lc6y = -0.0000013;
lc6z = 0.0649379;

%% link 7 check
m7 = 0.01496;
% inertia
% Ic7x =0.000002522;
% Ic7y = 0.000061371;
% Ic7z = 0.000061886;
Ic7x = 0.0001567;
Ic7y = 0.0001761;
Ic7z = 0.0000224;
Ic7xy = 0;
Ic7xz = 0.0000547;
Ic7zy = 0;

% position of center mass
lc7x = -0.0328898;
lc7y = -0.0000093;
lc7z = -0.0826049;

%% Mass and inertia setup
m = [0 m1 m2 m3 m4 m5 m6 m7]';

Pc = [
    0 lc1x lc2x lc3x lc4x lc5x lc6x lc7x;
    0 lc1y lc2y lc3y lc4y lc5y lc6y lc7y;
    0 lc1z lc2z lc3z lc4z lc5z lc6z lc7z];

Ic = zeros(3,3,8);

Ic(:,:,2) = [
    Ic1x Ic1xy Ic1xz;
    Ic1xy Ic1y Ic1zy;
    Ic1xz Ic1zy Ic1z];

Ic(:,:,3) = [
    Ic2x Ic2xy Ic2xz;
    Ic2xy Ic2y Ic2zy;
    Ic2xz Ic2zy Ic2z];

Ic(:,:,4) = [
    Ic3x Ic3xy Ic3xz;
    Ic3xy Ic3y Ic3zy;
    Ic3xz Ic3zy Ic3z];

Ic(:,:,5) = [
    Ic4x Ic4xy Ic4xz;
    Ic4xy Ic4y Ic4zy;
    Ic4xz Ic4zy Ic4z];

Ic(:,:,6) = [
    Ic5x Ic5xy Ic5xz;
    Ic5xy Ic5y Ic5zy;
    Ic5xz Ic5zy Ic5z];

Ic(:,:,7) = [
    Ic6x Ic6xy Ic6xz;
    Ic6xy Ic6y Ic6zy;
    Ic6xz Ic6zy Ic6z];

Ic(:,:,8) = [
    Ic7x Ic7xy Ic7xz;
    Ic7xy Ic7y Ic7zy;
    Ic7xz Ic7zy Ic7z];

%% Newton Euler Equation

%% Initial values

g = 9.81;
% To include gravity
vdot(:,1) = [0,0,g]';

%% Outward Iteration

thetadot = [0 qdot(1) qdot(2) qdot(3) qdot(4) qdot(5) qdot(6) 0]';
thetaddot = [0 qddot(1) qddot(2) qddot(3) qddot(4) qddot(5) qddot(6) 0];

R = T(1:3, 1:3, :);
P = T(1:3, 4, :);

for j=0:6
    i = j+1;
    
    w(:, i+1) = (R(:,:,i+1)' * w(:,i) + [0 0 thetadot(i+1)]');
    wdot(:, i+1) = (R(:,:,i+1)' * wdot(:, i) + cross (R(:,:,i+1)' * w(:,i), [0 0 thetadot(i+1)]') + [0 0 thetaddot(i+1)]');
    vdot(:, i+1) = (R(:,:,i+1)' * (cross(wdot(:, i), P(:,:, i+1)) + cross(w(:,i), cross(w(:,i), P(:,:, i+1))) + vdot(:, i)));
    vcdot(:, i+1) = (cross(wdot(:, i+1), Pc(:,i+1)) + cross(w(:, i+1), cross(w(:, i+1), Pc(:,i+1))) + vdot(:, i+1));
    
    F(:, i+1) = ((m(i+1) * vcdot(:, i+1)));
    N(:, i+1) = ((Ic(:,:,i+1) * wdot(:,i+1) + cross(w(:,i+1), Ic(:,:,i+1)*w(:,i+1))));
    
    %disp(j)
end

%% Inward Iteration

for j = 6:-1:1
    i = j+1;
    
    f(:,i) = (R(:,:,i+1) * f(:,i+1) + F(:,i));
    n(:,i) = (N(:,i) + R(:,:,i+1)*n(:,i+1) + cross(Pc(:,i), F(:,i)) + cross(P(:,:,i+1), R(:,:,i+1)*f(:,i+1)));
    
    %disp(j)
end

%f = simplify(f);
%n = simplify(n);

tau(1) = n(3,2);
tau(2) = n(3,3);
tau(3) = n(3,4);
tau(4) = n(3,5);
tau(5) = n(3,6);
tau(6) = n(3,7);

end