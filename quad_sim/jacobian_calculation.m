% define the symbols of state
syms x y z xdot ydot zdot phi theta psi omega1 omega2 omega3
% omega1,2,3 represent the angular velocity vector

% define the symbols of dynamics
syms phidot thetadot psidot

% define the input
syms gamma1 gamma2 gamma3 gamma4

% define the delta input
syms delta1 delta2 delta3 delta4

% define the properties of the quadcopter
syms m I g kd k L b
% m=0.2, g=9.8, kd=0.1, k=1, L=0.2, b=0.1

% define the quadcopter rotational inertia matrix
I = [1,0,0;0,1,0;0,0,0.5];

% initialise the input
inputs = [gamma1;gamma2;gamma3;gamma4];
inputs_0 = [0.2*9.8/4; 0.2*9.8/4; 0.2*9.8/4; 0.2*9.8/4];
delta_inputs = [delta1;delta2;delta3;delta4];

% define the state
state_1 = [x;y;z];
state_2 = [xdot;ydot;zdot];
state_3 = [phi;theta;psi];
state_4 = [omega1;omega2;omega3];

state = [state_1;state_2;state_3;state_4];

% define the dynamics
% define the rotation matrix
R = [cos(psi)*cos(theta), cos(psi)*sin(phi)*sin(theta)-cos(phi)*sin(psi), sin(phi)*sin(psi)+cos(phi)*cos(psi)*sin(theta);
    cos(theta)*sin(psi), cos(phi)*cos(psi)+sin(phi)*sin(psi)*sin(theta), cos(phi)*sin(psi)*sin(theta)-cos(psi)*sin(phi);
    -sin(theta), cos(theta)*sin(phi), cos(phi)*cos(theta)];
% Rx = [1 0 0;
%       0 cos(phi) -sin(phi);
%       0 sin(phi) cos(phi)];
% 
% Ry = [cos(theta) 0 sin(theta);
%       0 1 0;
%       -sin(theta) 0 cos(theta)];
% 
% Rz = [cos(psi) -sin(psi) 0;
%       sin(psi) cos(psi) 0;
%       0 0 1];
% 
% R = Rz * Ry * Rx;

%TB = [0;0;k*sum(inputs,'all')];
TB = [0;0;1*sum(inputs,'all')];

%Fd = -kd * state_2;
Fd = -0.1 * state_2;

matrix = [1,0,-sin(theta);
          0,cos(phi),cos(theta)*sin(phi);
          0,-sin(phi),cos(theta)*cos(phi)];

tauB = [0.2*1*(gamma1-gamma3);
        0.2*1*(gamma2-gamma4);
        0.1*(gamma1-gamma2+gamma3-gamma4)];

dynamics_1 = state_2;
dynamics_2 = [0;0;-9.8] + R*TB/0.2 + Fd/0.2;
dynamics_3 = inv(matrix) * state_4;
dynamics_4 = inv(I) * (tauB - cross(state_4, (I*state_4)));

dynamics = [dynamics_1;dynamics_2;dynamics_3;dynamics_4];

dynamics = subs(dynamics,[gamma1,gamma2,gamma3,gamma4],[delta_inputs(1)+0.2*9.8/4, delta_inputs(2)+0.2*9.8/4, delta_inputs(3)+0.2*9.8/4, delta_inputs(4)+0.2*9.8/4]);

% calculate the jacobian
A_jacobian = jacobian(dynamics, state);
B_jacobian = jacobian(dynamics, delta_inputs);
% B_jacobian = jacobian(dynamics, inputs);

A = double(subs(A_jacobian,[phi, theta, psi, phidot, thetadot, psidot, omega1, omega2, omega3, delta1, delta2, delta3, delta4],[0,0,0,0,0,0,0,0,0,0,0,0,0]));
B = double(subs(B_jacobian, [phi, theta, psi, omega1, omega2, omega3, gamma1, gamma2, gamma3, gamma4], [0,0,0,0,0,0,0,0,0,0]));
C = eye(12);
D = zeros(12,4);

cont_sys = ss(A,B,C,D);
    
disc_sys = c2d(cont_sys,0.02,'zoh');

A_d = disc_sys.A;
B_d = disc_sys.B;

Q = diag([1,1,1,1,1,1,1,1,1,1,1,1]);
R = 1;

% Q = [1/(5^2) 0 0 0 0 0 0 0 0 0 0 0;
%      0 1/(7.5^2) 0 0 0 0 0 0 0 0 0 0;
%      0 0 1/(5^2) 0 0 0 0 0 0 0 0 0;
%      0 0 0 0.01 0 0 0 0 0 0 0 0;
%      0 0 0 0 0.01 0 0 0 0 0 0 0;
%      0 0 0 0 0 0.01 0 0 0 0 0 0;
%      0 0 0 0 0 0 1 0 0 0 0 0;
%      0 0 0 0 0 0 0 1 0 0 0 0;
%      0 0 0 0 0 0 0 0 1/((2*pi)^2) 0 0 0;
%      0 0 0 0 0 0 0 0 0 1 0 0;
%      0 0 0 0 0 0 0 0 0 0 1 0;
%      0 0 0 0 0 0 0 0 0 0 0 1];
% R = eye(4)*(1/(0.49*1.15)^2);

K = dlqr(A_d, B_d, Q, R);

save('jacobian');


