%% LQR Design for "Two Wheel SBR"
% Adapted from poleplace_cartpend.m / lqr_cartpend.m (Brunton cart-pendulum model)
%
% STATE VECTOR CONVENTION (this matters for the firmware transplant):
%   y = [x; dx; theta; dtheta]
%     x      = wheel/robot horizontal position (m)          [not measured on this robot]
%     dx     = horizontal velocity (m/s)                    [known exactly from commanded step rate]
%     theta  = body tilt from vertical, UPRIGHT = 0 (rad)   <-- matches complementary filter output directly
%     dtheta = tilt rate (rad/s)                             <-- matches gyro Y output directly
%
% Because s = 1 (upright linearization) was already used when this A,B pair was
% derived, theta here already means "deviation from vertical," not "angle from
% hanging down." That's why you can hand theta/dtheta straight from your MPU6500
% complementary filter into -K*y without any pi-offset. The pi-offset you see in
% lqr_cartpend.m (y0 = [-3;0;pi+.1;0], and -K*(y-[1;0;pi;0])) exists ONLY because
% cartpend.m's *nonlinear* ODE (used for the MATLAB animation) measures the
% pendulum angle from hanging-down. Your firmware never touches that nonlinear
% ODE, so it never needs that offset.

clear all, close all, clc

%% ---- PHYSICAL PARAMETERS: measured off the real robot ----
% Cart-pendulum analogy mapping for a two-wheel balancer:
%   m = mass of the part that TIPS OVER  -> the robot BODY (chassis + battery + electronics)
%   M = mass of the part that TRANSLATES -> the WHEEL/DRIVE assembly (steppers + wheels + brackets)
%
% Measured on GODARC BOT: the drive assembly (1100 g) is actually HEAVIER than the
% body (900 g) -- opposite of the usual "heavy body, light cart" assumption for
% these robots, since the two NEMA17s + brackets outweigh the acrylic body shell
% and electronics. The model doesn't care which is heavier, it just needs the
% real numbers, so these are used as-is below.

m  = 0.900;    % body mass (kg) -- body/pendulum, measured
M  = 1.100;    % effective translating mass (kg) -- 2x NEMA17 + wheels + brackets, measured
L  = 0.16;     % distance from wheel axle to body center of mass (m).
               % Measure with the battery mounted where it actually sits (bottom of chassis
               % per current build) -- L changes if the battery position changes, and L
               % directly scales the gravity-torque term below, so re-measure after any
               % mass redistribution.
g  = -9.81;    % KEEP THIS NEGATIVE. It is baked into the sign convention of the A matrix
               % below (same convention as cartpend.m) -- it is not the literal direction
               % of gravity, and flipping the sign will flip the stabilizing direction of K.
d  = 0.02;     % translational damping / rolling friction (N.s/m) -- start small, tune
               % experimentally by comparing simulated vs. real settling behavior.

s = 1; % linearizing about the UPRIGHT equilibrium (this robot never balances hanging down)

A = [0 1 0 0;
     0 -d/M -m*g/M 0;
     0 0 0 1;
     0 -s*d/(M*L) -s*(m+M)*g/(M*L) 0];

B = [0; 1/M; 0; s*1/(M*L)];

disp('Open-loop eigenvalues (expect one unstable real pole -- that''s the fall mode):')
eig(A)

disp('Controllability rank (need 4 for full state):')
rank(ctrb(A,B))

%% ---- Continuous-time LQR (full 4-state, requires a position measurement) ----
Q = diag([1, 1, 10, 100]);   % [x, dx, theta, dtheta] -- theta/dtheta weighted hardest
R = 0.0001;

K = lqr(A,B,Q,R);
disp('Continuous full-state K = [Kx, Kdx, Ktheta, Kdtheta]:'); disp(K)

%% ---- Discrete-time LQR, matched to your ESP32 loop rate ----
% Continuous K is only exact for a continuous controller. Your firmware runs a
% discrete loop, so discretize the plant at your actual loop period and use dlqr --
% this gives gains that are correct for the sample-and-hold reality of a
% microcontroller, not just an approximation of the continuous ones.

Ts = 0.005;   % <-- SET THIS to your measured loop period (e.g. 0.005 = 200 Hz).
              % Print micros() deltas in your main loop and use the real average, not a guess.

sysc = ss(A,B,eye(4),0);
sysd = c2d(sysc,Ts);
[Ad,Bd] = ssdata(sysd);
Kd = dlqr(Ad,Bd,Q,R);
disp('Discrete full-state Kd (use these if you add position feedback later):'); disp(Kd)

%% ---- Reduced 3-state LQR: [dx, theta, dtheta] -- NO position state ----
% Your hardware has no absolute position sensor (no wheel encoders), and there is
% no need for one just to balance: only dx (velocity), theta, and dtheta are
% needed to keep the robot upright and prevent it from running away. Practically,
% dx should come from your COMMANDED step rate (exact, since the stepper is
% open-loop) rather than from double-integrating the accelerometer, which drifts
% badly over the timescales this loop cares about.

A3 = A(2:end,2:end);
B3 = B(2:end);
Q3 = Q(2:end,2:end);

disp('Reduced-model controllability rank (need 3):')
rank(ctrb(A3,B3))

K3  = lqr(A3,B3,Q3,R);
sysc3 = ss(A3,B3,eye(3),0);
sysd3 = c2d(sysc3,Ts);
[Ad3,Bd3] = ssdata(sysd3);
Kd3 = dlqr(Ad3,Bd3,Q3,R);

disp('Continuous reduced K3  = [Kdx, Ktheta, Kdtheta]:'); disp(K3)
disp('Discrete   reduced Kd3 = [Kdx, Ktheta, Kdtheta]  <-- paste this row into firmware:'); disp(Kd3)

%% ---- Optional: sanity-check step in simulation before flashing ----
% Uses your existing cartpend.m / drawcartpend_bw.m. Recall cartpend.m's nonlinear
% ODE measures the pendulum angle from hanging DOWN, so theta=pi is upright there
% -- this offset is a simulation-only artifact, see note at top of file.

tspan = 0:.005:6;
y0 = [0; 0; pi+0.15; 0];   % ~8.6 deg initial tilt, simulation coordinates
[t,y] = ode45(@(t,y) cartpend(y,m,M,L,g,d, -K*(y-[0;0;pi;0])), tspan, y0);

figure
plot(t, y(:,3)-pi, 'LineWidth', 1.5); grid on
xlabel('time (s)'); ylabel('\theta deviation from upright (rad)');
title('Simulated recovery from initial tilt (full-state continuous K)');

for k=1:2:length(t)
    drawcartpend_bw(y(k,:),m,M,L);
end