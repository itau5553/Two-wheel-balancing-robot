% System model
A = [0 1 0 0;
     0 -1.23529412e-02 4.32794118e+00 0;
     0 0 0 1;
     0 -8.82352941e-03 1.00985294e+01 0];
B = [0;
     0.04117647;
     0;
     0.02941176];
C = [1 0 0 0;
     0 0 1 0];
D = [0;
     0];

% Reachability/Controllability matrix
W = ctrb(A, B);

% Canonical form
At = [0 1 0;
      0 0 1;
      -8.82352941e-03 1.00985294e+01 0];
Bt = [0;
      0;
      0.02941176];
% Reachability/Controllability matrix for canonical form
Wt = ctrb(At, Bt);

% Desired CP
z = 0.7;
ts = 2.8;
w = 3.92 / (z * ts);
N = 5; % Here we put it 5 times further left
p1 = N * z * w + 2 * z * w;
p2 = 2 * N * z^2 * w^2 + w^2;
p3 = N * z * w^3;

% Coefficient matching
Kt = [p1, p2, p3] + At(1, :);

% Calculate transformation matrix to canonical form
T = Wt / W;

% Controller gains u = -Kx + kr
K = Kt * T;           % State gain
kr = -1 / (C / (A - B * K) * B); % Feedforward gain