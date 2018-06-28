%% Sim setup

% graph properties
S = 6; % number of arcs
N = 7; % number of nodes
T = 3; % number of tanks

% node-arc incidence matrix
A = [1 0 0 0 0 0; -1 1 0 0 0 0; 0 -1 1 0 0 0; 
    0 0 -1 -1 0 0; 0 0 0 0 0 1; 0 0 0 0 1 -1; 0 0 0 1 -1 0];

% pump-arc incidence matrix
B = [0 0 0 0 0 1];

% utility-node incidenc matrix
M = [1 0 0 0 0 0 0];

% tank-node incidenc matrix
lambda = [0 1 0 0 0 0 0; 0 0 0 0 1 0 0; 0 0 0 0 0 0 1];

% tank parameters
v1 = 100.0; % gal
a2 = .25; % m^2
v2 = 100.0; % gal
V0 = [75; 10; 75];
max_inflow = 20; % gpm
max_outflow = 20; % gpm

% tank-pressure relation matrix
D1 = [v1/60 0 0; 0 6894.76*a2/(9.81*997) 0; 0 0 v2/60];
D2 = [-15; 0; -15];

% pipe parameters
pipe_d = .5; % in
c = 130^1.852;
d = pipe_d^4.8704;
L = [10 2 3 3 1 1]'; % length of pipes [ft]
G = 4.52/(c*d)*diag(L);
max_head = 100; % psi
min_head = 10; % psi
max_pipeflow = 30; % gpm

% cost function
PD_water = .03; % kw/gallon
phi_water = .2; % $/gal
phi_e = ;


%% Optimization

% horizon parameters
Tf = 24*60; % minutes
step = .5; % minutes
Nf = Tf/step;

% decision variables and initial condition
v0 = sdpvar(1,3); % initial tank vol
H = sdpvar(N,1); % pressure head
Q = sdpvar(S,1); % pipe flowrate
C = sdpvar(repmat(T,1,Nf),repmat(1,1,Nf)); % water inflow
D = sdpvar(repmat(T,1,Nf),repmat(1,1,Nf)); % water outflow
W_util = sdpvar(repmat(1,1,Nf),repmat(1,1,Nf)); % utility inflow

% pipe linearization variables
J = 5;
lam = sdpvar(S,J);
q = sdpvar(S,J);
al = sdpvar(S,J);


% mpc
constraints = [];
objective = 0;
v = v0;
for k = 1:N
    
    % state evolution
    v = v + C{k} - D{k};
    
    objective = objective + norm(W_util{k}*phi_water,1) + norm(P_pump{k}*phi_e,1) + norm(W_waste{k}*phi_water,1);
    
    % constraints
    constraints = [constraints,
        
        % tanks
        v == D1*lambda*H + D2,
        [0; 0; 0] <= v <= [v1; 50; v2],
        0 <= C{k} <= max_inflow,
        0 <= D{k} <= max_outflow,
        
        % pressure loss
        Q == sum(q.*lam,2),
        -A'*H == sum(G*sign(q)*q^1.852*lam,2),
        sum(al(:,J-1),2) == 1,
        sum(lam,2) == 1,
        %% add constraint
        lam(:,1) <= al(:,1),
        lam(:,J) <= al(:,J-1)',
        0 <= lam <= 1,
        
        % pumps
        70 <= B*A'*H <= 80,
        
        
        
        
    
    
