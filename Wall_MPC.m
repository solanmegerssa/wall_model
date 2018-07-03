%% Sim setup

% graph properties
S = 6; % number of arcs
N = 7; % number of nodes
T = 2; % number of tanks

% node-arc incidence matrix
A = [1 0 0 0 0 0; -1 1 0 0 0 0; 0 -1 1 0 0 0; 
    0 0 -1 -1 0 0; 0 0 0 0 0 1; 0 0 0 0 1 -1; 0 0 0 1 -1 0];

% pump-arc incidence matrix
B = [0 0 0 0 0 1];

% reservoir-node incidenc matrix
M = [1 0 0 0 0 0 0; 0 0 0 0 1 0 0];

% tank-node incidence matrix
lambda = [0 1 0 0 0 0 0; 0 0 0 0 0 0 1];

% tank parameters
v1 = 100.0; % gal
a2 = .25; % m^2
v3 = 100.0; % gal
V0 = [75; 75];
max_inflow = 20; % gpm
max_outflow = 20; % gpm

% tank-pressure relation matrix
D1 = [v1/60 0; 0 v3/60];
D2 = [-15; -15];

% pipe parameters
pipe_d = .5; % in
c = 130^1.852;
d = pipe_d^4.8704;
L = [10 2 3 3 1 1]'; % length of pipes [ft]
G = 4.52/(c*d)*diag(L);
max_head = 100; % psi
min_head = 10; % psi
max_pipeflow = 30; % gpm

% horizon parameters
Tf = 23*60; % minutes
step = .5; % minutes
Nf = Tf/step;

% cost function params
PD_water = .03; % kw/gallon
phi_water = .2; % $/gal
cost_file = csvread('June28_CAISOAVERAGEPRICE.csv', 1, 1);
t = 0:5:(24*60-5);
ts = step:step:23*60;
phi_e = interp1(t,cost_file,ts);

% water properties
rho = 998; % kg/m^2
g = 9.8; % m/s^2


%% Optimization

% Random demand
t_hours = 0:60:23*60;
demand = [0 0 0 0 1 4 11 12 8 8 11 5 2 3 5 7 9 6 8 6 6 3 2 1]; % hourly demand gph
demand_minute = interp1(t_hours, demand, ts)/60 + normrnd(2,0.3,1,Nf);
demand_minute(demand_minute<0) = 0;

% decision variables and initial condition
v_cist0 = sdpvar(1,1); % initial grey cistern vol
vp0 = sdpvar(2,1); % initial pressure tank vol
H = sdpvar(N,Nf); % pressure head
Q = sdpvar(S,Nf); % pipe flowrate
C = sdpvar(T,Nf); % water inflow
D = sdpvar(T,Nf); % water outflow

W_pull = sdpvar(N,Nf); % inflow from util and grey
W_waste = sdpvar(N,Nf); % waste outflow
W_rec = sdpvar(N,Nf); % recycle flow
P_pump = sdpvar(1,Nf); % pump power [kw]
W_demand = sdpvar(N,Nf);

% pipe linearization variables
J = 2;
lam = sdpvar(S,J,Nf);
q = sdpvar(S,J,Nf);
al = sdpvar(S,J,Nf);

%%
% mpc
constraints = [];
objective = 0;
v = vp0;
v_cist = v_cist0;
for k = 1:5 % need to change to Nf
    
    % volume evolution
    v = v + C(:,k) - D(:,k);
    
%     if k > 1
%         inflow = M*W_rec{k-1};
%         v_cist = v_cist + inflow(2);
%     end
    
    % amount of recycle flow
    %W_rec(:,k) = .9*W_demand(:,k);
    
    % objective optimizes for cost per unit water (cost of utility water,
    % cost to filter, cost of waste)
    objective = objective + W_pull(1,k)*phi_water + P_pump(1,k)*phi_e(k) + W_waste(1,k)*phi_e(k);
    
    % constraints
    constraints = [constraints,
        
        % tanks
        v == D1*lambda*H(:,k) + D2,
        [0; 0] <= v <= [v1; v3],
        0 <= v_cist <= 50,
        0 <= C(:,k) <= max_inflow,
        0 <= D(:,k) <= max_outflow,
        
        % pressure at utility and grey cistern'
%         M*H == [50; v_cist*rho*g/(a2*6894.76)],
        M*H(:,k) == [50; 40],
        
        % pressure loss in pipes
        H(:,k) >= 0,
        -max_pipeflow <= Q(:,k) <= max_pipeflow,
        Q(:,k) == sum(q(:,:,k).*lam(:,:,k),2),
        -A'*H(:,k) == sum(G*sign(q(:,:,k)).*q(:,:,k).^1.852.*lam(:,:,k),2),
        sum(al(:,J-1,k),2) == 1,
        sum(lam(:,:,k),2) == 1,
        lam(:,1,k) <= al(:,1,k),
        lam(:,J,k) <= al(:,J-1,k),
        0 <= lam(:,:,k) <= 1,
        
        % pumps
        70 <= B*A'*H(:,k) <= 80,
        P_pump(:,k) == B*Q(:,k)*B*A'*H(:,k)/435 + 70*10^-3,
        
        % water conservation
        W_waste([1:5,7],k) == zeros(6,1),
%         W_waste(:,k)(6) >= 0.5*B*Q,
        W_waste(6,k) >= 0,
        
        W_demand(3,k) == demand_minute(k)*.3,
        W_demand(4,k) == demand_minute(k)*.7,
        W_demand([1:2,5:7],k) == zeros(5,1),
        
        W_pull([2:4,6:7],k) == zeros(5,1),
        W_pull([1,5],k) >= zeros(2,1),
        W_pull(:,k) - W_demand(:,k) - W_waste(:,k) == A*Q(:,k) + lambda'*(C(:,k)-D(:,k))]
        
    for j = 2:J-1
        constraints = [constraints, lam(:,j) <= al(:,j-1) + al(:,j)]
    end
end

%% optimize
optimize([constraints, v_cist0 == 25, vp0 == V0], objective); 

%% test
for i = 1:5
    display('Head')
    value(H(:,i))'
    display('')
    display('Flow rate')
    value(Q(:,i))'
    display('')
    display('volume')
    value(v)
    display('')
    display('W pull')
    value(W_pull(:,i))'
    display('')
    display('W demand')
    value(W_demand(:,i))'
    display('')
    display('W waste')
    value(W_waste(:,i))'
    display('Pump power')
    value(P_pump(:,i))
    display('tank inflow')
    value(C(:,i))
    display('tank draw')
    value(D(:,i)) 
end
display('Objective')
value(objective)