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
%G = 4.52/(c*d)*diag(L);
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
cost_file = csvread('SimData/June28_CAISOAVERAGEPRICE.csv', 1, 1);
t = 0:5:(24*60-5);
ts = step:step:23*60;
phi_e = interp1(t,cost_file,ts);

% water properties
rho = 998; % kg/m^2
g = 9.8; % m/s^2

% linearized pipe parameters
h0 = [5 0.5 0.5 1 0.5 20]';
c = 1.22E10;
e1 = 1.85;
e2 = 4.87;
e3 = 1/e1 - 1;
e4 = e3 - 1;
rk = c*L*.3048/(130^e1*6.35^e2);
gk = rk.^(-1/e1);
g_branch = e3*gk.*h0.^e4;
g_node = abs(A)*g_branch;

g_branch = diag(g_branch);
g_branch = -abs(A)*g_branch;

G = diag(g_node);
for m = 2:N-1
    G(m,[1:(m-1), (m+1):N]) = g_branch(m,:);
end
G(1,2:N) = g_branch(1,:);
G(N,(1:N-1)) = g_branch(N,:);

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
v_record = sdpvar(2,Nf);

W_pull = sdpvar(N,Nf); % inflow from util and grey
W_waste = sdpvar(N,Nf); % waste outflow
W_rec = sdpvar(N,Nf); % recycle flow
P_pump = sdpvar(1,Nf); % pump power [kw]
W_demand = sdpvar(N,Nf);


%%
% mpc
constraints = [];
objective = 0;
v = vp0;
v_cist = v_cist0;

for k = 1:10 % need to change to Nf
    
    % volume evolution
    v = v + step*(C(:,k) - D(:,k));
%     
%     if k > 1
%         inflow = M*W_rec(:,k-1);
%         v_cist = v_cist + inflow(2);
%     end
%     
%     % amount of recycle flow
%     W_rec(:,k) = .9*W_demand(:,k);
    
    % objective optimizes for cost per unit water (cost of utility water,
    % cost to filter, cost of waste)
    objective = objective + W_pull(1,k)*phi_water + P_pump(1,k)*phi_e(k) + W_waste(6,k)*phi_water;
    
    % constraints
    constraints = [constraints,
        
        % tanks
        v == D1*lambda*.433*H(:,k) + D2,
        [0; 0] <= v <= [v1; v3],
        0 <= v_cist <= 50,
        0 <= C(:,k) <= max_inflow,
        0 <= D(:,k) <= max_outflow,
        
        % pressure at utility and grey cistern'
%         M*H == [50; v_cist*rho*g/(a2*6894.76)],
        .433*M*H(:,k) == [60; 40],
        
        % pressure loss in pipes
        H(:,k) >= 0,
        H([2,7],k) >= 50,
        A*Q(:,k) == G*0.433*H(:,k),
        Q(3,k) >= 0,
        
        % pumps
        70 <= B*A'*0.433*H(:,k) <= 80,
        P_pump(:,k) == B*Q(:,k)*B*A'*.433*H(:,k)/435 + 70*10^-3,
        
        % record water levels
        v_record(:,k) == v,
        
        % water conservation
        W_waste([1:5,7],k) == zeros(6,1),
        W_waste(6,k) >= 0 %0.5*B*Q,
        
        W_demand(3,k) == demand_minute(k)*.3,
        W_demand(4,k) == demand_minute(k)*.7,
        W_demand([1:2,5:7],k) == zeros(5,1),
        
        W_pull([2:4,6:7],k) == zeros(5,1),
        W_pull([1,5],k) >= zeros(2,1),
        W_pull(:,k) - W_demand(:,k) - W_waste(:,k) == A*Q(:,k) + lambda'*(C(:,k)-D(:,k))]
end

%% optimize
optimize([constraints, v_cist0 == 25, vp0 == V0], objective); 

%% plots
figure
plot(1:k,value(.433*H(:,1:k)), 'LineWidth',2)
title("Pressure (psi)")
legend("utility","potable tank","combine junction 1","combine junction 2","grey cistern","RO-waste", "recycle tank")

figure
plot(1:k,value(Q(:,1:k)),'LineWidth',2)
title("Flows (gpm)")
legend("utility-potable tank","potable-combine1","combine1-combine2","recycle tank-combine2","RO-recycle tank","grey cistern-RO")

figure
plot(1:k,value(W_pull([1,5],1:k)),'LineWidth',2)
title("Water pulled (gpm)")
legend("utility pull","grey pull")

% figure
% plot(1:k,value(W_demand([3 4],1:k)),'LineWidth',2)
% title("Water demand (gpm)")
% legend("potable", "recycled")

figure
plot(1:k,value(W_waste(6,1:k)),'LineWidth',2)
title("Waste flow (gpm)")

% figure
% plot(1:k,value(P_pump(:,1:k)),'LineWidth',2)
% title("Pump power (Watts)")

% figure
% plot(1:k,value(C(:,1:k)),'LineWidth',2)
% title("Tank inflow")
% legend("potable","recycled")
% 
% figure
% plot(1:k,value(D(:,1:k)),'LineWidth',2)
% title("Tank outflow")
% legend("potable","recycled")

figure
plot(1:k,value(C(:,1:k))-value(D(:,1:k)),'LineWidth',2)
title("Net tank flow")
legend("potable","recycled")

figure
plot(1:k,value(v_record(:,1:k)),'LineWidth',2)
title("Tank volumes")
legend("potable", "recycled")
