%% Sim setup

% graph properties
S = 3; % number of arcs
N = 4; % number of nodes
T = 1; % number of tanks

% node-arc incidence matrix
A = [1 0 0; -1 1 0; 0 -1 -1; 0 0 1];

% pump-arc incidence matrix
B = [0 0 1];

% reservoir-node incidenc matrix
M = [1 0 0 0; 0 0 0 1];

% tank-node incidence matrix
lambda = [0 1 0 0];

% tank parameters
v1 = 100.0; % gal
v0 = 75; % gal
max_inflow = 20; % gpm
max_outflow = 20; % gpm

% tank-pressure relation matrix
D1 = v1/60;
D2 = -15;

% pipe parameters
pipe_d = .5; % in
d = pipe_d^4.8704;
L = [10 2 2]'; % length of pipes [ft]
max_head = 100; % psi
min_head = 10; % psi
max_pipeflow = 30; % gpm

% horizon parameters
Tf = 23*60; % minutes
step = .5; % minutes
Nf = Tf/step;

% cost function params
PD_water = .03; % kw/gallon
phi_water = 0.2; % $/gal
cost_file = csvread('SimData/June28_CAISOAVERAGEPRICE.csv', 1, 1);
t = 0:5:(24*60-5);
ts = step:step:23*60;
phi_e = interp1(t,cost_file,ts);

% water properties
rho = 998; % kg/m^2
g = 9.8; % m/s^2

% linearized pipe parameters
h0 = 1/.43*[60 55 50 60]'; % m
dh0 = A'*h0; % m
c = 1.22E10;
e1 = 1.85;
e2 = 4.87;
e3 = 1/e1 - 1;
e4 = e3 - 1;
rk = c*L*.3048/(130^e1*6.35^e2);
gk = rk.^(-1/e1);
q0 = gk.*abs(dh0).^e3.*sign(dh0); % L/s
q0_gal = 15.8*q0

% pipe conductance matrix
g_branch = e3*gk.*abs(dh0).^e4;
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
demand_minute = interp1(t_hours, demand, ts)/60; %  normrnd(0.5,0.1,1,Nf)
demand_minute(demand_minute<0) = 0;

% decision variables and initial condition
vp0 = sdpvar(1,1); % initial pressure tank vol
H = sdpvar(N,Nf); % pressure head [m]
Q = sdpvar(S,Nf); % pipe flowrate [l/s]
C = sdpvar(T,Nf); % water inflow [gal/min]
D = sdpvar(T,Nf); % water outflow [gal/min]
v_record = sdpvar(T,Nf); % record of tank vol [gal]
O = sdpvar(N,N,Nf); % valve variables

W_pull = sdpvar(N,Nf); % inflow from util and grey
W_rec = sdpvar(N,Nf); % recycle flow
P_pump = sdpvar(1,Nf); % pump power [kw]
W_demand = sdpvar(N,Nf);


%%
% mpc
constraints = [];
objective = 0;
v = vp0;
%v_cist = v_cist0;

for k = 1:20 % need to change to Nf
    
    % volume evolution
    v = v + step*(C(:,k) - D(:,k));
    
    % objective optimizes for cost per unit water (cost of utility water,
    % cost to filter, cost of waste)
    objective = objective + W_pull(1,k)*phi_water + W_pull(4,k)*phi_water*3 ; % abs((.43*H(3,k) - 50)) + P_pump(1,k)*phi_e(k)
    
    % constraints
    constraints = [constraints,
        
        % tanks
        v == D1*lambda*.433*H(:,k) + D2,
        0 <= v <= v1,
        0 <= C(:,k) <= max_inflow,
        0 <= D(:,k) <= max_outflow,
        
        % pressure at utility and grey cistern'
        .433*M*H(:,k) == [60; 60],
        
        % pressure loss in pipes
        H(:,k) >= 0,
        abs(.433*H(2,k) - 50) <= 10,
        %abs(.433*H(3,k) - 50) <= 10,
        A*Q(:,k) - A*q0 == O(:,:,k).*G*(H(:,k)-h0),       
        Q(2,k) >= 0,

        % valves
        diag(O(:,:,k)) == 1,
        0 <= diag(O(:,:,k),1) <= 1,
        0 == diag(O(:,:,k),2),
        0 == diag(O(:,:,k),3),
        
        % pumps
%         40 <= B*A'*0.433*H(:,k) <= 80,
%         P_pump(:,k) == B*Q(:,k)*B*A'*.433*H(:,k)/435 + 70*10^-3,
        
        % record water levels
        v_record(:,k) == v,
        
        W_demand(3,k) == demand_minute(k),
        W_demand([1:2, 4],k) == zeros(3,1),
        
        W_pull([2:3],k) == zeros(2,1),
        W_pull([1,4],k) >= 0,
%         W_pull(:,k) <= max_inflow
        W_pull(:,k) - W_demand(:,k) == A*15.85*Q(:,k) + lambda'*(C(:,k)-D(:,k))]
end

%% optimize
optimize([constraints, vp0 == v0], objective); 

%% plots
figure
plot(1:k,value(.433*H(:,1:k)), 'LineWidth',2)
title("Pressure (psi)")
legend("utility","potable tank","combine junction 1","recycle cistern")

figure
plot(1:k,15.8*value(Q(:,1:k)),'LineWidth',2)
title("Flows (gpm)")
legend("utility-potable tank","potable-combine1","recycle-combine")

figure
plot(1:k,value(W_pull([1,4],1:k)),'LineWidth',2)
title("Water pulled (gpm)")
legend("utility pull","recycle pull")

figure
plot(1:k,value(W_demand([3 4],1:k)),'LineWidth',2)
title("Water demand (gpm)")
legend("potable", "recycled")

% figure
% plot(1:k,.0013*value(P_pump(:,1:k)),'LineWidth',2)
% title("Pump power (HP)")

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

figure
plot(1:k,squeeze(value(O(1,2,1:k))), 1:k,squeeze(value(O(2,3,1:k))),1:k,squeeze(value(O(3,4,1:k))), 'LineWidth',2)
title("Valve Open")
legend("utility-potable tank","potable-combine1","recycle-combine")
