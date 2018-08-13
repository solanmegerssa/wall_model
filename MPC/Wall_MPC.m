%% Sim setup

% graph properties
S = 8; % number of arcs
N = 9; % number of nodes
T = 2; % number of tanks

% node-arc incidence matrix
A = [1 0 0 0 0 0 0 0; -1 1 0 0 0 0 0 0; 0 -1 1 0 0 0 1 0; 
    0 0 -1 -1 0 0 0 1; 0 0 0 0 0 1 0 0; 0 0 0 0 1 -1 0 0;
    0 0 0 1 -1 0 0 0; 0 0 0 0 0 0 -1 0; 0 0 0 0 0 0 0 -1];

% pump-arc incidence matrix
B = [0 0 0 0 0 1 0 0];

% reservoir-node incidenc matrix
M = [1 0 0 0 0 0 0 0 0; 0 0 0 0 1 0 0 0 0];

% tank-node incidence matrix
lambda = [0 1 0 0 0 0 0 0 0; 0 0 0 0 0 0 1 0 0];

% tank parameters
v1 = 100.0; % gal
a2 = .25; % m^2
v3 = 100.0; % gal
V0 = [50; 50];
max_inflow = 20; % gpm
max_outflow = 20; % gpm

% tank-pressure relation matrix
D1 = [v1/60 0; 0 v3/60];
D2 = [-15; -15];

% pipe parameters
pipe_d = .5; % in
d = pipe_d^4.8704;
L = [10 2 3 3 1 1 10 10]'; % length of pipes [ft]
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
h0 = 1/.43*[60 50 48 45 50 61 59 0 0]'; % m
dh0 = A'*h0; % m
c = 1.22E10;
e1 = 1.85;
e2 = 4.87;
e3 = 1/e1 - 1;
e4 = e3 - 1;
rk = c*L*.3048/(130^e1*6.35^e2);
gk = rk.^(-1/e1);
q0 = abs(dh0).^(1/e1-1).*sign(dh0).*gk;
q0_gal = 15.8*q0;

% pipe conductance matrix
g_branch = e3*gk.*abs(dh0).^e4;
g_node = abs(A)*g_branch;
g_branch = diag(g_branch);
g_branch = -abs(A)*g_branch;

G = diag(g_node);
for i = 1:S
    j1 = 0;
    for j = 1:N
        if g_branch(j,i) ~= 0
            if j1 == 0
                j1 = j;
            else
                j2 = j;
            end
        end
    end
    G(j1,j2) = g_branch(j1,i);
    G(j2,j1) = g_branch(j1,i);
end
G


% Random demand
t_hours = 0:60:23*60;
demand = [0 0 0 0 1 4 11 12 8 8 11 5 2 3 5 7 9 6 8 6 6 3 2 1]; % hourly demand gph
demand_minute = interp1(t_hours, demand, ts)/60;
demand_minute(demand_minute > 0.01) = demand_minute(demand_minute > 0.01) + normrnd(0.5, .25, [size(demand_minute(demand_minute > 0.01))]);
demand_minute(demand_minute<0) = 0;

% decision variables and initial condition
v_cist0 = sdpvar(1,1); % initial grey cistern vol
% vp0 = sdpvar(T,1); % initial pressure tank vol
v = sdpvar(T,Nf);
H = sdpvar(N,Nf); % pressure head [m]
Q = sdpvar(S,Nf); % pipe flowrate [l/s]
C = sdpvar(T,Nf); % water inflow [gal/min]
D = sdpvar(T,Nf); % water outflow [gal/min]
v_record = sdpvar(T,Nf); % record of tank vol [gal]
O = sdpvar(N,N,Nf); % valve variables

W_pull = sdpvar(N,Nf); % inflow from util and grey
W_rec = sdpvar(N,Nf); % recycle flow
P_pump = sdpvar(1,Nf); % pump power [kw]
W_demand = sdpvar(N,Nf); % water demand [gal/min]
W_waste = sdpvar(N,Nf); % waste from RO [gal/min]


%% solve relaxed problem (no pump power or pressure at tank constraints)
constraints = [];
objective = 0;
% v = vp0;
%v_cist = v_cist0;

for k = 1:50 % need to change to Nf
    
    % volume evolution
%     v = v + step*(C(:,k) - D(:,k));
    
    % objective optimizes for cost per unit water (cost of utility water,
    % cost to filter, cost of waste)
    objective = objective + W_pull(1,k)*phi_water + 50*W_pull(5,k)*phi_e(k+1000)*PD_water;
    
    O_k = O(:,:,k);
    % constraints
    constraints = [constraints,
        
        % tanks
%         v == D1*lambda*.433*H(:,k) + D2,

        v(:,k+1) == v(:,k) + step*(C(:,k) - D(:,k)),
        v(:,k) == D1*lambda*.433*H(:,k) + D2,
        
        0 <= v(:,k) <= v1,
        0 <= C(:,k) <= max_inflow,
        0 <= D(:,k) <= max_outflow,
        
        % pressure at utility and grey cistern'
        .433*M*H(:,k) == [60; 40],
        
        % pressure loss in pipes
        .433*H(:,k) >= 0,
        .433*H([8,9],k) == 0
        %A*Q(:,k) - A*q0 == O(:,:,k).*G*(H(:,k)-h0),
        
        % flow in pipes
        Q(2,k) >= 0,
        Q(3,k) >= 0,
        Q(4,k) >= 0,
        Q(5,k) >= 0,
      
        % valves
        0 <= O_k(G ~= 0) <= 1,
        O_k(G == 0) == 0,
        
        % pumps
        68 >= -1*B*A'*0.433*H(:,k) >= 0,
        B*Q(:,k) >= 0,
        
        % record water levels
        v_record(:,k) == v(:,k),
        
        % water demand
        W_demand(8,k) == .3*demand_minute(k+1000),
        W_demand(9,k) == .7*demand_minute(k+1000)
        W_demand([1:7],k) == zeros(7,1),
        
        % waste
        W_waste(6,k) == .5*Q(6,k),
        W_waste(6,k) >= 0,
        W_waste([1:5,7:9],k) == zeros(8,1),
        
        % water balance
        W_pull([2:4, 6:9],k) == zeros(7,1),
        W_pull([1,5],k) >= 0,
        W_pull(:,k) <= max_inflow,
        W_pull(:,k) - W_demand(:,k) - 15.85*W_waste(:,k) == A*15.85*Q(:,k) + lambda'*(C(:,k)-D(:,k))]
    
end

x0 = optimize([constraints, v(:,1) == V0], objective, sdpsettings('fmincon.MaxIter',250))

%% solve full problem
constraints = [];
objective = 0;
% v = vp0;
%v_cist = v_cist0;

for k = 1:50 % need to change to Nf
    
    % volume evolution
    %v = v + step*(C(:,k) - D(:,k));
    
    % objective optimizes for cost per unit water (cost of utility water,
    % cost to filter, cost of waste)
    objective = objective + W_pull(1,k)*phi_water + 50*W_pull(5,k)*phi_e(k+1000)*PD_water;
    
    O_k = O(:,:,k);
    
    % constraints
    constraints = [constraints,
        
        % tanks
%         v == D1*lambda*.433*H(:,k) + D2,
        
        v(:,k+1) == v(:,k) + step*(C(:,k) - D(:,k)),
        v(:,k) == D1*lambda*.433*H(:,k) + D2,
        
        0 <= v(:,k) <= v1,
        0 <= C(:,k) <= max_inflow,
        0 <= D(:,k) <= max_outflow,
        
        % pressure at utility and grey cistern
        .433*M*H(:,k) == [60; 40],
        
        % pressure loss in pipes
        .433*H(:,k) >= 0,
        .433*H([8,9],k) == 0
        abs(.433*H([2,7],k) - 50) <= 10*ones(2,1),
        A*Q(:,k) - A*q0 == O(:,:,k).*G*(H(:,k)-h0),       
        
        % flow in pipes
        Q(2,k) >= 0,
        Q(3,k) >= 0,
        Q(4,k) >= 0,
        Q(5,k) >= 0,
      
        % valves
        0 <= O_k(G ~= 0) <= 1,
        O_k(G == 0) == 0,
        
        % pumps
        68 >= -1*B*A'*0.433*H(:,k) >= 0,
        B*Q(:,k) >= 0,
        %1.5 == B*Q(:,k)*(-1*B*A')*.433*H(:,k)/(.5*5145),
        
        % record water levels
        v_record(:,k) == v(:,k),
        
        % water demand
        W_demand(8,k) == .3*demand_minute(k+1000),
        W_demand(9,k) == .7*demand_minute(k+1000)
        W_demand([1:7],k) == zeros(7,1),
        
        % waste
        W_waste(6,k) == .5*Q(6,k),
        W_waste(6,k) >= 0,
        W_waste([1:5,7:9],k) == zeros(8,1),
        
        % water balance
        W_pull([2:4, 6:9],k) == zeros(7,1),
        W_pull([1,5],k) >= 0,
        W_pull(:,k) <= max_inflow,
        W_pull(:,k) - W_demand(:,k) - 15.85*W_waste(:,k) == A*15.85*Q(:,k) + lambda'*(C(:,k)-D(:,k))]
end

options = sdpsettings('solver','fmincon','fmincon.MaxIter', 500, 'usex0',1, 'fmincon.TolCon', .1);
optimize([constraints, v(:,1) == V0], objective, options); 

%% plots
figure
plot(1:k,value(.433*H([1:7],1:k)), 'LineWidth',2)
title("Pressure (psi)")
legend("utility","potable tank","combine junction 1","combine junction 2","grey cistern","RO-waste", "recycle tank")

figure
plot(1:k,15.85*value(Q(:,1:k)),'LineWidth',2)
title("Flows (gpm)")
legend("utility-potable tank","potable-combine1","combine1-combine2","recycle tank-combine2","RO-recycle tank","grey cistern-RO", "potable use", "grey use")

figure
plot(1:k,value(W_pull([1,5],1:k)),'LineWidth',2)
title("Water pulled (gpm)")
legend("utility pull","grey pull")

% figure
% plot(1:k,value(W_demand([3 4],1:k)),'LineWidth',2)
% title("Water demand (gpm)")
% legend("potable", "recycled")

figure
plot(1:k,15.85*value(W_waste(6,1:k)),'LineWidth',2)
title("Waste flow (gpm)")

% figure
% plot(1:k,value(P_pump(:,1:k)),'LineWidth',2)
% title("Pump power (Watts)")

figure
plot(1:k,value(C(:,1:k))-value(D(:,1:k)),'LineWidth',2)
title("Net tank flow")
legend("potable","recycled")

figure
plot(1:k,value(v_record(:,1:k)),'LineWidth',2)
title("Tank volumes")
legend("potable", "recycled")

figure
valves_total = zeros(S,k);
for i = 1:k
    valves = value(O(:,:,i));
    valves = triu(valves,1);
    valves = valves(valves~=0);
    valves_total(:,i) = valves;
end
plot(1:k, valves_total,'LineWidth',2)
legend("utility-potable tank","potable-combine1","combine1-combine2", "recycle-combine", "waste-recycle","grey-waste","potable use","rec use")
hold on
title("Valve Open")

