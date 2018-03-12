%empctoolbox Economic Model Predictive Control Toolbox
% history = empctoolbox returns simulation results from
% default example. 
%
% The EMPC problem is solved using MATLABs linprog or quadprog.
% We recommend to install MOSEK (free license for academics)
% that replaces these functions with significant speedup.
%
% Author: Rasmus Halvgaard (rhal@dtu.dk)
% www.compute.dtu.dk/~rhal
% Feb 2014; Last revision: 6-Feb-2014

function history = empctoolbox
%% Setup simulation scenario
    rng('default'); % random number generator seed
    if nargin == 0
        Ts = 1;     	% model sampling period [h]
        Np = 24/Ts;     % prediction horizon
        Ns = 3*Np;     	% simulation horizon [1 = openloop]
        Nd = Np+Ns-1;   % simulation data horizon
        
        % Choose 'n' units
        [unitList,m] = makeUnitList({'powerplant',1;
                                       'heatpump',5;
                                        'battery',3;
                                            'V2G',1});
        n = m; % number of controller models = units
    end
    
% Pre-compute system and controller model
    [x0,~,~,As,Bs,Cs,Es,Hs,H0s,Hds] = setupSystem(Nd,Np,Ts,unitList,m);
    
    [xh0,par,u0,dst,q,c,U,dU,Y,P,G,Rww,Rvv,Rdd,sw,sv] = ...
                                      setupScenario(Nd,Ts,unitList,n);
    
    [mpc,A,B,E,C,H,H0,Hd,H0z,Cz,Dz,Fz] = designMPC(Np,Ts,unitList,n,par);

    % controller model = system simulation model
    A = As; B = Bs; E = Es; C = Cs; H = Hs; H0 = H0s; Hd = Hds;
    history.x0 = x0; history.d = dst; history.c = c; history.q = q;
    history.Y = Y; history.H = H; history.Hd = Hd; history.H0 = H0;
    
%% Closed loop simulation
    x = x0; xh = xh0; u0 = cell2mat(u0);
    for k = 1:Ns
        % Update openloop MPC problem
        [mpc,xh,P,y,d0,c0] = updateMPC(mpc,k,n,Np,u0,x,xh,dst,q,c,Y,U,dU,H0z,H0,Hd,A,C,E,Cs,G,P,Rww,Rvv,Rdd,sv);

        % Solve openloop MPC problem
        [~,u0,isConverged] = computeMPCLPsoftrsoft(Np,n,mpc); % computeMPCQPsoft, computeMPCLPsoftrsoft
        
        % dual price method
        % [~,~,isConverged,z] = computeMPCQPsoft(Np,n,mpc);% mpc.z = z; z0 = z(1);
        % [u,u0,isConvergedz] = computeMPCQPz(Np,n,mpc); isConverged = isConvergedz || isConverged;
        
        % Actuate system and update states
        for j = 1:n, z(:,j) = Cz{j}*xh{j} + Dz{j}*u0(j) + Fz{j}*d0{j}; end
        for j = 1:n, xh{j} = A{j}*xh{j} + B{j}*u0(j) + E{j}*d0{j}; end
        for j = 1:m,  x{j} = As{j}*x{j} + Bs{j}*u0(j) + Es{j}*d0{j}; end % + G{j}*sw{j}*randn;  
        
        % Save closedloop history
        history.ucl(k,:) = u0;
        history.ycl(k,:) = cell2mat(y);
        history.dcl(k,:) = cell2mat(d0);
        history.ccl(k,:) = cell2mat(c0);
        history.zcl(k,:) = cell2mat(c0); % z0
        history.pcl(k,:) = sum(z,2);
        if ~isConverged, warning('MPC open loop problem infeasible'), end
    end  

    % Plot results
    % plotMPC(Ns,Np,n,history)
    history.unitList = unitList;
    plotMPCsim1(Ns,Np,n,history)
end

%% Plot
    function plotMPC(Ns,Np,n,history)
    if Ns == 1 % open loop
        for j = 1:n
            u(:,j) = history.u(:,j);
            c(:,j) = history.c{j};
            d(:,j) = history.d{j}; % d = [d; reshape(history.d{k}{j},1,Np)];
            
            % predict output (time-invariant model)
            y(:,j) = history.H{j}*u(:,j) + history.H0{j}*history.x0{j} ...
                                         + history.Hd{j}*d(:,j);
        end
        z = history.z;
        N = Np;
    else % closed loop 
        c = history.ccl;
        z = history.zcl;
        d = history.dcl;
        u = history.ucl;
        y = history.ycl;
        p = history.pcl;
        N = Ns;
    end
    q = history.q(1:N,:);
    Y = cell2mat(history.Y);
    
    figure(1), clf, lw = 2; fs = 10;
    
    subplot(511), hold on, box on
    stairs(0:N-1,y,'linewidth',lw)
    stairs(0:N-1,repmat(Y(1,:),N,1),'--k','linewidth',lw)
    stairs(0:N-1,repmat(Y(2,:),N,1),'--k','linewidth',lw)
    ylabel('Output (y)','fontsize',fs)
    set(gca,'fontsize',fs)
    hold off
    
    subplot(512), hold on, box on
    stairs(0:N-1,d,'linewidth',lw)
    ylabel('Disturbance (d)','fontsize',fs)
    set(gca,'fontsize',fs)
    hold off    
    
    subplot(513), hold on, box on
    stairs(0:N-1,u,'linewidth',lw)
    % stairs(0:N-1,c,'r','linewidth',lw)
    ylabel('Input (u)','fontsize',fs)
    set(gca,'fontsize',fs)
    
    subplot(514), hold on, box on
    stairs(0:N-1,z,'k','linewidth',lw)
%    ylim([(min(z)) (max(z))])
    ylabel('Price (z)','fontsize',fs)
    set(gca,'fontsize',fs)
    
    subplot(515), hold on, box on
    stairs(0:N-1,p,'linewidth',lw)
    stairs(0:N-1,q,'--k','linewidth',lw)
    ylabel('Balance','fontsize',fs)
    set(gca,'fontsize',fs)    
	xlabel('Time [h]','fontsize',fs)
    hold off
    end
    function plotMPCsim1(Ns,Np,n,history)
    c = history.ccl;
    z = history.zcl;
    d = history.dcl;
    u = history.ucl;
    y = history.ycl;
    p = history.pcl;
    N = Ns;
    q = history.q(1:N,:);
    Y = cell2mat(history.Y);
    
    figure(10), clf, lw = 2; fs = 10;
    sp = 410;
    sp=sp+1; subplot(sp), hold on, box on
    stairs(0:N-1,p,'linewidth',lw)
    stairs(0:N-1,q,'--k','linewidth',lw)
    ylabel('Balance','fontsize',fs)
    set(gca,'fontsize',fs) 
    hold off
    
    sp=sp+1; subplot(sp), hold on, box on
    stairs(0:N-1,u,'linewidth',lw)
    % stairs(0:N-1,c,'r','linewidth',lw)
    ylabel('Input (u)','fontsize',fs)
    set(gca,'fontsize',fs)
    hold off
    
    sp=sp+1; subplot(sp), hold on, box on
    stairs(0:N-1,y,'linewidth',lw)
    stairs(0:N-1,repmat(Y(1,:),N,1),'--k','linewidth',lw)
    stairs(0:N-1,repmat(Y(2,:),N,1),'--k','linewidth',lw)
    ylabel('Output (y)','fontsize',fs)
    set(gca,'fontsize',fs)
    hold off
    
    sp=sp+1; subplot(sp), hold on, box on
    stairs(0:N-1,d,'linewidth',lw)
    ylabel('Disturbance (d)','fontsize',fs)
    set(gca,'fontsize',fs)
    legend(history.unitList)
	xlabel('Time [h]','fontsize',fs)
    hold off
    end

%% Setup system models and simulation scenario
    function [x0,par,u0,dst,q,c,U,dU,Y,P,G,Rww,Rvv,Rdd,sw,sv] = setupScenario(Nd,Ts,unitList,n)

    % Support functions
    randUni = @(a,b,N) a + (b-a).*rand(1,N);
    randSine = @(a,b,Nd,Ts) a.*sin(2*pi/24*(0:Ts:(Nd-1)*Ts)-pi/2) + b;    

    for j = 1:n
        switch unitList{j}
            case 'thermal1st'
                U{j} = [0 100]';
                Y{j} = [15 25]';
                dU{j} = [-U{j}(2) U{j}(2)]'./1;
                x0{j} = mean(Y{j});
                u0{j} = 0;
                a = randUni(2,3,1); b = randUni(8,14,1);
                dst{j} = randSine(a,b,Nd,Ts)';
                %dst{j} = zeros(1,Nd)';
                tau = randUni(50,100,1);
                par{j} = [tau 1];
                %c{j} =  randSine(10,20,Nd,Ts)'; % local cost
                c{j} = zeros(Nd,1);
                
            case 'battery' 
                U{j} = [0 3.3]'.*3.6; % 3.3, 9.6, 16.8 kW (*3.6 -> MJ/h)
                Y{j} = [0 1]';
                dU{j} = [-U{j}(2) U{j}(2)]'./1;
                x0{j} = mean(Y{j});         
                u0{j} = 0;
                a = randUni(2,3,1); b = randUni(0,0,1);
                dst{j} = abs(randSine(a,b,Nd,Ts))';
                % dst{j} = zeros(1,Nd)';
                Qn = randUni(20,30,1).*3.6; % kWh (*3.6 -> MJ)
                par{j} = [0.9 1 Qn]; % etap,Kd,Qn
                %c{j} =  randSine(10,20,Nd,Ts)'; % local cost
                c{j} = zeros(Nd,1);
                
                % Qn
                % 8  kWh [City EVs]
                % 13 kWh [Citroen C1 EV]
                % 16 kWh [Chevy Volt, Mitsubishi MiEV]
                % 22 kWh [Renault Fluence, Chevrolet Spark]
                % 24 kWh [Nissan Leaf]
                % 35 kWh [BMW Mini E]
                % 53 kWh [Tesla Roadster]  
                
            case 'V2G'
                U{j} = [0 9.6]'.*3.6; % 3.3, 9.6, 16.8 kW (*3.6 -> MJ/h)
                Y{j} = [0 1]';
                dU{j} = [-U{j}(2) U{j}(2)]'./1;
                x0{j} = mean(Y{j});         
                u0{j} = 0;
                a = randUni(2,3,1); b = randUni(0,0,1);
                dst{j} = abs(randSine(a,b,Nd,Ts))';
                % dst{j} = zeros(1,Nd)';
                Qn = randUni(30,40,1).*3.6; % kWh (*3.6 -> MJ)
                par{j} = [0.9 1 Qn]; % etam,Kd,Qn
                %c{j} =  randSine(10,20,Nd,Ts)'; % local cost
                c{j} = zeros(Nd,1); 

            case 'heatpump'
                % Low energy building -- nominal parameters
                Cr = 810.19e-3;    % MJ/K
                Cf = 3314.74e-3;   % MJ/K
                Cw = 200*4.181e-3; % MJ/K (200 l water tank) cw=4.1813 J/(g K)
                UAra = 28.35e-3;   % MJ/(K h)
                UAfr = 624.34e-3;  % MJ/(K h)
                UAwf = 28.35e-3;   % MJ/(K h)
                eta = 3;           % COP

                % generate random parameters
                r = randUni(0.75,1,7);
                par{j} = r.*[Cr Cf Cw UAra UAfr UAwf eta];

                % constraints
                U{j} = [0 3.6]'; % MJ/h ( = 1 kW)
                dU{j} = [-U{j}(2) U{j}(2)]'./1;
                Y{j} = [20 24]';
                x0{j} = [21 22 25]'; % initial conditions
                u0{j} = 0;	% previous decision u(k-1)            

                % disturbance
                a = randUni(2,3,1); b = randUni(4,8,1);
                dst{j} = randSine(a,b,Nd,Ts)';
                %dst{j} = zeros(1,Nd)';
                c{j} = zeros(Nd,1);
                %c{j} =  randSine(1,2,Nd,Ts)'; % local cost

            case 'heatpump2'
                % Low energy building -- nominal parameters
                g = 198/85*1e-6;
                Ci = 4.601e6*g; % kJ/K
                Ce = 1.230e6*g; % kJ/K
                Cf = 1.575e7*g; % kJ/K
                Cw = 200*4181*1e-6;  % L * kJ/(K kg)
                UAai2 = (2/1.208e-2)*g*3600; % kJ/h/K
                UAfi = (1/1.350e-3)*g*3600; % kJ/h/K
                UAwf = UAfi;
                eta = 3; % COP

                % generate random parameters +-50%
                r = randUni(0.75,1.1,9);
                par{j} = r.*[g Ci Ce Cf Cw UAai2 UAfi UAwf eta];

                % constraints
                U{j} = [0 3.6]'; % kW -> kJ/h [3600 s/h]
                dU{j} = [-U{j}(2) U{j}(2)]'./1;
                Y{j} = [18 24]';
                x0{j} = [19.5 3.5 23 22]'; % initial conditions
                u0{j} = 0;	% previous decision u(k-1)            

                % disturbance
                a = randUni(2,3,1); b = randUni(6,8,1);
                dst{j} = randSine(a,b,Nd,Ts)';
                %dst{j} = zeros(1,Nd)';
                %c{j} = randSine(1,1,Nd,Ts)'; % local cost
                c{j} = zeros(Nd,1);
                
            case 'powerplant'
                K = 1;
                tau = 0.1;

                % generate random parameters
                r = randUni(1,1,2); 
                par{j} = r.*[K tau];

                % constraints
                U{j} = [0 10]'; % MJ/h
                dU{j} = [-U{j}(2) U{j}(2)]'./2;
                Y{j} = [0 15]';
                x0{j} = zeros(3,1); % initial conditions
                u0{j} = 0;	% previous decision u(k-1)

                % disturbance
                dst{j} = zeros(1,Nd)';
                c{j} = zeros(Nd,1);
                %c{j} =  randSine(1,2,Nd,Ts)'; % local cost                
        end
        
        % Kalman filter init
        nx = size(x0{j},1); ny = 1;
        sw{j} = 1e-3.*ones(nx,1); % process noise standard deviation
        sv{j} = 0.5.*ones(ny,1); % measurement noise standard deviation 
        Rww{j} = diag(sw{j}.^2);
        Rvv{j} = diag(sv{j}.^2);
        Rdd{j} = 0;
        P{j} = eye(nx);
        G{j} = diag([1 zeros(1,nx-1)]); % NB
    end

    % power tracking reference
    Umax = max(sum(abs(cell2mat(U)),2));
    q = zeros(Nd,1);
    
%     q(20:30) = Umax*0.4;
%     q(31:40) = Umax*0.5;
%     %q(45:50) = Umax*1.2;
    
    %dataWindConsumption;
    rd = wind;
    q = rd(1:Nd)./max(rd(1:Nd)).*Umax.*0.2;  %+ Umax*0.2;
    %q = abs(q);
    q = [q q];
    
    % baseload = 1.*randSine(a,b,Nd)'; % wind power, assume free (disturbance)
    % q = [3.*ones(Nd,1) 10.*ones(Nd,1) ];
    end
    function [x0,u0,dst,As,Bs,Cs,Es,Hs,H0s,Hds] = setupSystem(Nd,Np,Ts,unitList,m)

        [x0,par,u0,dst] = setupScenario(Nd,Ts,unitList,m);
        for j = 1:m
            [As{j},Bs{j},Es{j},Cs{j},Hs{j},H0s{j},Hds{j}] = computeModel(Np,Ts,unitList{j},par{j});
        end
    end

%% MPC
    function [mpc,A,B,E,C,H,H0,Hd,H0z,Cz,Dz,Fz] = designMPC(Np,Ts,unitList,n,par)
    I = eye(Np); S = sparse(repmat(I,1,n)); mpc.I = I; mpc.S = S;
    mpc.I0 = [1; zeros(Np-1,1)];
    dH = differenceMatrix(Np,1); % rate of movement matrix    
    mpc.V = kron(eye(n),[-I; -I; zeros(4*Np,Np)]); % soft constraint matrix
    mpc.W = [ S zeros(Np,Np*n) -I;
             -S zeros(Np,Np*n) -I;
              zeros(Np,2*Np*n) -I];
          
    % trade off balance for unit output constraint violation Vrho > Wrho
    mpc.Vrho = 1e3 * ones(Np*n,1); % soft output constraint penalty
    mpc.Wrho = 1e4 * ones(Np,1); % soft balance penalty
    mpc.lambda = 0.5; % control action regularization tuning
    
    mpc.ldHdH = kron(eye(n),(dH'*dH).*mpc.lambda);
    mpc.H = blkdiag(mpc.ldHdH,eye(Np*n,Np*n),mpc.I); % QP Hessian
    mpc.B = [-mpc.S zeros(Np,Np*n) mpc.I]; % QP balance equality constraint
    
    WC = [];
    for j = 1:n
        [A{j},B{j},E{j},C{j},H{j},H0{j},Hd{j},...
        Hz{j},H0z{j},Hdz{j},Cz{j},Dz{j},Fz{j}] = computeModel(Np,Ts,unitList{j},par{j});

        % Constraints A*u <= b
        mpc.A{j} = [H{j}; -H{j}; I; -I ; dH; -dH];
        
        WC = [WC Hz{j}];
    end
   
    mpc.WC = [WC zeros(Np,Np*n) -I;
             -WC zeros(Np,Np*n) -I;
              zeros(Np,2*Np*n) -I];
    end
    function [mpc,xh,P,y,d0,c0] = updateMPC(mpc,k,n,Np,u0,x,xh,dst,q,c,Y,U,dU,H0z,H0,Hd,A,C,E,Cs,G,P,Rww,Rvv,Rdd,sv)
    l = ones(Np,1);
    ld = zeros(Np,1);
    
    for j = 1:n
        % Measure outputs
        z{j} = Cs{j}*x{j};
        y{j} = z{j};% + sv{j}*randn;
        
        % [xh{j},P{j}] = updateStateEstimate(xh{j},P{j},y{j},A{j},C{j},E{j},G{j},Rww{j},Rvv{j},Rdd{j});
        xh{j} = x{j};

        % Update disturbance forecast
        d{j} = dst{j}(k:k+Np-1,:);
        d0{j} = d{j}(1,:)';

        % Update MPC constraints
        mpc.b{j} =  [Y{j}(2)*l - H0{j}*xh{j} - Hd{j}*d{j};
                    -Y{j}(1)*l + H0{j}*xh{j} + Hd{j}*d{j};
                     U{j}(2)*l;
                    -U{j}(1)*l
                     ];
        mpc.H0zxh{j} = H0z{j}*xh{j};% + Hdz{j}*d{j};
        
        % Update input rate of movement constraints
        ld(1) = u0(j);
        mpc.um1{j} = u0(j);
        mpc.b{j} =  [ mpc.b{j};
                      dU{j}(2)*l+ld;
                     -dU{j}(1)*l-ld];       

        % udpate time-varying input costs
        mpc.c{j} = c{j}(k:k+Np-1,:);
        c0{j} = mpc.c{j}(1,:)';
    end
        mpc.q = q(k:k+Np-1,:); % update tracking reference (bounds)
    end
    function [xh,P] = updateStateEstimate(xh,P,y,A,C,E,G,Rww,Rvv,Rdd)
    P = A*P*A' + G*Rww*G' + E*Rdd*E';
    Re = Rvv + C*P*C';
    Kfx = P*C'/Re;
    P = P - Kfx*Re*Kfx'; % covariance
    e = y - C*xh;        % oberserver error
    xh = xh + Kfx*e;     % state update
end

%% Solvers and different cost functions
    % no coordination, only costs
    function [u,u0,isConverged] = computeMPCLPsoft(N,n,mpc)
        A = []; b = []; c = [];
        for j = 1:n
            A = blkdiag(A,mpc.A{j});
            b = [b; mpc.b{j}];
            c = [c; mpc.c{j}];
        end
        
        % add soft output constraints
        A = [A mpc.V; zeros(N*n,N*n) -eye(N*n,N*n)]; % NB
        b = [b; zeros(N*n,1)];
        c = [c; mpc.Vrho];
        
      	[uv,~,isConverged] = linprog(c,A,b);

        u = uv(1:N*n);
        u = reshape(u,N,n);
        u0 = u(1,:);
        % u = num2cell(u,1);
    end
    function [u,u0,isConverged] = computeMPCLP(N,n,mpc)
        A = []; b = []; c = [];
        for j = 1:n
            A = blkdiag(A,mpc.A{j});
            b = [b; mpc.b{j}];
            c = [c; mpc.c{j}];
        end
        
%         % add soft output constraints
%         A = [A mpc.V; zeros(N*n,N*n) -eye(N*n,N*n)]; % NB
%         b = [b; zeros(N*n,1)];
%         c = [c; mpc.Vrho];
        
      	[uv,~,isConverged] = linprog(c,A,b);

        u = uv(1:N*n);
        u = reshape(u,N,n);
        u0 = u(1,:);
    end
    function [u,u0,isConverged] = computeMPCQPz(N,n,mpc)
        A = []; b = []; z = [];
        for j = 1:n
            A = blkdiag(A,mpc.A{j});
            b = [b; mpc.b{j}];
            z = [z; mpc.z - mpc.lambda*mpc.um1{j}*mpc.I0];
        end
        
        H = mpc.ldHdH;
        
      	[uv,~,isConverged] = quadprog(H,z,A,b);

        u = uv(1:N*n);
        u = reshape(u,N,n);
        u0 = u(1,:);
    end    
    
    % coordination LP reference q = [rmin,rmax]
    function [u,u0,isConverged] = computeMPCLPsoftrsoft(N,n,mpc)
        A = []; b = []; c = []; p0 = 0;
        for j = 1:n
            A = blkdiag(A,mpc.A{j});
            b = [b; mpc.b{j}];
            c = [c; mpc.c{j}];
            p0 = p0 + mpc.H0zxh{j};
        end
        
        % add soft output constraints 
        A = [A mpc.V; zeros(N*n,N*n) -eye(N*n,N*n)];
        b = [b; zeros(N*n,1)];
        c = [c; mpc.Vrho];
            
        % add soft total power balance output constraints
        % rmin <= S*u + w <= rmax
        A = [A zeros(size(A,1),N); mpc.WC];
        b = [b; mpc.q(:,2) - p0;
               -mpc.q(:,1) + p0;
                zeros(N,1)];
        c = [c; mpc.Wrho];
        
      	[uvw,~,isConverged] = linprog(c,A,b);

        u = uvw(1:N*n);
        u = reshape(u,N,n);
        u0 = u(1,:);
    end       
    
    % coordination QP tracking reference q = rmin
    function [u,u0,isConverged,z] = computeMPCQP(N,n,mpc)
        A = []; b = []; c = [];
        for j = 1:n
            A = blkdiag(A,mpc.A{j});
            b = [b; mpc.b{j}];
            c = [c; mpc.c{j} - mpc.lambda*mpc.um1{j}*mpc.I0];
        end
            
        % add total power balance output constraints
        A = [A zeros(size(A,1),N)];
        c = [c; zeros(N,1)];
        
        B = [-mpc.S mpc.I];
        d = -mpc.q(:,1); % = rmin
        
        H = blkdiag(mpc.ldHdH,mpc.I);
        
      	[ud,~,isConverged,~,lam] = quadprog(H,c,A,b,B,d);

        u = ud(1:N*n);
        u = reshape(u,N,n);
        u0 = u(1,:);
        z = -lam.eqlin; % dual variable associated with balance
    end
    function [u,u0,isConverged,z] = computeMPCQPsoft(N,n,mpc)
        A = []; b = []; c = [];
        for j = 1:n
            A = blkdiag(A,mpc.A{j});
            b = [b; mpc.b{j}];
            c = [c; mpc.c{j} - mpc.lambda*mpc.um1{j}*mpc.I0];
        end
        
        % add soft output constraints
        A = [A mpc.V; zeros(N*n,N*n) -eye(N*n,N*n)];
        b = [b; zeros(N*n,1)];
        c = [c; mpc.Vrho];
            
        % add total power balance output constraints
        A = [A zeros(size(A,1),N)];
        c = [c; zeros(N,1)];
        
        B = mpc.B; % [-mpc.S 0 mpc.I];
        d = -mpc.q(:,1); % = rmin
        
        H = mpc.H; % blkdiag(mpc.ldHdH,eye(N*n,N*n),mpc.I);
        
      	[ud,~,isConverged,~,lam] = quadprog(H,c,A,b,B,d);

        u = ud(1:N*n);
        u = reshape(u,N,n);
        u0 = u(1,:);
        z = -lam.eqlin; % dual variable associated with balance
    end
    
    % add L1 regularization % NB change updateMPC
    function [u,u0,isConverged] = computeMPCLPrsoftreg(N,n,mpc)
        A = []; Av = []; b = []; c = [];
        for j = 1:n
            A = blkdiag(A,mpc.A{j});
            b = [b; mpc.b{j}];
            c = [c; mpc.c{j}];
        end
        
        % add soft reg constraints  
        Qdu = 1e2;
        for j = 1:n
            A = blkdiag(A,-mpc.I);
            b = [b; zeros(N,1)];
            Av = blkdiag(Av,mpc.W);
            c = [c; Qdu.*ones(N,1)];
        end
            A(1:6*N*n,N*n+1:end) = Av;  
            
        % add constraints on total power output
        A = [A zeros(7*N*n,N); mpc.S zeros(N,N*n) -mpc.I;
            -mpc.S zeros(N,N*n) -mpc.I; zeros(N,2*N*n) -mpc.I];
        b = [b; mpc.q(:,2); -mpc.q(:,1); zeros(N,1)];
        c = [c; mpc.rho'];
        
      	[uvw,~,isConverged] = linprog(c,A,b);

        u = uvw(1:N*n);
        u = reshape(u,N,n);
        u0 = u(1,:);
        u = num2cell(u,1);
    end 

%% Model library and discretization
    function [A,B,E,C,H,H0,Hd,Hz,H0z,Hdz,Cz,Dz,Fz] = computeModel(N,Ts,model,par)
    
    switch model
        case 'thermal1st'
            % G(s) = c/(tau s + 1)
            tau = par(1);
            c = par(2);
            f = -par(2);

            A = -1/tau;
            B = c/tau;
            C = 1;
            E = f/tau;
            
            Cz = 0;
            Dz = 1;
            Fz = 0;

        case 'heatpump'
            Cr = par(1);
            Cf = par(2);
            Cw = par(3);
            UAra = par(4);
            UAfr = par(5);
            UAwf = par(6);
            eta = par(7);

            A = [ (-UAfr-UAra)/Cr UAfr/Cr 0;
                        UAfr/Cf (-UAwf-UAfr)/Cf UAwf/Cf;
                        0 UAwf/Cw -UAwf/Cw];
            B = [0 0 eta/Cw]';
            %E = [UAra/Cr (1-p)/Cr; 0 p/Cf; 0 0];
            E = [UAra/Cr 0 0]';
            C = [1 0 0];
            
            Cz = zeros(1,3);
            Dz = 1;
            Fz = 0;

        case 'heatpump2'
            % Greenland floor heating house
            % C.hus = C.lejlighed * 198/85 
            % UA.hus = UA.lejlighed * 198/85
            c = par(1);
            Ci = par(2);
            Ce = par(3);
            Cf = par(4);
            Cw = par(5);
            UAai2 = par(6);
            UAfi = par(7);
            UAwf = par(8);
            eta = par(9);

            A = [(-UAai2-UAfi)/Ci, UAai2/Ci, UAfi/Ci, 0;
                 UAai2/Ce, -2*UAai2/Ce, 0, 0;
                 UAfi/Cf, 0, (-UAwf-UAfi)/Cf, UAwf/Cf;
                 0, 0, UAwf/Cw, -UAwf/Cw];

            B = [0 0 0 eta/Cw]';
            C = [1 0 0 0];
            %E = [0 As/Ci; UAai2/Ce 0; 0 0; 0 0];
            E = [0 UAai2/Ce 0 0]';
            
            Cz = zeros(1,3);
            Dz = 1;
            Fz = 0;            

        case 'battery'
            etap = par(1); % [etap,Kd,Qn] = getElements(par);
            Kd = par(2);
            Qn = par(3);

            A = 0;
            B = etap;
            E = -Kd;
            C = 1/Qn;
            
            Cz = 0;
            Dz = 1;
            Fz = 0;
            
        case 'V2G'
            etam = par(1);
            Kd = par(2);
            Qn = par(3);

            A = 0;
            B = 1/etam;
            E = Kd;
            C = 1/Qn;
            
            Cz = 0;
            Dz = -1;
            Fz = 0;             
            
        case 'solartank'
            mw = par(1); % kg = L
            cw = par(2); % J/(kg * K)
            eta = par(3);
            UA = par(4); % W/K [*3600 s/h * 1e-6 MJ/J] -> MJ/(K*h)
            Ct = cw*mw*1e-6; % MJ/K

            nu = 1; % electricial heating element power [W]
            nx = 1; ny = 1; % tank temperature [C]
            nd = 3; % solar power [W], consumption [W], ambient temperature [C]

            A = -UA/Ct;
            B = eta/Ct;
            E = [1 -1 UA]./Ct;
            C = 1;
            
            Cz = 0;
            Dz = 1;
            Fz = 0;  

        case 'powerplant'
            K = par(1);
            tau1 = par(2);
            tau2 = tau1*tau1;
            tau3 = tau2*tau1;

            A = [-3/tau1, -3/tau2, -1/tau3; 1 0 0; 0 1 0];
            B = [K/tau3 0 0]';
            E = zeros(3,1);
            C = [0 0 1];
            
            Cz = [0 0 -1];
            Dz = zeros(1,1);
            Fz = zeros(1,1);
            
        case 'EV2G'
            % model parameters
            etap = par(1);
            etam = par(2);
            Kd = par(3);
            Qn = par(4);

            A = 0;
            B = [etap -1/etam]; % NB etam = inverse
            E = -Kd;
            C = 1/Qn;
            
            Cz = 0;
            Dz = [1 -1];
            Fz = 0;             
    end

    % Discretize
        [A,B,E] = css2dss(A,B,E,Ts);
        [H,H0,Hd] = impulseResponse(N,A,B,E,C); % NB N-1
        [Hz,H0z,Hdz] = impulseResponse(N,A,B,E,Cz,Dz,Fz);
    end
    function [Hu,H0,Hd,nx,nu,ny,nd] = impulseResponse(N,A,B,E,C,D,F)
    % Discrete time state space model
    % x(k+1) = A x + B u + E d
    %      y = C x + D u + F d
    % to impulse response matrices
    %      y = Hu*u + H0*x0 + Hd*d
    [nx,nu] = size(B);
    nd = size(E,2);
    ny = size(C,1);
    
    if nargin <= 5, D = zeros(ny,nu); 
                    F = zeros(ny,nd); end
    
    H0 = zeros((N-1)*ny,nx);
    Hu = zeros((N-1)*ny,N*nu);
    Hd = zeros((N-1)*ny,N*nd);

    % compute first column with all impulse response coefficients
    T = C;
    k1 = 1;
    k2 = ny;
    for k=1:N-1
       Hu(k1:k2,1:nu) = T*B;
       Hd(k1:k2,1:nd) = T*E;
       T = T*A;
       H0(k1:k2,1:nx) = T;
       k1 = k1+ny;
       k2 = k2+ny;
    end

    % add extra row with direct output contributions for k = 0
    Hu = [D, zeros(size(D,1),size(Hu,2)-size(D,2)); Hu];
    Hd = [F, zeros(size(F,1),size(Hd,2)-size(F,2)); Hd];
    H0 = [C, zeros(size(C,1),size(H0,2)-size(C,2)); H0];
    
    % copy coefficients and fill out remaining columns
    k1row = ny+1;
    k2row = N*ny;
    k1col = nu+1;
    k2col = nu+nu;
    kk = N*ny-ny;
    for k=2:N
       Hu(k1row:k2row,k1col:k2col) = Hu(1:kk,1:nu);
       k1row = k1row+ny;
       k1col = k1col+nu;
       k2col = k2col+nu;
       kk = kk-ny;
    end

    k1row = ny+1;
    k2row = N*ny;
    k1col = nd+1;
    k2col = nd+nd;
    kk = N*ny-ny;
    for k=2:N
       Hd(k1row:k2row,k1col:k2col) = Hd(1:kk,1:nd);
       k1row = k1row+ny;
       k1col = k1col+nd;
       k2col = k2col+nd;
       kk = kk-ny;
    end
end
    function dH = differenceMatrix(N,nu)
    % Compute difference matrix for rate of movement constraints
    I = eye(nu,nu);
    dH = kron(diag(ones(N,1)),I) - kron(diag(ones(N-1,1),-1),I);
    end
    function [Ad,Bd,Ed] = css2dss(A,B,E,Ts)
    % discretize ss model (c2d)
    [nx,nu] = size(B);
    nd = size(E,2);

    dss = expm([A B E; zeros(nu+nd,nx+nu+nd)]*Ts);

    Ad = dss(1:nx,1:nx);
    Bd = dss(1:nx,nx+1:nx+nu);
    Ed = dss(1:nx,nx+nu+1:nx+nu+nd);
    end
    
%% Support functions
    function [unitList,n] = makeUnitList(cmdlist)
    % Generate list of 'n' units from cmdlist
        if size(cmdlist,2) > 2, error('cmdlist not valid. size(cmdlist) > 2. Must be < 2'), end
        v = [cmdlist{:,2}];
        n = sum(v); unitList = cell(n,1);
        cmd = size(cmdlist,1);
        j = 0;
        for p = 1:cmd
            for q = 1:v(p)
                j = j + 1;
                unitList{j} = cmdlist{p,1};
            end
        end
    end
    function varargout = getElements(p)
        for i = 1:length(p)
             varargout(i) = {p(i)};
        end
    end
    function rd = wind
           windproduction = [
        1041.1
        1170.5
        1267.8
        1354.6
        1448.2
        1551.9
        1612.7
        1565.7
        1429.8
        1263.2
        1296.4
        1333.6
        1368.8
        1375.5
        1340.7
        1238.0
        1116.1
        1112.4
        1050.5
        1079.0
        1069.6
        990.6
        887.6
        832.8
        840.6
        701.7
        683.3
        861.8
        922.5
        967.6
        974.0
        969.4
        964.4
        900.1
        872.3
        950.9
        1043.3
        1075.4
        1080.7
        1068.7
        1073.4
        1144.0
        1220.5
        1171.5
        1237.8
        1228.9
        1234.1
        1254.4
        1355.5
        1423.9
        1473.6
        1534.0
        1583.1
        1644.4
        1707.6
        1843.2
        1977.0
        2165.1
        2262.5
        2210.7
        2298.1
        2463.0
        2568.8
        2561.4
        2497.1
        2355.0
        2328.8
        2291.4
        2028.4
        1784.9
        1664.2
        1487.1
        1289.7
        1150.8
        1195.4
        1447.8
        1643.5
        1876.3
        2078.4
        2154.1
        2274.1
        2340.3
        2352.2
        2368.3
        2434.1
        2419.1
        2422.0
        2364.0
        2272.8
        2244.4
        2268.5
        2248.8
        2204.0
        2167.5
        2144.0
        2090.1
        1937.5
        1675.3
        1435.5
        1252.6
        1206.0
        1324.7
        1536.5
        1940.1
        2090.5
        1884.9
        1611.7
        1241.2
        1161.1
        1193.2
        1183.9
        1500.0
        1689.1
        1829.7
        1834.6
        1560.3
        1208.2
        1104.2
        1056.7
        1006.6
        1010.6
        974.8
        970.7
        1019.4
        1005.0
        941.1
        789.4
        633.6
        564.0
        506.6
        453.7
        437.6
        463.6
        535.8
        670.3
        609.1
        679.9
        767.8
        787.2
        912.1
        1079.6
        1035.0
        1124.8
        1199.5
        1321.2
        1181.6
        1124.4
        1164.1
        1222.1
        1162.8
        1092.7
        1099.7
        1249.2
        1403.1
        1507.5
        1613.0
        1792.3
        1852.1
        1885.8
        1868.3
        1732.2
        1630.8
        1633.8
        1601.0
        1552.9
        1551.6
        1526.7
        1438.3
        1394.8
        1303.5
        1199.2
        1122.7
        1028.5
        969.8
        910.7
        884.4
        821.5
        755.8
        692.6
        617.4
        551.2
        438.2
        500.5
        411.9
        396.4
        282.2
        203.0
        100.9
        51.7
        34.6
        27.8
        21.9
        22.8
        20.4
        11.0
        13.4
        25.0
        48.5
        97.7
        176.8
        279.7
        400.3
        519.1
        612.6
        645.4
        796.0
        933.0
        1125.8
        1372.5
        1686.9
        2060.3
        2206.0
        2202.9
        2215.5
        2262.5
        2260.6
        2150.4
        1835.9
        1349.3
        1006.3
        793.2
        718.9
        538.5
        508.8
        600.3
        627.9
        579.1
        609.6
        705.2
        851.1
        827.8
        803.6
        786.4
        760.5
        805.3
        912.9
        1041.2
        1245.9
        1390.8
        1538.4
        1671.8
        1705.2
        1737.9
        1794.1
        1797.3
        1790.9
        1750.1
        1755.0
        1782.7
        1674.8
        1540.4
        1482.9
        1396.0
        1295.7
        1091.2
        889.5
        813.7
        866.9
        1012.4
        1129.7
        1373.0
        1658.7
        1950.0
        2196.2
        2334.3
        2480.4
        2571.4
        2620.6
        2656.3
        2678.6
        2682.8
        2689.5
        2675.6
        2613.4
        2531.2
        2594.3
        2597.9
        2503.4
        2510.1
        2509.9
        2509.8
        2400.1
        2280.7
        2257.2
        2249.0
        2249.0
        2256.8
        2244.5
        2187.1
        2205.8
        2333.0
        2327.5
        2332.6
        2295.4
        2276.8
        2173.1
        1979.7
        1802.7
        1796.8
        1847.1
        2015.2
        2159.8
        2199.5
        2157.6
        2119.1
        2120.9
        2065.8
        1998.8
        1928.7
        1874.7
        1743.7
        1598.1
        1465.1
        1339.9
        1221.7
        1087.7
        931.9
        816.2
        691.1
        592.8
        505.4
        379.7
        284.3
        199.3
        172.6
        159.6
        172.0
        217.1
        241.6
        225.0
        211.6
        177.6
        150.7
        137.9
        118.8
        101.2
        78.3
        69.7
        70.6
        64.6
        51.6
        36.9
        31.7
        49.6
        63.9
        79.7
        101.6
        104.9
        104.6
        126.1
        142.2
        179.8
        218.5
        244.6
        246.7
        255.4
        264.3
        285.4
        299.9
        293.2
        304.9
        320.1
        324.8
        328.1
        338.3
        370.9
        395.1
        428.3
        448.3
        441.0
        464.8
        488.1
        580.0
        712.3
        815.3
        849.9
        817.9
        787.5
        836.2
        804.4
        781.1
        722.1
        655.7
        609.7
        580.4
        506.7
        528.8
        498.5
        403.5
        329.6
        345.2
        321.1
        224.4
        128.0
        84.8
        85.0
        114.6
        159.1
        155.0
        135.7
        136.2
        183.0
        229.1
        261.4
        332.7
        399.9
        334.1
        355.8
        380.2
        399.6
        445.2
        461.3
        562.4
        621.1
        754.5
        921.1
        1046.3
        1088.1
        1217.9
        1447.2
        1667.3
        1789.1
        1847.8
        1914.6
        1888.5
        1763.7
        1631.8
        1503.2
        1202.6
        1069.2
        957.0
        982.4
        939.9
        1036.5
        1169.7
        1364.4
        1336.3
        1355.5
        1433.7
        1473.3
        1402.6
        1221.2
        1066.3
        969.4
        777.2
        829.6
        970.5
        1137.7
        1326.3
        1508.7
        1693.7
        1802.5
        1862.9
        1801.5
        1564.5
        1191.2
        884.3
        644.1
        746.7
        553.7
        453.9
        398.1
        418.2
        513.2
        463.2
        542.6
        557.6
        556.9
        627.7
        743.4
        802.0
        812.4
        796.2
        743.1
        605.8
        361.9
        189.7
        89.2
        62.0
        68.1
        134.7
        277.4
        434.8
        589.4
        756.1
        1079.7
        1396.6
        1856.6
        2188.0
        2207.1
        1897.8
        1680.3
        1630.6
        1728.2
        1752.5
        1725.1
        1605.1
        1587.8
        1414.8
        1332.8
        1259.9
        1155.3
        1025.3
        998.6
        1019.3
        1011.8
        1020.0
        1033.2
        1050.3
        945.6
        726.6
        668.9
        619.4
        664.8
        595.8
        520.2
        428.2
        389.2
        369.6
        439.4
        432.0
        391.4
        296.0
        240.0
        175.4
        138.4
        112.3
        89.5
        66.3
        44.2
        33.3
        29.0
        37.0
        46.6
        55.6
        70.8
        110.6
        116.8
        104.2
        119.0
        109.6
        136.6
        133.9
        154.7
        214.4
        251.2
        259.0
        253.3
        213.6
        221.2
        222.0
        200.2
        188.2
        206.2
        249.6
        272.0
        250.1
        249.8
        271.0
        262.8
        272.9
        253.3
        220.5
        194.2
        162.7
        129.2
        118.1
        128.8
        141.7
        176.1
        217.8
        257.2
        292.8
        304.5
        322.0
        294.6
        266.7
        271.7
        297.6
        345.2
        366.0
        427.4
        506.8
        554.0
        601.4
        630.6
        600.7
        634.1
        677.0
        684.1
        665.0
        663.2
        821.1
        997.2
        1072.1
        1116.3
        1174.7
        1262.7
        1341.2
        1491.9
        1594.1
        1707.7
        1788.8
        1838.0
        1907.5
        1964.0
        2019.6
        2081.9
        2085.8
        2154.1
        2285.0
        2377.2
        2444.4
        2465.0
        2483.6
        2477.6
        2468.3
        2468.3
        2474.5
        2489.3
        2553.0
        2556.0
        2524.6
        2445.0
        2355.1
        2233.7
        2128.5
        2056.2
        1989.4
        1943.7
        1930.2
        1914.5
        1822.8
        1755.5
        1746.9
        1709.5
        1637.8
        1518.2
        1445.7
        1408.7
        1393.5
        1373.9
        1357.6
        1356.0
        1337.5
        1306.5
        1264.1
        1242.6
        1207.4
        1135.3
        1026.9
        938.2
        860.5
        787.8
        779.4
        815.9
        827.7
        789.8
        774.8
        719.7
        617.7
        593.1
        593.2
        627.0
        703.1
        693.2
        663.8
        613.6
        496.0
        527.8
        463.7
        461.2
        492.8
        515.0
        581.2
        667.3
        712.2
        739.8
        720.2
        687.6
        696.6
        658.9
        614.9
        639.9
        724.7
        818.8
        915.8
        971.9
        1098.0
        1217.0
        1286.5
        1318.6
        1331.8
        1349.2
        1315.4
        1350.9
        1368.3
        1375.5
        1430.5
        1441.0
        1421.8
        1429.3
        1506.2
        1606.1
        1685.4
        1805.0
        1899.0
        1985.6
        2022.8
        1987.4
        1927.5
        1884.3
        1839.5
        1837.9
        1883.7
        1882.5
        1861.6
        1783.9
        1761.7
        1743.3
        1625.1
        1524.4
        1505.2
        1471.7
        1470.2
        1463.3
        1449.5
        1423.6
        1379.5
        1356.5
        1351.1
        1400.9
        1444.8
        1411.5
        1322.7
        1204.1
        1183.2
        1216.0
        1251.8
        1217.8
        1189.7
        1092.7
        992.0
        880.1
        821.7
        773.5
        754.2
        703.2
        698.0
        734.7
        706.4
        657.6
        631.1
        574.6
        543.3
        519.0
        535.5
        564.7
        570.3
        565.8
        563.8
        622.6
        716.9
        749.9
        678.4
        592.2
        475.0
        393.3
        330.6
        269.1
        238.2
        211.3
        200.1
        228.0
        214.2
        189.8
        158.7
        113.0
        90.8
        61.8
        40.7
        22.9
        20.1
        21.8
        28.9
        53.3
        86.7
        98.1
        121.9
        141.7
        165.8
        208.5
        268.7
        311.2
        395.0
        534.4
        595.7
        637.2
        734.0
        863.8
        843.9
        847.5
        775.2
        683.0
        615.2
        472.7
        348.6
        250.6
        205.4
        219.3
        229.1
        253.8
        229.5
        199.6
        163.6
        157.2
        143.2
        139.3
        155.9
        169.3
        169.1
        192.1
        201.3
        225.7
        220.2
        202.2
        163.2
        111.3
        107.4
        91.5
        65.2
        61.4
        75.5
        107.9
        197.0
        294.9
        378.5
        482.9
        596.4
        715.8
        782.0
        781.5
        738.7
        683.9
        714.1
        731.0
        741.7
        798.8
        868.6
        872.1
        849.5
        934.7
        1058.5
        1103.0
        1019.0
        886.1
        805.8
        823.5
        861.8
        912.9
        930.4
        941.9
        924.8
        857.6
        789.5
        763.4
        778.6
        788.2
        769.5
        773.1
        748.7
        737.2
        739.9
        780.5
        900.5
        978.7
        934.4
        803.4
        646.0
        579.8
        531.1
        518.6
        485.0
        426.7
        388.2
        351.4
        325.5
        300.2
        287.9
        226.2
        156.1
        160.3
        135.3
        141.2
        152.5
        218.5
        250.9
        244.1
        195.3
        168.6
        175.3
        183.9
        191.6
        196.5
        215.4
        243.6
        261.0
        265.2
        263.1
        269.2
        294.1
        293.8
        219.6
        178.5
        222.0
        263.9
        294.8
        290.4
        252.1
        192.1
        161.5
        111.8
        62.0
        44.7
        52.2
        46.8
        52.8
        61.8
        75.8
        109.3
        161.4
        210.2
        234.6
        251.2
        299.2
        303.1
        336.9
        426.4
        513.9
        631.3
        896.2
        1070.2
        1067.9
        1117.6
        1240.2
        1182.6
        1088.4
        976.3
        939.6
        980.2
        938.7
        865.5
        678.8
        533.4
        489.9
        431.9
        385.5
        411.2
        366.4
        322.6
        281.7
        243.2
        221.5
        171.4
        131.9
        121.0
        106.3
        124.1
        110.7
        90.6
        78.9
        54.4
        29.8
        19.7
        12.3
        9.4
        11.3
        10.5
        11.3
        23.2
        27.9
        28.9
        26.2
        33.0
        44.1
        45.1
        37.5
        34.2
        27.5
        32.1
        39.9
        58.9
        94.7
        141.5
        176.7
        205.0
        255.7];

        consumption=[
        1856.8
        1787.0
        1708.9
        1625.6
        1561.0
        1561.9
        1617.1
        1677.7
        1736.0
        1791.7
        1904.8
        1998.1
        2013.9
        2003.2
        2016.1
        2071.6
        2291.1
        2499.6
        2453.5
        2284.3
        2142.7
        2006.2
        1854.5
        1695.3
        1594.8
        1535.9
        1503.3
        1513.4
        1560.4
        1717.6
        2125.5
        2614.3
        2853.6
        2866.1
        2926.8
        2936.9
        2850.1
        2852.8
        2810.6
        2748.6
        2927.9
        3211.8
        3073.6
        2846.3
        2637.1
        2451.1
        2236.4
        2016.1
        1882.9
        1804.9
        1786.9
        1797.6
        1840.2
        2007.3
        2408.3
        2920.9
        3130.1
        3130.1
        3211.7
        3244.4
        3204.0
        3260.5
        3212.2
        3135.3
        3233.7
        3411.9
        3207.6
        2980.7
        2770.4
        2595.7
        2376.4
        2129.8
        1995.0
        1913.0
        1880.6
        1887.6
        1920.3
        2086.8
        2528.5
        3049.1
        3207.7
        3156.3
        3187.9
        3152.8
        3070.3
        3092.8
        3056.3
        2996.5
        3158.2
        3386.3
        3215.1
        2987.8
        2792.9
        2585.0
        2353.3
        2173.3
        1991.5
        1925.0
        1909.9
        1907.1
        1951.1
        2064.8
        2516.1
        3057.5
        3228.7
        3164.5
        3186.4
        3127.6
        3053.5
        3076.8
        3037.3
        3008.0
        3164.2
        3384.1
        3209.0
        2978.5
        2760.6
        2555.1
        2334.9
        2118.9
        1958.4
        1883.4
        1851.1
        1859.8
        1912.5
        2083.9
        2532.7
        3085.1
        3229.2
        3150.7
        3144.4
        3048.9
        2886.0
        2779.9
        2652.6
        2598.1
        2806.9
        3117.8
        3007.0
        2748.1
        2505.9
        2333.2
        2160.9
        1985.8
        1847.5
        1751.5
        1699.5
        1692.5
        1708.2
        1754.8
        1879.0
        2089.5
        2281.4
        2399.0
        2458.6
        2447.2
        2390.2
        2352.9
        2330.9
        2349.7
        2551.1
        2846.8
        2760.2
        2506.6
        2326.6
        2184.3
        2030.6
        1900.7
        1773.5
        1679.8
        1624.9
        1607.3
        1609.2
        1650.3
        1755.0
        1924.1
        2102.1
        2222.5
        2292.8
        2319.6
        2284.5
        2271.0
        2276.2
        2311.0
        2524.9
        2817.2
        2741.4
        2533.2
        2365.3
        2201.1
        2028.3
        1865.0
        1746.1
        1703.3
        1683.8
        1688.5
        1747.9
        1947.8
        2449.9
        3011.9
        3187.6
        3134.9
        3176.9
        3171.9
        3100.2
        3116.7
        3083.1
        3007.4
        3112.6
        3339.3
        3169.3
        2959.3
        2750.1
        2572.4
        2424.0
        2236.8
        1960.4
        1867.5
        1824.5
        1824.3
        1867.8
        2028.8
        2514.3
        3060.8
        3194.1
        3113.3
        3115.8
        3068.3
        2978.8
        3039.4
        3024.7
        2940.0
        3058.7
        3303.2
        3118.1
        2908.1
        2700.4
        2516.7
        2299.7
        2075.5
        1916.9
        1846.3
        1822.0
        1831.4
        1874.4
        2033.9
        2500.8
        3056.6
        3228.0
        3152.4
        3163.2
        3115.7
        2980.0
        2990.1
        2931.6
        2831.8
        2972.8
        3292.3
        3134.2
        2930.2
        2720.7
        2542.0
        2319.8
        2093.5
        1942.1
        1870.3
        1850.2
        1863.8
        1912.8
        2074.4
        2541.1
        3085.6
        3276.1
        3243.2
        3288.3
        3266.7
        3150.9
        3134.9
        3051.2
        2956.8
        3065.8
        3348.5
        3187.9
        2972.5
        2772.6
        2576.6
        2349.1
        2126.7
        1966.4
        1896.0
        1885.7
        1888.4
        1925.3
        2099.7
        2558.2
        3091.6
        3242.7
        3153.7
        3135.9
        3038.6
        2883.9
        2792.6
        2669.4
        2613.5
        2798.1
        3132.9
        3023.5
        2758.2
        2515.7
        2355.2
        2186.1
        2011.8
        1865.1
        1765.7
        1713.0
        1696.6
        1702.2
        1750.6
        1884.1
        2107.1
        2327.8
        2411.5
        2422.6
        2379.7
        2311.3
        2259.8
        2259.0
        2284.9
        2476.8
        2833.5
        2737.8
        2484.0
        2317.8
        2190.2
        2059.7
        1930.1
        1792.0
        1699.6
        1653.8
        1642.3
        1659.7
        1705.5
        1809.7
        1984.7
        2158.8
        2257.1
        2273.5
        2252.9
        2192.2
        2162.9
        2150.5
        2182.1
        2413.2
        2825.3
        2736.7
        2522.7
        2355.7
        2211.0
        2045.8
        1874.5
        1749.5
        1697.8
        1683.6
        1694.5
        1751.2
        1938.1
        2435.6
        2987.0
        3166.4
        3112.9
        3168.3
        3154.7
        3065.6
        3094.2
        3045.3
        2941.7
        3046.1
        3302.0
        3122.2
        2904.1
        2694.1
        2501.6
        2281.0
        2056.6
        1888.1
        1807.8
        1785.2
        1794.3
        1845.0
        2012.9
        2492.6
        3037.6
        3187.1
        3122.7
        3143.0
        3090.3
        3006.1
        3021.1
        2963.1
        2863.9
        2969.6
        3302.2
        3143.0
        2926.7
        2698.2
        2511.7
        2304.7
        2072.2
        1912.4
        1832.5
        1792.4
        1802.8
        1860.7
        2037.9
        2520.2
        3066.3
        3217.9
        3165.9
        3185.2
        3182.2
        3108.7
        3138.0
        3106.9
        3068.6
        3189.9
        3390.2
        3185.4
        2954.9
        2739.0
        2552.7
        2322.6
        2094.1
        1918.5
        1849.5
        1830.2
        1838.0
        1898.5
        2071.0
        2538.8
        3076.9
        3245.3
        3166.4
        3205.8
        3172.4
        3076.3
        3103.9
        3070.2
        2997.0
        3104.9
        3348.9
        3151.7
        2926.9
        2758.9
        2567.0
        2331.2
        2095.3
        1928.2
        1845.4
        1822.4
        1824.2
        1880.5
        2045.9
        2495.6
        3052.7
        3185.8
        3128.0
        3134.3
        3054.9
        2920.3
        2857.7
        2752.0
        2658.0
        2776.6
        3094.1
        2986.7
        2715.4
        2495.7
        2337.0
        2162.6
        1994.8
        1850.7
        1751.7
        1709.6
        1694.2
        1694.5
        1763.4
        1911.2
        2148.6
        2411.1
        2562.0
        2636.4
        2631.9
        2548.6
        2454.2
        2396.8
        2400.5
        2538.7
        2886.2
        2735.1
        2485.6
        2318.7
        2189.1
        2055.0
        1906.0
        1777.5
        1688.3
        1631.9
        1612.7
        1624.8
        1682.6
        1789.2
        1964.4
        2146.1
        2250.4
        2314.7
        2342.6
        2309.9
        2300.3
        2317.0
        2319.6
        2477.8
        2841.3
        2783.1
        2584.1
        2408.3
        2271.2
        2109.9
        1939.2
        1830.0
        1784.0
        1772.1
        1780.8
        1842.8
        2047.0
        2541.1
        3111.6
        3250.9
        3187.8
        3199.1
        3143.5
        3029.2
        3029.1
        2987.9
        2875.9
        2970.7
        3366.8
        3171.3
        2962.3
        2795.6
        2596.9
        2386.4
        2164.8
        1998.9
        1924.1
        1900.3
        1915.0
        1966.6
        2132.9
        2618.7
        3173.9
        3303.4
        3225.4
        3222.9
        3149.5
        3048.1
        3036.1
        2972.5
        2867.9
        2952.5
        3373.8
        3231.2
        3032.2
        2828.8
        2648.9
        2435.7
        2209.6
        2049.0
        1976.5
        1968.5
        1975.4
        2024.7
        2194.6
        2682.4
        3238.4
        3347.1
        3270.8
        3263.3
        3229.1
        3148.3
        3165.3
        3127.9
        3028.2
        3115.2
        3446.8
        3271.5
        3034.3
        2870.6
        2682.0
        2448.1
        2225.1
        2071.0
        2002.6
        1987.8
        2002.5
        2055.9
        2232.9
        2712.7
        3253.7
        3370.4
        3305.5
        3306.3
        3254.1
        3135.4
        3140.3
        3081.5
        2978.5
        3052.8
        3426.6
        3310.9
        3106.8
        2912.7
        2717.1
        2491.7
        2255.5
        2101.1
        2018.1
        1984.5
        1988.3
        2044.8
        2213.0
        2668.6
        3197.6
        3349.6
        3317.4
        3356.3
        3293.3
        3167.0
        3056.6
        2902.4
        2814.9
        2918.8
        3212.6
        3042.2
        2811.4
        2598.1
        2426.0
        2261.9
        2084.0
        1932.7
        1840.1
        1796.8
        1778.4
        1788.7
        1851.9
        1988.6
        2213.9
        2403.4
        2506.2
        2534.8
        2523.9
        2464.2
        2409.1
        2384.9
        2408.2
        2554.4
        2910.7
        2838.4
        2597.8
        2413.2
        2269.9
        2145.5
        2001.2
        1866.5
        1781.0
        1733.1
        1720.5
        1738.1
        1784.1
        1892.1
        2075.6
        2260.7
        2397.7
        2460.8
        2487.4
        2443.1
        2415.3
        2424.2
        2432.2
        2565.3
        2815.7
        2798.2
        2693.3
        2556.6
        2395.7
        2204.0
        2034.8
        1940.8
        1888.0
        1878.0
        1901.2
        1964.2
        2151.2
        2647.7
        3205.9
        3303.6
        3253.3
        3256.3
        3217.5
        3135.1
        3133.1
        3072.3
        2954.4
        2988.2
        3411.5
        3330.3
        3129.3
        2932.1
        2727.2
        2508.2
        2288.6
        2122.5
        2052.5
        2029.1
        2038.9
        2092.3
        2254.9
        2751.9
        3290.4
        3365.0
        3292.3
        3280.1
        3225.9
        3127.6
        3118.0
        3044.8
        2936.1
        2978.4
        3384.8
        3339.0
        3131.5
        2921.2
        2736.0
        2511.8
        2270.8
        2122.8
        2051.5
        2025.9
        2036.1
        2104.1
        2278.6
        2751.4
        3302.0
        3380.9
        3321.7
        3321.6
        3278.7
        3173.5
        3180.6
        3114.3
        2996.7
        3028.0
        3407.3
        3350.1
        3140.7
        2935.5
        2746.5
        2530.0
        2291.5
        2128.7
        2055.2
        2028.7
        2044.8
        2110.4
        2284.7
        2774.0
        3319.1
        3407.4
        3349.7
        3334.4
        3294.9
        3187.6
        3195.4
        3139.4
        3027.4
        3068.0
        3437.7
        3389.6
        3186.2
        2978.7
        2782.0
        2557.9
        2332.4
        2172.5
        2093.1
        2077.5
        2083.7
        2146.2
        2323.7
        2801.5
        3344.0
        3444.8
        3380.1
        3388.6
        3315.2
        3160.8
        3066.7
        2918.9
        2835.1
        2914.0
        3294.9
        3304.4
        3050.2
        2821.5
        2645.8
        2497.2
        2341.1
        2197.1
        2096.2
        2058.3
        2048.4
        2085.3
        2138.6
        2269.9
        2507.9
        2680.8
        2799.5
        2808.2
        2760.1
        2671.5
        2584.2
        2533.8
        2529.7
        2645.4
        3065.5
        3102.6
        2861.8
        2685.6
        2543.4
        2414.7
        2277.6
        2154.1
        2070.6
        2027.9
        2011.5
        2019.5
        2068.2
        2179.5
        2362.2
        2522.0
        2639.9
        2671.4
        2661.3
        2602.2
        2561.7
        2548.3
        2555.1
        2692.6
        3114.1
        3154.6
        2957.8
        2773.5
        2614.1
        2441.4
        2285.2
        2169.6
        2119.5
        2111.3
        2117.9
        2178.6
        2368.2
        2859.0
        3393.3
        3497.4
        3486.8
        3502.9
        3478.9
        3383.4
        3367.5
        3312.2
        3201.2
        3221.6
        3599.2
        3555.6
        3348.8
        3146.9
        2948.0
        2727.6
        2506.2
        2363.5
        2299.6
        2280.3
        2286.5
        2338.4
        2509.9
        2986.7
        3527.1
        3603.4
        3564.2
        3549.6
        3490.2
        3356.2
        3334.9
        3265.7
        3136.8
        3160.5
        3549.8
        3547.0
        3349.8
        3161.7
        2966.5
        2744.6
        2520.9
        2361.8
        2296.4
        2274.3
        2282.6
        2330.6
        2494.6
        2969.9
        3483.0
        3582.8
        3566.8
        3585.7
        3539.6
        3439.8
        3435.2
        3367.9
        3246.1
        3207.2
        3530.4
        3506.1
        3287.9
        3087.5
        2893.4
        2662.8
        2437.4
        2266.5
        2200.2
        2163.4
        2168.8
        2222.8
        2407.6
        2883.2
        3406.1
        3470.1
        3407.8
        3408.0
        3351.1
        3248.8
        3263.2
        3202.1
        3083.1
        3096.6
        3414.6
        3425.5
        3234.7
        3035.6
        2852.8
        2641.7
        2418.9
        2264.5
        2200.9
        2181.2
        2191.3
        2234.0
        2401.0
        2876.1
        3398.0
        3471.4
        3414.4
        3400.0
        3313.5
        3160.1
        3048.8
        2901.6
        2804.5
        2851.6
        3182.8
        3260.5
        3016.7
        2796.0
        2628.6
        2484.3
        2325.0
        2167.1
        2090.0
        2033.8
        2024.2
        2039.5
        2098.2
        2231.3
        2444.8
        2630.0
        2764.8
        2793.8
        2760.0
        2677.7];    

        rd = 2*windproduction-consumption; rd = rd./max(abs(rd));
    end