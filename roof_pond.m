%%
Ts = 1/4; % sampling period [hrs]
Np = 24/Ts;     % prediction horizon
Ns = 3*Np;     	% simulation horizon [1 = openloop]
Nd = Np+Ns-1;   % simulation data horizon

system = setUpSystem()