%% Parameters of interest for WALL water system
days = 7;
tf = days*24*3600;
ts = 0:60:tf;
random_seed = rand();

% roof properties
roof_area = 2000 % ft^2
roof_cistern_pipe_length = 10; % feet
roof_height = 15; % feet

% swh heater properties
swh_valve_open = 0;
swh_volume = 2500; % gallons
swh_inlet_diam = 2; % inches
swh_outlet_diam = 2; % inches

% cistern properties
roof_valve_open = 1;
cistern_inlet_height = 5; % feet
cistern_diam = 100; % inches
cistern_inlet_diam = 2; % inches
cistern_outlet_diam = 2; % inches
cistern_pump_pipe_length = 1; % feet

pump_inlet_area = 3.14*1^2; % inches^2

% pressure tank properties
ptank_pipe_length = 10; % ft
ptank_diam =  15; % inches
ptank_length = 31; % inches

% hot water heater properties
heater_pipe_length = 5; % ft
heater_diam = 20; % in
heater_length = 60; % in





