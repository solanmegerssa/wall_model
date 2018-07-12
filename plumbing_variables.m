%% Parameters of interest for WALL water system
days = 20;
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


%%
water_use = xlsread('Data/SampleWaterUse2.xlsx','C17:H40');
t = 1:30:24*3600*days;
demand_profiles = zeros(days,2880,6);
time_per_use = [7.8*60 30 30 30 9.8*60 6.7*60];
freq_dev = [0.1 0.75 2 2 0.1 0.1];
for i = 1:days
    for j = 0:23
        hour_start = j*3600; 
        hour_end = (j+1)*3600;
        for z = 1:6
            freq = round(normrnd(water_use(j+1,z),freq_dev(z)));
            if freq > 0
                if freq > 1
                    start_times = linspace(hour_start,hour_end,freq);
                else
                    start_times = [(hour_start+hour_end)/2];
                end
                 
                end_times = start_times + time_per_use(z);
                for u = 1:size(start_times,2)
                    
                    t_start = floor(floor(start_times(u)/30) + 1);
                    t_end = floor(floor(end_times(u)/30)+1);
                    
                    if t_end < 2881
                        demand_profiles(i,t_start:t_end,z) = 1;
                    end
                    
                end
            end
        end
    end
end

demand_series = zeros(size(t,2),6);
for i = 0:(days-1)
    demand_series((1+2880*i):2880*(i+1),:) = squeeze(demand_profiles(i+1,:,:));
end

demand_profiles = timeseries(demand_series,t);
         

