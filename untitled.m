n_days = 30
hours = 3600*linspace(1,24*n_days,24*n_days);
sample_vol = resample(data_vol_cistern,hours);
sample_potable = resample(data_potable,hours);
sample_rain = resample(data_rain,hours);
sample_recycle = resample(data_recycle,hours);

%%
avgs = zeros(n_days,3);
cist_vol = zeros(n_days,1);
for i = 1:n_days
    lower = 1+24*(i-1);
    upper = i*24;
    
    daily_avg_pot = mean(sample_potable.Data(lower:upper));
    avgs(i,1) = 24*3600*daily_avg_pot;
    
    daily_avg_vol = mean(sample_vol.Data(lower:upper));
    cist_vol(i) = daily_avg_vol;
    
    daily_rain = mean(sample_rain.Data(lower:upper));
    avgs(i,2) = 24*3600*daily_rain;
    
    daily_recycle = mean(sample_recycle.Data(lower:upper));
    avgs(i,3) = 24*3600*daily_recycle;
end

%%
bar(days,cist_vol);
xlabel("Days")
ylabel("Gallons")
hold on
days = linspace(1,n_days,n_days);
bar(days,avgs)
xlabel("Days")
ylabel("Gallons")
legend("cistern volume", "water consumption", "rainfall", "water recycled")

hold off

