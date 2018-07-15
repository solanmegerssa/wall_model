
ts = sim_total_potable.time;
sim_potable = zeros(size(ts,1),1);
sim_rec = zeros(size(ts,1),1);
for i = 1:size(ts,1)-1
    sim_potable(i) = sim_total_potable.data(i+1) - sim_total_potable.data(i);
    sim_rec(i) = sim_total_recycled.data(i+1) - sim_total_recycled.data(i);
end
sim_v_rec = 140/1.5 - ((10+14.7)*(140/1.5)^1.4./(sim_p_rec.data + 14.7)).^(1/1.4);

figure
plot(sim_potable);
hold on
plot(sim_rec);
legend("potable use","recycled use")
xlabel("time")
ylabel("gallons")

figure
hold on
plot(sim_vol_cist);
plot(sim_v_rec);
plot(timeseries(sim_v_rec,ts))
xlabel("time")
ylabel("gallons")
legend("volume rain", "volume grey")
title("water balance")

figure
plot(sim_cistern_intake);
xlabel("time")
ylabel("gallons/min")
title("cistern intake")

figure
plot(rain_series)
xlabel("time")
ylabel("inches")
title("total rain fall")
xlim([ts(1),ts(end)])

%%
filename = "WaterBalanceNormal.xlsx";
water_balance_data = zeros(size(sim_potable));
water_balance_data(:,1) = sim_vol_cist.data;
water_balance_data(:,2) = sim_vol_grey.data;
water_balance_data(:,3) = sim_v_rec;
water_balance_data(:,4) = sim_potable;
water_balance_data(:,5) = sim_rec;
xlswrite(filename, water_balance_data)