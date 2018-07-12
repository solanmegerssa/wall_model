
ts = data_total_potable.time;
potable = zeros(size(ts,1),1);
rec = zeros(size(ts,1),1);
for i = 1:size(ts,1)-1
    potable(i) = data_total_potable.data(i+1) - data_total_potable.data(i);
    rec(i) = data_total_recycled.data(i+1) - data_total_recycled.data(i);
end
v_rec = 140/1.5 - ((10+14.7)*(140/1.5)^1.4./(data_p_rec.data + 14.7)).^(1/1.4);

figure
plot(potable);
hold on
plot(rec);
legend("potable use","recycled use")

figure
hold on
plot(data_vol_cist);
plot(data_vol_grey);
plot(timeseries(v_rec,ts))

legend("volume rain", "volume grey", "volume recycled")

%%
filename = "WaterBalance.xlsx";
v_rec = 140/1.5 - ((10+14.7)*(140/1.5)^1.4./(data_p_rec.data + 14.7)).^(1/1.4);
water_balance_data = zeros(481,5);
water_balance_data(:,1) = data_vol_cist.data;
water_balance_data(:,2) = data_vol_grey.data;
water_balance_data(:,3) = v_rec;
water_balance_data(:,4) = potable;
water_balance_data(:,5) = rec;
xlswrite(filename, water_balance_data)