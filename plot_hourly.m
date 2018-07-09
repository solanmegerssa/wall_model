
ts = data_total_potable.time;
potable = zeros(size(ts,1),1);
rec = zeros(size(ts,1),1);
for i = 1:size(ts,1)-1
    potable(i) = data_total_potable.data(i+1) - data_total_potable.data(i);
    rec(i) = data_total_recycled.data(i+1) - data_total_recycled.data(i);
end

figure
bar(potable);
hold on
bar(rec);
legend("potable use","recycled use")

figure
hold on
plot(data_vol_cist);
plot(data_vol_grey);

legend("volume rain", "volume grey")
