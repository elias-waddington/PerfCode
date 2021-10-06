clear all

ctsweep = 25.5; %degs
M = linspace(0.4,.95);

for i = 1:length(M)
    RLS(i) = RLS_find(M(i),ctsweep);


end

plot(M,RLS)    