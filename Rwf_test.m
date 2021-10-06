clear all


M = linspace(0.4,0.95);
ac.fuse.length = 130;

mission.altitude = 30000; %ft
mission.a = speedofsound(mission.altitude); %fps
mission.rho = airdensity(mission.altitude); %slugs/ft^3
mission.viscocity = airviscocity(mission.altitude);
% mission.v_cruise = mission.M*mission.a; %fps
% mission.qbar = 0.5*mission.v_cruise^2*mission.rho;

RLS = zeros(1,length(M));

for i = 1:length(M)
    mission.v_cruise = M(i)*mission.a; %fps
    RN = mission.rho*mission.v_cruise*ac.fuse.length/mission.viscocity;
    RLS(i) = Rwf_find(M(i),RN);


end

plot(M,RLS)    