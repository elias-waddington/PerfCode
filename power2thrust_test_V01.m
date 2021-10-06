load('V03_ac.mat')
load('V03_weight.mat')
CLto = 2.2;

M0 = .77;
h = 37000;

[Tout,Pout] = power2thrust_v01(M0,ac,h);

% Pshaft = FC_C_size*interp2(MP_h_rng,MP_M_rng,MP_Wlb_mat);

PShaft = 22473534.91;
Ta = interp1(Pout,Tout,PShaft)
% Pshaft = interp1(Tout,Pout,11645); % (Shaft Power)
% 
% Pfc = Pshaft/.995/.97/.99/.99
