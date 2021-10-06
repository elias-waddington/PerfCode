


% Some type of function to calculate max thrust array for PEM FC

%load FC info
FCmap_drag = allloadin_V003('FCmapR2_1214_drag.dat','n');
FCmap_eff = allloadin_V003('FCmapR2_1214_eff.dat','n');
load_R2_1214_cor; 
% FCmap_drag = allloadin_V003('FCmapR1_LD_drag.dat','n');
% FCmap_eff = allloadin_V003('FCmapR1_LD_eff.dat','n');
% load_LD1_LowDrag; 

%Sample FC size
FC_size = 100;

altitude = linspace(0,37500,38);
Mach = linspace(0.25,0.77,8);
NetthrustMatrix = zeros(length(altitude),length(Mach));
MaxthrustMatrix = zeros(length(altitude),length(Mach));

for a = 1:(length(altitude))
    for m = 1:length(Mach)
        Max_Power = interp2(MP_h_rng,MP_M_rng,MP_Wlb_mat,altitude(a),Mach(m)); %watts
        velocity = speedofsound(altitude(a))*Mach(m); %fps
        mp = linspace(100,Max_Power,10);
        NetThrust = zeros(length(mp),1);
        for i = 1:length(mp)
            HexDrag = interp3D_V003(FCmap_drag,1,mp(i),Mach(m),altitude(a),'n'); %pounds of drag/pound of FC)
            Thrust = mp(i)*1.3558/velocity*FC2fan(altitude(a));
            NetThrust(i) = Thrust-HexDrag;
            if NetThrust(i) < 0
                NetThrust(i) = 0;
            end
        end
        x = find(NetThrust==max(NetThrust));
        NetthrustMatrix(a,m) = NetThrust(x);
        MaxthrustMatrix(a,m) = mp(x);
    end
end

