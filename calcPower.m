function P = calcPower(T,V,h)
    ThrustPower = T*V;
%     prop_eff = FanEff(h);
%     efficiency = ; %Fuel Cell demand to propulsive power
    P = ThrustPower / FC2fan(h) * 1.3558;
end