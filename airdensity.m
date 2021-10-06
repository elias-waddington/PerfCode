function rho = airdensity(h)
    
    if h < 36089
        Sigma = (1.0 - h/145442)^(4.255876);
    else
        Sigma = 0.297076 * exp( (36089 - h)/20806 );
    end
    
    rhoo = 0.00237689;
    
    rho = rhoo*Sigma;
end