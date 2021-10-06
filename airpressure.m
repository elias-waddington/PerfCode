function P = airpressure(h)
    if h < 36089; % in ft
        Delta = (1-h/145442)^(5.255876);
    else
        Delta = 0.223361*exp((36089-h)/20806);
    end
    
    PresSL = 2116.224;
    P = PresSL*Delta;
    
end