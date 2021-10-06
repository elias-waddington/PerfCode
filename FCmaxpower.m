function maxpower = FCmaxpower(weight,h,M)
    

    
%     load_LD1_LowDrag; 
%     load_R2_current_LD;
    % load_R2_current_LD_LTD;
%     load_R2_SOFC;
%     load_R2_SOFCx2;
%     load_R2_SOFCx3;
    load_R2_1214_cor; 
    if h>37500
        h = 37000;
    end
    maxpower = interp2(MP_h_rng,MP_M_rng,MP_Wlb_mat,h,M)*weight.fuelcell;
    
end
