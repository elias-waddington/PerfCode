% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SCRIPT:  'allinterp.m'
% VERSION: 1.0
% AUTHOR:  Jason M Merret
% O. DATE: 8-3-2008
% M. DATE: 8-3-2008 
% DEPT:    Preliminary Design - 0686
% DESC:    This is script interpolates a TAPE22 file or a TECPLOT file.
%            This script does not read in the table from the input file, it
%            wants the input data structures directly as input.  This was
%            designed for for TAPE22 and TECPLOT file, but can be used with
%            any table of the proper format.  If the user want to use this
%            as a plain interpolater then repeat the data structure in both
%            inputs and specify 'FN' for the 'FNorWF' input.  This will
%            function just like any other interpolator.  In actuallity this
%            is solely a front for the intper3D routine that is tailored
%            for the Performance and PD group's file formats.
%
% NOTES:   A) There needs to be a version of this that is written for quick
%            conversion to C or C++.  That should be simple by changing a
%            few lines of code.  This can be accomplished by avoiding the
%            use of cell arrays and using for loops instead of the MATLAB :
%            syntax.
%
%          B) The Program functions differently for TAPE22 and TECPLOT 
%            file formats.  Since the TECPLOT file format has both FN and
%            WF in the file it is capable of producing both outputs.  The 
%            TAPE22 file has either FN of WF so only that interpolated  
%            corresponding value is possible. Therefore to acquare 
%            interpolated value for TAPE22, buth table must be submitted 
%            in the input 
%
%          C) The output could be combined for simplicity, but was kept
%            seperate for clariity right now.  In a lrge code it might be
%            cleaner to have it combined. The script line have been  
%            included to make this change, but have been commented out so 
%            they are not active. 
%
%
%
% INPUT:   'inFN'   - Input Thrust data structure
%            'inFN.name'  - Tablename aka (input file name)'
%            'inFNformat' - Table format (1=TECPLOT, 22=TAPE22)
%            'inFN.head'  - Table Header 
%            'inFN.nhead' - Number of Header Lines
%            'inFN.nums'  - iX , iY, and iZ indicies
%            'inFN.data'  - table data fmttd as (i,j,k,1) - X(Z)
%                         - table data fmttd as (i,j,k,2) - Y(Z)
%                         - table data fmttd as (i,j,k,3) - Z
%                         - table data fmttd as (i,j,k,4) - f(X(Z),Y(Z),Z)
%          'inWF'   - Input Fuel Flow data structure
%            'inWF.name'  - Tablename aka (input file name)'
%            'inWFformat' - Table format (1=TECPLOT, 22=TAPE22)
%            'inWF.head'  - Table Header 
%            'inWF.nhead' - Number of Header Lines
%            'inWF.nums'  - iX , iY, and iZ indicies
%            'inWF.data'  - table data fmttd as (i,j,k,1) - X(Z)
%                         - table data fmttd as (i,j,k,2) - Y(Z)
%                         - table data fmttd as (i,j,k,3) - Z
%                         - table data fmttd as (i,j,k,4) - f(X(Z),Y(Z),Z)
%          'xwant'  - Desired X dimension value
%          'ywant'  - Desired Y dimension value
%          'zwant'  - Desired Z dimension value
%          'dtl'    - Detail flag for dianostic output ('y' or 'n')
%          'FNorWF' - Type of Interpolation 
%                    - 'FN'      - Thrust interpolation
%                    - 'WF'      - Fuel flow interppolation
%                    - 'BOTH'    - Thrust and Fuel Flow interpolation
%                    - 'WFPPTEC' - Special Fuel Flow interpolation for the
%                      TECPLOT file format that references the WF to FN 
%                      through a "throttle" value which is a % of maximum 
%                      thrust at that condition.
%          'FNmult'  - Thrust multiplier
%          'WFmult'  - Fuel Flow multiplier
%
% OUTPUT:  'valFN'   - Interpolated thrust value
%          'valWF'   - Interpolated fuel flow value
%          'maxFN'   - Maximum thrust for that condition (TECPLOT only)
%          'minFN'   - Minimum thrust for that condition (TECPLOT only)
%          'idleFN'  - Idle thrust for that condition (TECPLOT only)
%          'thrtl'   - % of Max thrust value (TECPLOT only)
% 
%
% REVISION HISTORY:
% REV:     DATE:           CONTACT:       DESCRIPTION:
% 3.0      03/25/2020      JMM            Sets all to the same version #
% 
%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [valFN valWF maxFN minFN idleFN thrtl]=...
  allinterp_V003(inFN,inWF,xwant,ywant,zwant,dtl,FNorWF,FNmult,WFmult)

%function[rtnvlues]=allinterp(inFN,inWF,...
%    xwant,ywant,zwant,dtl,FNorWF,FNmult,WFmult)

tabformat = inFN.format;

% Converts the input to all upper case
FNorWF=upper(FNorWF);

% Sting comparison for the type of interpolation
a1 = strcmp(FNorWF,'FN');
a2 = strcmp(FNorWF,'WF');
a3 = strcmp(FNorWF,'BOTH');
a4 = strcmp(FNorWF,'WFPPTEC');

% This is the Interpolation routine for the TECPLOT file format
% It has a lot more options than the TAPE22 format because the TECPLOT
% format has a lot more information in it.  
if (tabformat==1) 
% This only calculates the FN value based on Mach(X), Altitude(Y), 
% and Throttle/%Max Thrust (Z) 
  if (a1==1)
    valFN = interp3D_V003(inFN,1,xwant,ywant,zwant,dtl);
    valWF = 9999999.;
    thrtl = zwant;
    maxFN = 9999999.;
    minFN = 9999999.;
    idleFN = 9999999.;
% This only calculates the WF value based on Mach(X), Altitude(Y), 
% and Throttle/%Max Thrust (Z) 
  elseif (a2==1)
    valWF = interp3D_V003(inWF,1,xwant,ywant,zwant,dtl);
    valFN = 9999999.;
    thrtl = zwant;
    maxFN = 9999999.;
    minFN = 9999999.;
    idleFN = 9999999.;    
% This calculates both the FN and WF values based on Mach(X), Altitude(Y), 
% and Throttle/%Max Thrust (Z)     
  elseif (a3==1)
    valFN = interp3D_V003(inFN,1,xwant,ywant,zwant,dtl);
    valWF = interp3D_V003(inWF,1,xwant,ywant,zwant,dtl);
    thrtl = zwant;
    maxFN = 9999999.;
    minFN = 9999999.;
    idleFN = 9999999.;
    
% This calculates the WF value based on Mach(X), Altitude(Y), and 
% specified thrust(Z).  This would mimic the TAPE22 Part power table.  
% The routine is a bit different because the TECPLOT tables are a function
% of the maximum power in the Z direction.  In addition the TECPLOT format
% interpolates the last two values.  It does this to preserve the idle
% thrust and fuel flow.  The program assumes idle is throttle zero, but
% does NOT assume that thrust or FF are zero at that point.
  elseif (a4==1)
    if (dtl=='y')
      disp(' ')
      disp('*******************************')
      disp('Running and Printing the Max FN')
      disp(' ')
    end
% Obtaining the maximum thrust for the given condition
    maxFN = interp3D_V003(inFN,1,xwant,ywant,100.0,dtl);
    if (dtl=='y')
      disp(' ')
      disp('*******************************')
      disp(' ')
    end
% Obtaining the last two points in the Y dimension.  This could get very 
% complicated if the Y index becomes a function of X and Z.  I am going to
% really have to think how this would work in that case     
    idlethr1 = 0;
    idlethr2 = inFN.data(1,1,2,3);
    thr = zwant/maxFN*100;    
    if (thr<=idlethr2)
      if (dtl=='y')
        disp(' ')
        disp('*********************************')
        disp('Running and Printing the Idle FNs')
        disp(' ')
      end
% Obtaining the idle thrust to complete the interpolation for the points
% between the last thrust point and idle.
      idle1FN = interp3D_V003(inFN,1,xwant,ywant,idlethr1,dtl);
      idle2FN = interp3D_V003(inFN,1,xwant,ywant,idlethr2,dtl); 
      minFN = idle2FN;
      idleFN = idle1FN;
      thr=(idlethr2-idlethr1)/(idle2FN-idle1FN)*(zwant-idle1FN)...
          + idlethr1;      
      valWF = interp3D_V003(inWF,1,xwant,ywant,thr,dtl);
      valFN = 9999999.;
      thrtl = thr;
      if (dtl=='y')
        disp(' ')
        disp('*******************************')
        disp(' ')
      end
    else  
      valWF = interp3D_V003(inWF,1,xwant,ywant,thr,dtl);
      valFN = 9999999.;
      thrtl = thr;
      minFN = 9999999.;
      idleFN = 9999999.;
    end 
  else
    valFN = 9999999.;
    valWF = 9999999.;
    thrtl = 9999999.;
    maxFN = 9999999.;
    minFN = 9999999.;
    idleFN = 9999999.;    
  end

% Same but simpler process for the TAPE22 files  
else
    
    
    
  if (a1==1)
    valFN = interp3D_V003(inFN,1,xwant,ywant,zwant,dtl);
    valWF = 9999999.;
    thrtl = 9999999.;
    maxFN = 9999999.;
    minFN = 9999999.;
    idleFN = 9999999.;
  elseif (a2==1)
    valFN = 9999999.;
    valWF = interp3D_V003(inWF,1,xwant,ywant,zwant,dtl);
    thrtl = 9999999.;
    maxFN = 9999999.;
    minFN = 9999999.;
    idleFN = 9999999.;    
  elseif (a3==1)
    valFN = interp3D_V003(inFN,1,xwant,ywant,zwant,dtl);
    valWF = interp3D_V003(inWF,1,xwant,ywant,zwant,dtl);
    thrtl = 9999999.;
    maxFN = 9999999.;
    minFN = 9999999.;
    idleFN = 9999999.;       
  elseif (a4==1)
    valFN = 9999999.;
    valWF = 9999999.;
    thrtl = 9999999.;
    maxFN = 9999999.;
    minFN = 9999999.;
    idleFN = 9999999.;
  else
    valFN = 9999999.;
    valWF = 9999999.;
    thrtl = 9999999.;
    maxFN = 9999999.;
    minFN = 9999999.;
    idleFN = 9999999.;
  end  
end

% This adds the multipler to the thrust and fuel flows
if (valFN<900000)
  valFN = valFN*FNmult;
end

if (valWF<900000)
  valWF = valWF*WFmult;
end

if (maxFN<900000)
  maxFN = maxFN*FNmult;
end
if (minFN<900000)
  minFN = minFN*FNmult;
end
if (idleFN<900000)
  idleFN = idleFN*FNmult;
end


%rtnvalues = [valFN valWF maxFN minFN idleFN thrtl];

end
