% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SCRIPT:  'interp3D.m'
% VERSION: 3.0
% AUTHOR:  Jason M Merret
% O. DATE: 7-11-2008
% M. DATE: 09-13-2017 
% DEPT:    Preliminary Design - 0686
% DESC:    This script interpolates a 3D table.  The X and Y dimensions
%          are a function of the Z dimension.  The script does special
%          checking to determine if the desired values of the X-Y-Z
%          dimensions are on a good or bad face/vertex/edge of the "cube"
%          The error value for this script is any value greater than
%          900,000.
%
% INPUT:   'intable' -  Data structure containing
%                    - 'intable(#).name' - Tablename aka (input file name)
%                    - 'intable(#).format'- Orig. File Format (1=TEC,2=T22)
%                    - 'intable(#).head'   - Header from the orig. file
%                    - 'intable(#).nhead'  - number of headerlines
%                    - 'intable(#).nums' - iX , iY, and iZ indices
%                    - 'intable(#).data - table data
%          'ntable'  - table number
%          'xwant'   - Desired value of the X dimension
%          'ywant'   - Desired value of the Y dimension
%          'zwant'   - Desired value of the Z dimension
%          'dtl'     - Detail flag for diagnostic output ('y' or 'n')
%
% OUTPUT:  'interpval' - Interpolated value
%
% REVISION HISTORY:
% REV:     DATE:           CONTACT:       DESCRIPTION:
% 1.1      8-9-2008        J. Merret      Comments: Grammar corrections
% 2.0      10-29-2014      J. Merret      For a nums(iz)=1 the program ignores zwant 
% 3.0      09-13-2017      J. Merret      Corrects a Typo Found by Ryan Stanford 
%                                           Edge condition was incorrect
%                                           (fx1y2z2 to fx2y2z2)
%                                           (Line ~ 300 EC 9999981)
%                                            (xx1y1z2 to zx1y1z2)
%                                           (Line ~ 300 EC 9999989)
%
%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function interpval = interp3D_V003(intable,ntable,xwant,ywant,zwant,dtl)
version = 3.0;
% Converting the input table into useable variables
nums(1) = intable(ntable).nums(1);
nums(2) = intable(ntable).nums(2);
nums(3) = intable(ntable).nums(3);
datatable = intable(ntable).data;
want(1) = xwant;
want(2) = ywant;
want(3) = zwant;

% Initializing the variables
nse = zeros(2,3,2);
xlo = zeros(2);
xhi = zeros(2);
ylo = zeros(2);
yhi = zeros(2);

%ntotal = nums(1)*nums(2)*nums(3);

% Searching the Z dimension for the desired value of the Z dimension
% It finds the two values equal to or on either side of the desired value.
% When is does this is assigns this Z hi and lo values and indexes to 
% variables that will be used later when constructioning the interpolation
% cube.
if (nums(3)>1)
  ztest = 1;
% Searching the Z index
  for j=1:nums(3)-1
    if ((want(3)>=datatable(1,1,j,3)) && (want(3)<=datatable(1,1,j+1,3)))
      nse(1,3,1) = j;
      nse(1,3,2) = j+1;
      zlo = datatable(1,1,j,3);
      zhi = datatable(1,1,j+1,3);
      ztest = 0;
    end
  end
% If the desired value is not in the index then return the error value  
  if (ztest==1)
    interpval = 1999999.;
    return
  end
% If there is only one value of z then this checks that.  Since I put added
% the extra value in each dimension I may not need this anymore.  I could
% just check Z+1 instead and call it all good.
elseif (want(3)==datatable(1,1,1,3))
  nse(1,3,1) = 1;
  nse(1,3,2) = 2;
  zlo = datatable(1,1,1,3);
  zhi = datatable(1,1,2,3);
  ztest = 0;
% If the desired value is not equal to the index then returning the error 
%value 
elseif(want(3)~=datatable(1,1,1,3))
  want(3)=datatable(1,1,1,3);
  nse(1,3,1) = 1;
  nse(1,3,2) = 2;
  zlo = datatable(1,1,1,3);
  zhi = datatable(1,1,2,3);
  ztest = 0;
    %   interpval = 9999999.;
%   return;
end

if (ztest == 0)
%  nse(1,3,1)
%  nse(1,3,2)
%  zlo
%  zhi
end


% Reapeating the process for the X and Y dimensions.  The only difference
% here is that the X and Y dimensions are now a function of the Z
% dimension.
for i3=1:2
  if (nums(1)>1)
    itest = 1;
    for j=1:nums(1)-1
      if((want(1)>=datatable(j,1,nse(1,3,i3),1)) && ... 
              (want(1)<=datatable(j+1,1,nse(1,3,i3),1))) %consider adding backwards-facing case here
        nse(i3,1,1) = j;
        nse(i3,1,2) = j+1;
        xlo(i3) = datatable(j,1,nse(1,3,i3),1);
        xhi(i3) = datatable(j+1,1,nse(1,3,i3),1);
        itest = 0;
      end
    end
    if (itest == 1)
      interpval = 9999999.;
      return
    end
  elseif (want(1)==datatable(1,1,1,1))
    nse(i3,1,1) = 1;
    nse(i3,1,2) = 2;
    xlo(i3) = datatable(1,1,nse(1,3,i3),1);
    xlo(i3) = datatable(2,1,nse(1,3,i3),1);
  elseif (want(1)~=datatable(1,1,1,1))
    interpval = 9999999.;
    return
  end
  
  if (nums(2)>1)
    jtest = 1;
    for j=1:nums(2)-1
      if((want(2)>=datatable(1,j,nse(1,3,i3),2)) && ...
              (want(2)<=datatable(1,j+1,nse(1,3,i3),2)))
        nse(i3,2,1) = j;
        nse(i3,2,2) = j+1;
        ylo(i3) = datatable(1,j,nse(1,3,i3),2);
        yhi(i3) = datatable(1,j+1,nse(1,3,i3),2);
        jtest = 0;
      end
    end
    if (jtest == 1)
      interpval = 9999999.;
      return
    end
  elseif (want(2)==datatable(1,1,1,2))
    nse(i3,2,1) = 1;
    nse(i3,2,2) = 2;
    ylo(i3) = datatable(1,1,nse(1,3,i3),2);
    ylo(i3) = datatable(1,2,nse(1,3,i3),2);
  elseif (want(2)~=datatable(1,1,1,2))
    interpval = 9999999.;
    return
  end    
end

% This assingns all X(Z), Y(Z), Z, and f(X,Y,Z) values to simple vaiables
% The comments line a value that will never be used in the interpolation
% due to the fact that the X and Y are not functions of each other.
% for the first(forward) Z plane of the interpolation cube.
xx1y1z1 = datatable(nse(1,1,1),nse(1,2,1),nse(1,3,1),1);
yx1y1z1 = datatable(nse(1,1,1),nse(1,2,1),nse(1,3,1),2);
zx1y1z1 = datatable(nse(1,1,1),nse(1,2,1),nse(1,3,1),3);
fx1y1z1 = datatable(nse(1,1,1),nse(1,2,1),nse(1,3,1),4);

xx2y1z1 = datatable(nse(1,1,2),nse(1,2,1),nse(1,3,1),1);
yx2y1z1 = datatable(nse(1,1,2),nse(1,2,1),nse(1,3,1),2);
% This value is equal to zx1y1z1 since z is constant on this plane
%zx2y1z1 = datatable(nse(1,1,2),nse(1,2,1),nse(1,3,1),3);
fx2y1z1 = datatable(nse(1,1,2),nse(1,2,1),nse(1,3,1),4);

% This value is equal to xx1y1z1 since x is constant for the varying y
%xx1y2z1 = datatable(nse(1,1,1),nse(1,2,2),nse(1,3,1),1);
yx1y2z1 = datatable(nse(1,1,1),nse(1,2,2),nse(1,3,1),2);
% This value is equal to zx1y1z1 since z is constant on this plane
%zx1y2z1 = datatable(nse(1,1,1),nse(1,2,2),nse(1,3,1),3);
fx1y2z1 = datatable(nse(1,1,1),nse(1,2,2),nse(1,3,1),4);

% This value is equal to xx2y1z1 since x is constant for the varying y
%xx2y2z1 = datatable(nse(1,1,2),nse(1,2,2),nse(1,3,1),1);
yx2y2z1 = datatable(nse(1,1,2),nse(1,2,2),nse(1,3,1),2);
% This value is equal to zx1y1z1 since z is constant on this plane
%zx2y2z1 = datatable(nse(1,1,2),nse(1,2,2),nse(1,3,1),3);
fx2y2z1 = datatable(nse(1,1,2),nse(1,2,2),nse(1,3,1),4);


% This assigns all X(Z), Y(Z), Z, and f(X,Y,Z) values to simple vaiables
% for the second(aft) Z plane of the interpolation cube.
xx1y1z2 = datatable(nse(2,1,1),nse(2,2,1),nse(1,3,2),1);
yx1y1z2 = datatable(nse(2,1,1),nse(2,2,1),nse(1,3,2),2);
zx1y1z2 = datatable(nse(2,1,1),nse(2,2,1),nse(1,3,2),3);
fx1y1z2 = datatable(nse(2,1,1),nse(2,2,1),nse(1,3,2),4);

xx2y1z2 = datatable(nse(2,1,2),nse(2,2,1),nse(1,3,2),1);
yx2y1z2 = datatable(nse(2,1,2),nse(2,2,1),nse(1,3,2),2);
% This value is equal to zx1y1z2 since z is constant on this plane
%zx2y1z2 = datatable(nse(2,1,2),nse(2,2,1),nse(1,3,2),3);
fx2y1z2 = datatable(nse(2,1,2),nse(2,2,1),nse(1,3,2),4);

%This value is equal to xx1y1z2 since x is constant for the varying y
%xx1y2z2 = datatable(nse(2,1,1),nse(2,2,2),nse(1,3,2),1);
yx1y2z2 = datatable(nse(2,1,1),nse(2,2,2),nse(1,3,2),2);
% This value is equal to zx1y1z2 since z is constant on this plane
%zx1y2z2 = datatable(nse(2,1,1),nse(2,2,2),nse(1,3,2),3);
fx1y2z2 = datatable(nse(2,1,1),nse(2,2,2),nse(1,3,2),4);

%This value is equal to xx2y1z2 since x is constant for the varying y
%xx2y2z2 = datatable(nse(2,1,2),nse(2,2,2),nse(1,3,2),1);
yx2y2z2 = datatable(nse(2,1,2),nse(2,2,2),nse(1,3,2),2);
% This value is equal to zx1y1z2 since z is constant on this plane
%zx2y2z2 = datatable(nse(2,1,2),nse(2,2,2),nse(1,3,2),3);
fx2y2z2 = datatable(nse(2,1,2),nse(2,2,2),nse(1,3,2),4);




% This part eliminates the Y dimension by interpolaing each Y index and
% turns the cube into a plane
% This is the line connecting Y2 to Y1 at X1 and Z1
% It checks each of the f(X,Y,Z) values to see if anyone of them is the
% error code.  If the interpolation uses one of these values then another
% error code is returned.  
fx1z1 = (fx1y2z1-fx1y1z1)/(yx1y2z1-yx1y1z1)*(want(2)-yx1y1z1)+fx1y1z1;
if (yx1y1z1==want(2))
  if (fx1y1z1>900000)
      fx1z1 = 9999970;
  end
elseif (yx1y2z1==want(2))
  if (fx1y2z1>900000)
      fx1z1 = 9999971;
  end
elseif (want(2)>yx1y1z1 && want(2)<yx1y2z1)
  if (fx1y1z1>900000 || fx1y2z1>900000)
      fx1z1 = 9999972;
  end
end

% This is the line connecting Y2 to Y1 at X2 and Z1
% It checks each of the f(X,Y,Z) values to see if anyone of them is the
% error code.  If the interpolation uses one of these values then another
% error code is returned.  
fx2z1 = (fx2y2z1-fx2y1z1)/(yx2y2z1-yx2y1z1)*(want(2)-yx2y1z1)+fx2y1z1;
if (yx2y1z1==want(2))
  if (fx2y1z1>900000)
      fx2z1 = 9999973;
  end
elseif (yx2y2z1==want(2))
  if (fx2y2z1>900000)
      fx2z1 = 9999974;
  end
elseif (want(2)>yx2y1z1 && want(2)<yx2y2z1)
  if (fx2y1z1>900000 || fx2y2z1>900000)
      fx2z1 = 9999975;
  end
end

% This is the line connecting Y2 to Y1 at X1 and Z2
% It checks each of the f(X,Y,Z) values to see if anyone of them is the
% error code.  If the interpolation uses one of these values then another
% error code is returned.  
fx1z2 = (fx1y2z2-fx1y1z2)/(yx1y2z2-yx1y1z2)*(want(2)-yx1y1z2)+fx1y1z2;
if (yx1y1z2==want(2))
  if (fx1y1z2>900000)
      fx1z2 = 9999976;
  end
elseif (yx1y2z2==want(2))
  if (fx1y2z2>900000)
      fx1z2 = 9999977;
  end
elseif (want(2)>yx1y1z2 && want(2)<yx1y2z2)
  if (fx1y1z2>900000 || fx1y2z2>900000)
      fx1z2 = 9999978;
  end
end

% This is the line connecting Y2 to Y1 at X2 and Z2
% It checks each of the f(X,Y,Z) values to see if anyone of them is the
% error code.  If the interpolation uses one of these values then another
% error code is returned.  
fx2z2 = (fx2y2z2-fx2y1z2)/(yx2y2z2-yx2y1z2)*(want(2)-yx2y1z2)+fx2y1z2;
if (yx2y1z2==want(2))
  if (fx2y1z2>900000)
      fx2z2 = 9999979;
  end
elseif (yx2y2z2==want(2))
  if (fx2y2z2>900000)
      fx2z2 = 9999980;
  end
elseif (want(2)>yx2y1z2 && want(2)<yx2y2z2)
  if (fx2y1z2>900000 || fx2y2z2>900000)
      fx2z2 = 9999981;
  end
end

% This part turns the plane into a line
% This is the line connecting X2 to X1 at Z1
% It checks each of the f(X,Y,Z) values to see if anyone of them is the
% error code.  If the interpolation uses one of these values then another
% error code is returned.  
fz1 = (fx2z1-fx1z1)/(xx2y1z1-xx1y1z1)*(want(1)-xx1y1z1)+fx1z1;
if (xx1y1z1==want(1))
  if (fx1z1>900000)
      fz1 = 9999982;
  end
elseif (xx2y1z1==want(1))
  if (fx2z1>900000)
      fz1 = 9999983;
  end
elseif (want(1)>xx1y1z1 && want(1)<xx2y1z1)
  if (fx2z1>900000 || fx1z1>900000)
      fz1 = 9999984;
  end
end

% This is the line connecting X2 to X1 at Z2
% It checks each of the f(X,Y,Z) values to see if anyone of them is the
% error code.  If the interpolation uses one of these values then another
% error code is returned.  
fz2 = (fx2z2-fx1z2)/(xx2y1z2-xx1y1z2)*(want(1)-xx1y1z2)+fx1z2;
if (xx1y1z2==want(1))
  if (fx1z2>900000)
      fz2 = 9999985;
  end
elseif (xx2y1z2==want(1))
  if (fx2z2>900000)
      fz2 = 9999986;
  end
elseif (want(1)>xx1y1z2 && want(1)<xx2y1z2)
  if (fx2z2>900000 || fx1z2>900000)
      fz2 = 9999987;
  end
end

% This part turns the line into a point.
% This is the line connecting Z2 to Z1
% It checks each of the f(X,Y,Z) values to see if anyone of them is the
% error code.  If the interpolation uses one of these values then another
% error code is returned.  
f = (fz2-fz1)/(zx1y1z2-zx1y1z1)*(want(3)-zx1y1z1)+fz1;
if (zx1y1z1==want(3))
  if (fz1>900000)
      f = 9999988;
  end
elseif (zx1y1z2==want(3))
  if (fz2>900000)
      f = 9999989;
  end
elseif (want(3)>zx1y1z1 && want(3)<zx1y1z2)
  if (fz2>900000 || fz1>900000)
      f = 9999990;
  end
end

if (f>=9999900)
  f = 8888888.0;  
end

% This part is very important because it communicates what data is being
% used to compute the final function value.
if (dtl=='y')
  for ii=1:intable(ntable).nhead
    fprintf('%s\n',intable(ntable).head(ii,:))
  end
    fprintf('\n\n')
  fprintf('%6s %6s %6s\n','Num-X','Num-Y','Num-Z')
  fprintf('%6g %6g %6g\n\n',nums(1),nums(2),nums(3))
  fprintf('Indices\n')
  fprintf('%10g\n',nse(1,3,1))
  fprintf('%10g %10g\n',nse(1,1,1),nse(1,1,2))
  fprintf('%10g %10g\n\n',nse(1,2,1),nse(1,2,2))
  fprintf('%10g\n',nse(1,3,2))
  fprintf('%10g %10g\n',nse(2,1,1),nse(2,1,2))
  fprintf('%10g %10g\n\n',nse(2,2,1),nse(2,2,2))  
    
  fprintf('Index Values\n')  
  fprintf('%10.3f\n',zlo)
  fprintf('%10.3f %10.3f\n',xlo(1),xhi(1))
  fprintf('%10.3f %10.3f\n\n',ylo(1),yhi(1))
  fprintf('%10.3f\n',zhi)
  fprintf('%10.3f %10.3f\n',xlo(2),xhi(2))
  fprintf('%10.3f %10.3f\n\n',ylo(2),yhi(2))
  
  fprintf('Point X1-Y1-Z1               Point X2-Y1-Z1\n');
  fprintf('x     = %11.3f          x     = %11.3f\n', ... 
      datatable(nse(1,1,1),nse(1,2,1),nse(1,3,1),1), ... 
      datatable(nse(1,1,2),nse(1,2,1),nse(1,3,1),1));
  fprintf('y     = %11.3f          y     = %11.3f\n', ...
      datatable(nse(1,1,1),nse(1,2,1),nse(1,3,1),2), ...
      datatable(nse(1,1,2),nse(1,2,1),nse(1,3,1),2));
  fprintf('z     = %11.3f          z     = %11.3f\n', ...
      datatable(nse(1,1,1),nse(1,2,1),nse(1,3,1),3), ...
      datatable(nse(1,1,2),nse(1,2,1),nse(1,3,1),3));
  fprintf('value = %11.3f          value = %11.3f\n', ...
      datatable(nse(1,1,1),nse(1,2,1),nse(1,3,1),4), ...
      datatable(nse(1,1,2),nse(1,2,1),nse(1,3,1),4));
  fprintf('\n');
  
  
  fprintf('Point X1-Y2-Z1               Point X2-Y2-Z1\n');
  fprintf('x     = %11.3f          x     = %11.3f\n', ...
      datatable(nse(1,1,1),nse(1,2,2),nse(1,3,1),1), ...
      datatable(nse(1,1,2),nse(1,2,2),nse(1,3,1),1));
  fprintf('y     = %11.3f          y     = %11.3f\n', ...
      datatable(nse(1,1,1),nse(1,2,2),nse(1,3,1),2), ...
      datatable(nse(1,1,2),nse(1,2,2),nse(1,3,1),2));
  fprintf('z     = %11.3f          z     = %11.3f\n', ...
      datatable(nse(1,1,1),nse(1,2,2),nse(1,3,1),3), ...
      datatable(nse(1,1,2),nse(1,2,2),nse(1,3,1),3));
  fprintf('value = %11.3f          value = %11.3f\n', ...
      datatable(nse(1,1,1),nse(1,2,2),nse(1,3,1),4), ...
      datatable(nse(1,1,2),nse(1,2,2),nse(1,3,1),4));
  fprintf('\n\n');
  
  
  fprintf('Point X1-Y1-Z2               Point X2-Y1-Z2\n');
  fprintf('x     = %11.3f          x     = %11.3f\n', ...
      datatable(nse(2,1,1),nse(2,2,1),nse(1,3,2),1), ...
      datatable(nse(2,1,2),nse(2,2,1),nse(1,3,2),1));
  fprintf('y     = %11.3f          y     = %11.3f\n', ...
      datatable(nse(2,1,1),nse(2,2,1),nse(1,3,2),2), ...
      datatable(nse(2,1,2),nse(2,2,1),nse(1,3,2),2));
  fprintf('z     = %11.3f          z     = %11.3f\n', ...
      datatable(nse(2,1,1),nse(2,2,1),nse(1,3,2),3), ...
      datatable(nse(2,1,2),nse(2,2,1),nse(1,3,2),3));
  fprintf('value = %11.3f          value = %11.3f\n', ...
      datatable(nse(2,1,1),nse(2,2,1),nse(1,3,2),4), ...
      datatable(nse(2,1,2),nse(2,2,1),nse(1,3,2),4));
  fprintf('\n');
  
 
  fprintf('Point X1-Y2-Z2               Point X2-Y2-Z2\n');
  fprintf('x     = %11.3f          x     = %11.3f\n', ...
      datatable(nse(2,1,1),nse(2,2,2),nse(1,3,2),1), ...
      datatable(nse(2,1,2),nse(2,2,2),nse(1,3,2),1));
  fprintf('y     = %11.3f          y     = %11.3f\n', ...
      datatable(nse(2,1,1),nse(2,2,2),nse(1,3,2),2), ...
      datatable(nse(2,1,2),nse(2,2,2),nse(1,3,2),2));
  fprintf('z     = %11.3f          z     = %11.3f\n', ...
      datatable(nse(2,1,1),nse(2,2,2),nse(1,3,2),3), ...
      datatable(nse(2,1,2),nse(2,2,2),nse(1,3,2),3));
  fprintf('value = %11.3f          value = %11.3f\n', ...
      datatable(nse(2,1,1),nse(2,2,2),nse(1,3,2),4), ...
      datatable(nse(2,1,2),nse(2,2,2),nse(1,3,2),4));
  fprintf('\n');
 
  fprintf('Desired Values\n')
  fprintf('%14s %14s %14s \n','X - Want(1)','Y - Want(2)','Z - Want(3)')
  fprintf('%14.5f %14.5f %14.5f\n\n',want(1),want(2),want(3))

  fprintf('%14s %14s %14s %14s\n','fx1z1','fx2z1','fx1z2','fx2z2')
  fprintf('%14.5f %14.5f %14.5f %14.5f\n\n',fx1z1,fx2z1,fx1z2,fx2z2)

  fprintf('%14s %14s\n','fz1','fz2')
  fprintf('%14.5f %14.5f\n\n',fz1,fz2)

  fprintf('%14s\n','f')
  fprintf('%14.5f\n\n',f)
end

% returning the function value
interpval = f;





























