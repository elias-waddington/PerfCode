% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SCRIPT:  'allloadin.m'
% VERSION: 2.0
% AUTHOR:  Jason M Merret
% O. DATE: 7-29-2008
% M. DATE: 06/07/2019 
% DEPT:    Preliminary Design - 0686
% DESC:    This is script reads in TAPE22 or TECPLOT formatted files and 
%            orders the matrices to an i,j,k format.  The script fills in 
%            values in the table when no values occur (9999999).
%
% NOTES:   A) There need to be a version of this that is written for quick
%            conversion to C or C++.  That should be simple by changing a
%            few lines of code.  This can be accomplished by avoiding the
%            use of cell arrays and using for loop instead of the MATLAB :
%            syntax.
%
%          B) TECPLOT file format is as such:
%             '# ...'
%             '# ...'
%             'VARIABLES = "X","Y","Z","FN","WF"'
%             'ZONE T = "xxxxx", I = #, J = #, K = #, DATAPACKING=POINT'
%              X(Z), Y(Z), Z, FN(X,Y,Z), WF(X,Y,Z) 
%
% INPUT:   'infilename' -  Input file name and Path (if necessary)
%          'dtl'        - Detail flag for dianostic output ('y' or 'n')
%
% OUTPUT:  Data Structures containing a FN table and a WF (Two different
%            structures are only returned for the TECPLOT format.  The 
%            TAPE22 format returns the same thing for FN and WF: USER 
%            BEWARE)
%          
%            'outFN.name'  - Tablename aka (input file name)'
%            'outFNformat' - Table format (1=TECPLOT, 22=TAPE22)
%            'outFN.head'  - Table Header 
%            'outFN.nhead' - Number of Header Lines
%            'outFN.nums'  - iX , iY, and iZ indicies
%            'outFN.data'  - table data fmtd as (i,j,k,1) - X(Z)
%                          - table data fmtd as (i,j,k,2) - Y(Z)
%                          - table data fmtd as (i,j,k,3) - Z
%                          - table data fmtd as (i,j,k,4) - f(X(Z),Y(Z),Z)
%
%            'outWF.name'  - Tablename aka (input file nam)'
%            'outWFformat' - Table format (1=TECPLOT, 22=TAPE22)
%            'outWF.head'  - Table Header 
%            'outWF.nhead' - Number of Header Lines
%            'outWF.nums'  - iX , iY, and iZ indicies
%            'outWF.data'  - table data fmtd as (i,j,k,1) - X(Z)
%                          - table data fmtd as (i,j,k,2) - Y(Z)
%                          - table data fmtd as (i,j,k,3) - Z
%                          - table data fmtd as (i,j,k,4) - f(X(Z),Y(Z),Z)
%
%            'tabformat'  - This is a numeric indicator for the table type
%
%            'tabletype'  - This is a string indicator for the table type
%
% REVISION HISTORY:
% REV:     DATE:           CONTACT:       DESCRIPTION:
% 2.0      06/07/2019      JMM            Updates for a 3rd output table
% (T22)
% 3.0      03/25/2020      JMM            Sets all to the same version #
%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [outFN, outWF, out03]= allloadin_V003(infilename,dtl)

%clear all;
%infilename = '4C_PP2.dat';
%dtl = 'n';


% Reading the first line of the file to determine the file type
infile = fopen(infilename);
inline = fgetl(infile);

insize = size(inline);

if (insize(2)>1)
  tabtype = inline(1:5);  
else    
  tabtype = inline(1:1);
end

% String comparisons for the file types a Tecplot 
compare1 = strcmp(tabtype,'VARIA');
compare2 = strcmp(tabtype,'varia');
compare3 = strcmp(tabtype(1),'#');

% setting the table format based on the string comparisons
if (compare1 == 1 || compare2== 1 || compare3==1) 
  tabformat = 1;
%  tabletype = 'TECPLOT';
else
  tabformat = 22;
%  tabletype = 'TAPE22';
end

% TECPLOT format readin
if (tabformat==1)
  line{1,:} = inline;
  test = inline(1);
  nhead = 1;

% Reading the remainder of the header in and storing the strings.  I had
% to read them in as character sting then hold them in a cell array and 
% then convert them to a char array 
  while (test=='#')
    nhead = nhead + 1;
    inline = fgetl(infile);
    line{nhead,:} = inline;
    test = inline(1);
  end
  line3 = fgetl(infile);
  nhead = nhead + 1;
  line{nhead,:}=line3;
  %line = char(line);

% This extracts the index values for the X, Y, and Z Directions

% Old Method of fixed width
%  ijkindexs = sscanf(line3,'%*30c %6d %*5c %6d %*5c %6d %*s');
%  ix = ijkindexs(1);
%  iy = ijkindexs(2);
%  iz = ijkindexs(3);

% New Method that allows for varying s
   line4 = line3(5:length(line3)); % Remone 'ZONE' from the line'
   line4 = upper(line4);
   IJKline1 = blanks(1);
   spacelist = isspace(line4);
   lenspacelist = length(spacelist);
   j=1;
   for i=1:lenspacelist 
     if (spacelist(1,i)==0)
%       IJKline1 = strcat(IJKline1,line4(i));
       IJKline1(j) = line4(i);
       j = j + 1;
     end    
   end
   IJKline2 = textscan(IJKline1,'%s','delimiter',',');
   IJKline3 = deblank((char(IJKline2{1})));   
   [numstr lenstr] = size(IJKline3); 
   for i=1:numstr
     testIeq=strcmp(IJKline3(i,1:2),'I=');
     if ((testIeq==1))
       nstart = i;
     end
   end 
  ix = str2double(IJKline3(nstart,3:lenstr));
  iy = str2double(IJKline3(nstart+1,3:lenstr));
  iz = str2double(IJKline3(nstart+2,3:lenstr));
  
  inline = fgetl(infile);
  %line{3,:} = inline;
  test = inline(1:3);
  while (test=='AUX')
    nhead = nhead + 1;
    line{nhead,:} = inline;
    inline = fgetl(infile);
    test = inline(1:3);
  end
  line = char(line);
  nhead;
% Rewinds the file and starts over.  Therefore the textscan read is 
% clean.
  frewind(infile);
  
% This reads the data into a cell array of 5 columns
  alldata2 = textscan(infile,['%f%f%f%f%f','%f%f%f%f%f','%f%f%f%f%f',...
    '%f%f%f%f%f','%f%f%f%f%f'], ...
    'headerlines',nhead,'emptyvalue',9999999);
  fclose(infile);
% converts the Cell array to a matrix  
  alldata3 = cell2mat(alldata2);

% Initializes the output matrices with the error value
  alldataFN = ones(ix+1,iy+1,iz+1,4)*9999999;
  alldataWF = ones(ix+1,iy+1,iz+1,4)*9999999;
  alldata03 = ones(ix+1,iy+1,iz+1,4)*9999999;
  
% Fills in the output matrices with the table data
  icount = 1;
  for k=1:iz
    for j=1:iy
      for i=1:ix
        alldataFN(i,j,k,1) = alldata3(icount,1);
        alldataFN(i,j,k,2) = alldata3(icount,2);
        alldataFN(i,j,k,3) = alldata3(icount,3);
        alldataFN(i,j,k,4) = alldata3(icount,4);
      
        alldataWF(i,j,k,1) = alldata3(icount,1);
        alldataWF(i,j,k,2) = alldata3(icount,2);
        alldataWF(i,j,k,3) = alldata3(icount,3);
        alldataWF(i,j,k,4) = alldata3(icount,5);
        
        alldata03(i,j,k,1) = alldata3(icount,1);
        alldata03(i,j,k,2) = alldata3(icount,2);
        alldata03(i,j,k,3) = alldata3(icount,3);
        alldata03(i,j,k,4) = alldata3(icount,6);
        icount = icount +1;  
      end
    end
  end

% Fills in the extra z dimension with the previous's X and Y index
% values.  This prevents an error when Z = 1 and allows the author to use
% the same interpolation without having to perform a special check
  icount = 1;
  %if (iz==1)
    for j=1:iy
      for i=1:ix
        alldataFN(i,j,iz+1,1) = alldata3(icount,1);
        alldataFN(i,j,iz+1,2) = alldata3(icount,2);   
        alldataWF(i,j,iz+1,1) = alldata3(icount,1);
        alldataWF(i,j,iz+1,2) = alldata3(icount,2);
        alldata03(i,j,iz+1,1) = alldata3(icount,1);
        alldata03(i,j,iz+1,2) = alldata3(icount,2);
        icount = icount +1;  
      end
    end  
  %end

% This prints out a detailed output to a specified file in the format that
% is used in for the interpolation scheme.  It prints out both the WF and
% FN data.
  if (dtl == 'y')
    writefile = fopen('tecreader.dat','w'); 
    for k=1:iz+1
      for j=1:iy
        for i=1:ix
          fprintf(writefile,'%11.3f %11.3f %11.3f %11.3f\n', ... 
            alldataFN(i,j,k,1),alldataFN(i,j,k,2),alldataFN(i,j,k,3), ... 
            alldataFN(i,j,k,4));
        end
      end
    end
    fprintf(writefile,'\n\n');
    for k=1:iz+1
      for j=1:iy
        for i=1:ix
          fprintf(writefile,'%11.3f %11.3f %11.3f %11.3f\n', ... 
            alldataWF(i,j,k,1),alldataWF(i,j,k,2),alldataWF(i,j,k,3), ... 
            alldataWF(i,j,k,4));
        end
      end
    end
    fprintf(writefile,'\n\n');
    for k=1:iz+1
      for j=1:iy
        for i=1:ix
          fprintf(writefile,'%11.3f %11.3f %11.3f %11.3f\n', ... 
            alldata03(i,j,k,1),alldata03(i,j,k,2),alldata03(i,j,k,3), ... 
            alldata03(i,j,k,4));
        end
      end
    end
    fclose(writefile);
  end
  
% Setting up the output
  nums(1) = ix;
  nums(2) = iy;
  nums(3) = iz;
  tableFN(1).name = infilename;
  tableFN(1).format = tabformat;
  tableFN(1).head = line;
  tableFN(1).nhead = nhead;
  tableFN(1).nums = nums;
  tableFN(1).data = alldataFN;
  outFN = tableFN;

  tableWF(1).name = infilename;
  tableWF(1).format = tabformat;
  tableWF(1).head = line;
  tableWF(1).nhead = nhead;
  tableWF(1).nums = nums;
  tableWF(1).data = alldataWF;
  outWF = tableWF;
  
  table30(1).name = infilename;
  table30(1).format = tabformat;
  table30(1).head = line;
  table30(1).nhead = nhead;
  table30(1).nums = nums;
  table30(1).data = alldata03;
  out03 = table30;


% TAPE22 format readin
elseif (tabformat==22)
        
  line1 = inline;
  line{1,:} = inline;
  ijkindexs = sscanf(line1,'%*10s %2d %2d %2d %*s');
  iy = ijkindexs(1);
  ix = ijkindexs(2);
  iz = ijkindexs(3);
   test = '*';
  nhead = 1;
  ntab = ceil((ix-1)/7);
  ntot = ntab*iy*iz;
  while test=='*' 
    inline = fgetl(infile);
    line2 = inline;
    test = line2(1);
    nhead = nhead + 1;
    line{nhead,:} = inline;
  end
  line{nhead} = '';
  nhead = nhead - 1;

  line = char(line);

% Rewinding the file and Obtaining the data matrix
% This data is read in as a CELL ARRAY with an array for
% each column of the data.  If there is an empty value in the
% matrix it automatically fills the missing values with 9999999.0
% The input file is then closed
  frewind(infile);
 alldata2 = textscan(infile,'%10f%10f%10f%10f%10f%10f%10f%10f', ...
   'headerlines',nhead,'emptyvalue',9999999);
  fclose(infile);


  %Intitializing a tranfer matrix
  alldata3 = ones(ntot,8)*9999999;

  % Converting the CELL ARRAY to a Matrix
  for i=1:8
    alldata3(:,i) = alldata2{1,i};
  end

% Inititalizing other transfer arrays and matrices
  alldata4 = ones(ix-1,iy-1,iz)*9999999.;
  alldata5 = ones(ix,iy,iz+1,4)*9999999;
  ixtab = zeros(ntab,2);
  indxx = zeros(ix-1,iz);
  indxy = zeros(iy-1,iz);
  indxz = zeros(iz);

% Creating a column array for the table wraps associated with the 
% Tape22 format.  The data is stored in columns 2-8 in each of the full 
% tables and 2-? on the partial table in the native.
  for i=1:(ntab-1)
    ixtab(i,1) = 2;
    ixtab(i,2) = 8;
  end
  ixtab(ntab,1) = 2;
  ixtab(ntab,2) = ix-(ntab-1)*7; 

% Transfering the data to the index arrays filled with the X, Y , and Z
% data and a data matrix with f(X(Z),Y(Z),Z)
  for i=1:iz
    ides = 1;
    indxz(i) = alldata3(1+ntab*iy*(i-1),1);
    for j=1:ntab
      for k=ixtab(j,1):ixtab(j,2) 
        indxx(ides,i) = alldata3(1+iy*(j-1),k);
        for l=2:iy
          indxy(l-1,i) = alldata3(l+iy*(i-1)*ntab,1);    
          alldata4(ides,l-1,i) =  alldata3((l+iy*ntab*(i-1)+(j-1)*iy),k); 
        end 
        ides = ides + 1;     
      end
    end
  end

% This takes the index arrays and data matrix and combines it all into
% one large matrix = f(ix,iy,iz,1:4) where
%                    f(ix,iy,iz,1) = X(Z)
%                    f(ix,iy,iz,2) = Y(Z)
%                    f(ix,iy,iz,3) = Z
%                    f(ix,iy,iz,4) = f(X(Z),Y(Z),Z)
% You can also print out the unwrapped Tape22 format if you want by 
% uncommenting the fprintf statements which will print out to the screen
  for k=1:iz
    for j=1:iy-1
      for i=1:ix-1
        alldata5(i,j,k,1) = indxx(i,k);
        alldata5(i,j,k,2) = indxy(j,k);
        alldata5(i,j,k,3) = indxz(k);
        alldata5(i,j,k,4) = alldata4(i,j,k);       
      end
%    fprintf('\n',1)
    end
%  fprintf('\n\n',1)
  end

% This part copies the X and Y indicies to the iZ+1 index (From the iZ 
% index) so the interpolation program will not crash when iZ=1.  It is 
% necessary because the X and Y indicies are a function of Z and will be 
% problematic if cannot find value in the search.
  %if (iz==1)
    for j=1:iy-1
      for i=1:ix-1
        alldata5(i,j,iz+1,1) = indxx(i,iz);
        alldata5(i,j,iz+1,2) = indxy(j,iz);      
      end
    end    
  %end    
  
% Writing out the matrix to an output file for analysis and diagnostics
  if (dtl == 'y')
    writefile = fopen('T22reader.dat','w'); 
    for k=1:iz+1
      for j=1:iy-1
        for i=1:ix-1
        fprintf(writefile,'%11.3f %11.3f %11.3f %11.3f\n', ... 
          alldata5(i,j,k,1),alldata5(i,j,k,2),alldata5(i,j,k,3), ... 
          alldata5(i,j,k,4));
        end
      end
    end
    fclose(writefile);
  end

% Setting up the output
  nums(1) = ix-1;
  nums(2) = iy-1;
  nums(3) = iz;
  table(1).name = infilename;
  table(1).format = tabformat;
  table(1).head = line;
  table(1).nhead = nhead;
  table(1).nums = nums;
  table(1).data = alldata5;
  outFN = table;
  outWF = table;
end

end