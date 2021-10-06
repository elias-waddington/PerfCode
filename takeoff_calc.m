%% Import data from text file
% Script for importing data from the following text file:
%
%    filename: C:\Users\Elias.DESKTOP-57LHO0I\Box Sync\CHEETA UIUC Only\Notes\EGW\Takeoff\fig5_4_2jet.csv
%
% Auto-generated by MATLAB on 30-Jul-2020 13:48:29

%% Setup the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 2);

% Specify range and delimiter
opts.DataLines = [1, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["TakeoffParam", "RunwayLength"];
opts.VariableTypes = ["double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
tbl = readtable("fig5_4_2jet.csv", opts);

%% Convert to output type
TakeoffParam = tbl.TakeoffParam;
RunwayLength = tbl.RunwayLength;


%% Clear temporary variables
clear opts

TOparamIn = 148.431116;

TOFL = interp1(TakeoffParam,RunwayLength,TOparamIn);
disp(TOFL)
