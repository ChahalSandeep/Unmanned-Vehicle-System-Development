%%%%%%%%%%%%%%% Track Vehicle Parameter %%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% Track Vehicle Parameter %%%%%%%%%%%%%%%%%%%%%%%%%
b = (92/70)*.31;%0.5842 ; % [m] Effective Platform Width = Diagonal length
             % Actual Width = 0.3556 m
rNominal = 0.052959; % [m] Nominal Wheel Radius

%Vmax = 0.43; %132/866.1417; % [m/s] Maximum speed of the vehicle
Vmax = 0.060; % @100%, moves 2.14 m in 4 sec 
% wMax = Vmax/rNominal; % [rad/s] Maximum angular speed of wheels % NOT USED

%%%%%%%%%%%%%%% Encoder Parameter %%%%%%%%%%%%%%%%%%%%%%%%%
eTick = 237;%236.8852;%900; % 866.1417-905.5118; % [ticks/m] number of ticks per 1 m of vehicle translation % from 22-23 [ticks/inch]
TsampleEncoder = 0.1; %1/100; % 0.1 [s] Encoder sample time
%%
%%%%%%%%%%%%%%% Duty Cycle -> Speed - Conversion %%%%%%%%%%%%%%%%%%%%%%%%%
deadband = 10;
dcArray = [-90  -deadband 0  deadband  90];
wArray  = [-Vmax, 0, 0,  0, Vmax]./rNominal;

%%%%%%%%%%%%%%%%%% Initial Conditions %%%%%%%%%%%%%%%%%%%%%%%%%%%
xIC = 0;
yIC = 0;
thetaIC = 0*(pi/180);

