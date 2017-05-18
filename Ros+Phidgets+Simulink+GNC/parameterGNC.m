% Navigation Parameters
% if we want the estimation parameters identical to actual parameters
VmaxDR = Vmax; 
rNominalDR = rNominal;
bDR = b;
eTickDR = eTick;
TsampleEncoderDR = TsampleEncoder;
TauEncoderDR = 0.1;
%%
L=0.5;%L<1
R=0.5;%R<1
%%%%%%%%%%%%%%%%%%% control parameter %%%%%%%%%%%%%%%%%%%%%%%%%%%
% 1:speed, 2:angle
KP1=10; %20;    % 1 is velocity controller
KP2=3;%10;  %7 ->   % 2 is angle controller
KI1=.5;%10;%0.3; %0.001; %0.001;%10;
KI2=0.001; %0.001; %0.001;%1; % 3 ->
KD1=0; %3; %0; %3; %0;%3;
KD2=0;% 3;%  %.5; %0; %0.5; %0;%0.5; %0.5 ->
Tsample = 0.1; %1/100; %1/10;   %sampling rate -> 0.01
Tmodel=Tsample;
Tau1 = 0.01;     %time constant of filter 1
Tau2 = 0.1;%0.1;       %time constant of filter 2

% wheel speed - DC conversion
%wArrayC  = [0,  VmaxDR/100, VmaxDR]./rNominalDR;
%dcArrayC = [0  30  90];


 wArrayC  = [-VmaxDR, -VmaxDR/100, 0,  VmaxDR/100, VmaxDR]./rNominalDR;
 dcArrayC = [-90  -10 0  10  90];
%%%%%%%%%%%%%%%%%%% guidance parameter %%%%%%%%%%%%%%%%%%%%%%%%%%%
rp1 =  0.3; % 1 -> [m] proximity circle to start slowing down
rp2 =  0.2;  % [m] radius of wayPoint proximity circle to switch to the next wayPoint
Vcom = 3*VmaxDR/4; % [m/s] used when constant speed is commanded

%%%%%%%%%%%%%%%%%% Way Points %%%%%%%%%%%%%%%%%%%%%%%%%%%
X_array  = [1  2  2 ]; 
Y_array  = [0  1  2 ]; 
%%%%%%%%%%%%%%%%%%% guidance parameter %%%%%%%%%%%%%%%%%%%%%%%%%%%
%oneLoop = 1 => stops at the last wayPoint
%oneLoop = 0 => continue cycling through wayPoints
oneloop =1;