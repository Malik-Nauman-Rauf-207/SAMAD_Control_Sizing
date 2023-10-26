clc
clear
% Taking Takeoff and Landing Configuration
% Given Parameters (Input Parameters)
Max_TO_W = 95;                      %kg, Max Takeoff Weight
S_w = 1.958;                        %m^2, Wing Area
AR_w = 10.345;                      % Wing Aspect Raio 
S_h = 0.29*(cosd(30))^2;                       %m^2, Horizontal Tail Area
S_v = 0.29*(sind(30))^2;%0.089;
%m^2, Vertical Tail Area
Vs_knots = 40;                       %knots
%Vs_app_knots = 135;                  %knots
V = Vs_knots*0.514444;               %m/s^2, Cruise Velocity
Vapp = 1.3*V;                         %m/s^2, Approach Velocity
%V_app = Vs_app_knots*0.514444;       %m/s^2, Approach Velocity
CL_aw_deg = 0.095;                   %/deg,
CL_aw = CL_aw_deg*57.3;            %/rad, Wing Lift Coefficient
Ixx = 14.61463;                     %kgm^2, Moment of Inertia
IN = 60;                            %percentage of span, Inboard Aileron Location
OUT = 90;                           %percentage of span, Outboard Aileron Location
Cr_w = 0.60;                        %m, Wing Root Chord
lamda = 0.45;                       % Taper Ratio
rau = 1.225;                        %kg/m^3, Density at Sea Level (1.125 at 3000ft)

% Assuming Flaps can go from 10 percent to 40 percent oƒ span.
% From Table 1 in paper [1]	O. Al-Shamma, R. Ali, and H. S. Hasan, “Programmable Aileron sizing algorithm for use in preliminary aircraft design software,” J. Eng. Appl. Sci., vol. 13, no. 10, pp. 3458–3462, 2018, doi: 10.3923/jeasci.2018.3458.3462.
% As configuration is <60000 kg, and for 30 deg of bank angle time = 1.3s
t1 = 1.3;                           %sec, Historical time to execute 30 deg bank
A2W_chord = 0.20;                    %Ratio, Aileron to Wing Chord
Tau = 1.129*(A2W_chord^0.4044)-0.1772;     % Aileron Effectiveness Parameter


% Computations 
b_w = sqrt(AR_w*S_w);               %m, Wing Span
c_w = b_w/AR_w;                     %m, Wing Chord (MAC)                       
yi = (IN/100)*(b_w/2);              %m, Inboard Aileron location
yo = (OUT/100)*(b_w/2);             %m, Outboard Aileron location
Cl_dela = ((2*CL_aw*Tau*Cr_w)/(S_w*b_w))*(((yo^2)/2 + (2/3)*((lamda-1)/b_w)*(yo^3))-((yi^2)/2 + (2/3)*((lamda-1)/b_w)*(yi^3)));
% Above is the Aileron Control Power
Del_a = 10/57.3;                    %rad, Aileron Deflection Angle (MAX)          
Cl = Cl_dela*Del_a;                 % Roll Moment Coefficient
L_Del = 0.5*rau*(Vapp^2)*S_w*Cl*b_w;   %Nm, Rolling Moment due to Aileron
CDR = 1.1;                         % Rolling Drag Coefficient
%yDR = 0.50*(b_w/2);
yDR = (b_w/6)*((1+2*lamda)/(1+lamda));
Pss = sqrt((2*L_Del)/(rau*(S_w+S_h+S_v)*CDR*(yDR^3)));
PSS = Pss*57.3;
% Steady State Roll Rate Computed Above
Phi = (Ixx*log(Pss^2))/(rau*(yDR^3)*(S_w+S_h+S_v)*CDR);
PHY = Phi*57.3;
P_star = (Pss^2)/(2*Phi);
t2 = sqrt((2*(30/57.3))/P_star);





