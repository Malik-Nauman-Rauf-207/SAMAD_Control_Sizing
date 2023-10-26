clear
clc
% Given Parameters
M_TO_m = 95;                %kg, Max Takeoff Mass
M_TO_W = M_TO_m*9.81;       %N, Max Takeoff Weight
Vs = 40*0.514444;           %m/s, Stall Velocity
Vc = 90*0.514444;           %m/s, Cruise Velocity
Power = 19*745.7;           %Watt, Shaft Power
Eff = 0.8;                  % Efficiency of Propeller
V_max = 120*0.514444;       %m/s, Max Velocity 
T_max = (Eff*Power)/V_max;  %N, Max Thrust
Lf = 2.88;                  %m, Fuselage Length
CDo = 0.026;                % Zero-Lift Drag Coefficient
CLo = 0.1;                  % Zero-Lift Lift Coefficient
S_w = 1.958;                %m^2, Area of Wing
AR_w = 10.345;              % Aspect Ratio of Wing
b_w = sqrt(AR_w*S_w);       %m, Span of Wing
c_w = b_w/AR_w;             %m, Mean Chord of Wing
Lamda_w = 0.7;              % Taper Ratio of Wing
Inc_w = 0;                  %rad, Incidence angle of Wing
CLalpha_w = 0.095*57.3;     %1/rad, Lift Slope of Wing
S_h = 0.29*(cosd(30))^2;                 %m^2, Horizontal Tail Area
S_v = 0.29*(sind(30))^2;                 %m^2, Vertical Tail Area
b_h = 0.5*cosd(30);                  %m, Horizontal Tail Span
AR_h = (b_h^2)/S_h;         % Aspect Ratio of Horizontal Tail
Lamda_h = 0.7;              % Horizontal Tail Taper Ratio
CLalpha_h = 0.10198*57.3;    %1/rad, Lift Slope of Horizontal Tail
Inc_h = -5/57.3;            %rad, Horizontal Tail Incidence Angle
CL_flap = 0;                % Flap Lift Coefficient
C_mac = -0.0037;            % Pitch Moment about Aerodynamic Center
rau = 1.225;                %kg/m^3, Density at Sea-Level
Iyy = 52.93;                %kgm^2, Moment of Inertia about Y-axis
%% Computation
% Step1
be_bh = 1.0;                % Ratio of Elevator Span to Tail Span
TO_Pitch_Acc = 20;          %deg/s^2, Takeoff Pitch Acceleration
% This Above value taken from Table 1 in O. Al-Shamma, R. Ali, and H. S. Hasan, “An instructive algorithm for aircraft elevator sizing to be used in preliminary aircraft design software,” J. Appl. Eng. Sci., vol. 15, no. 4, pp. 489–494, 2017, doi: 10.5937/jaes15-14829.
%Step2
V_TO = 1.1*Vs;              %m/s, Takeoff Velocity taken as 110% stall speed
D_TO = 0.5*rau*(V_TO^2)*CDo*S_w;    %N, Drag
CL_TO = CLo;
L_wf = 0.5*rau*(V_TO^2)*CL_TO*S_w;  %N, Wing/Fuselage Lift Force
%F_f = (0.04*(M_TO_W) + 0.04*(M_TO_W-L_wf))/2;   %Friction force taking as an approximate avg.
%Acc = (T_max-D_TO-F_f)/M_TO_m;   
Acc = (T_max-D_TO)/M_TO_m;          %m/s^2, Linear Acceleration


% -0.01896 is the coefficient of pitching moment without tail at 10% margin
Cm_No_Tail = -0.01896;
%Cm_No_Tail = -0.02883;      % 20% Margin
M_No_Tail = 0.5*rau*(V_TO^2)*Cm_No_Tail*S_w*c_w;
L_h = (M_No_Tail-Iyy*(TO_Pitch_Acc/57.3))/(1.0895); % Lift of Tail
% In above the denominator is the horizontal tail moment arm
CL_h = (2*L_h)/(rau*(V_TO^2)*S_h);
%CL_h = -0.5008;
Epsilon0 = (2*CLo)/(pi*AR_w);
dEpsilon = (2*CLalpha_w)/(pi*AR_w);
alpha_w = 12/57.3;           %rad, wing angle of attack
Epsilon = Epsilon0 + dEpsilon*alpha_w;      %rad, Downwash angle
alpha_h = alpha_w + Inc_h - 0.1*Epsilon;        %rad, AOA of the tail
Del_e = -22/57.3;        %rad, Deflection angle limits of Elevator
Tau_e = (CL_h-CLalpha_h*alpha_h)/(CLalpha_h*Del_e);     %Elevator Effectiveness
%Ce_Ch = 0.17;       % Ratio of Elevator Chord to Tail Chord
Se_Sh = CS2S(Tau_e);
Ce_Ch = Se_Sh/be_bh;
Del_alpha0_e = -1.15*Ce_Ch*Del_e;       %rad, change parameter
%CL_h_CFD = input("Enter the value of lift coefficient gained via CFD: -0.124 ");
CL_h_LLT = LiftingLine(S_h,AR_h,Lamda_h,-0.001,Inc_h*57.3,CLalpha_h,0);
eta_h = 1.0; %Assuming tail is out of the wake of the wing
VH = 0.4;   % Horitzontal Tail Picking these values from literature
C_m_DelE = -CLalpha_h*eta_h*VH*be_bh*Tau_e;
C_L_DelE = CLalpha_h*eta_h*(S_h/S_w)*be_bh*Tau_e;

