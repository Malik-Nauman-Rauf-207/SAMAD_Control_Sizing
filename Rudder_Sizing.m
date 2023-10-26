clc
clear
% Given Parameters
M_TO_m = 95;                %kg, Max Takeoff Mass
M_TO_W = M_TO_m*9.81;       %N, Max Takeoff Weight
Vs = 40*0.514444;           %m/s, Stall Velocity
Vc = 90*0.514444;           %m/s, Cruise Velocity
Power = 19*745.7;           %Watt, Shaft Power
Eff = 0.8;                  % Efficiency of Propeller
V_max = 120*0.514444;       %m/s, Max Velocity 
T_max = (Eff*Power)/V_max;  %N, Max Thrust
lf = 2.88;                  %m, Fuselage Length
Df = 0.27;                   %m, Fuselage Diameter
Cyo = 0;                    % Side-Force Coefficient
%CDo = 0.026;               % Zero-Lift Drag Coefficient
%CLo = 0.1;                 % Zero-Lift Lift Coefficient
Cno = 0;                    % Yaw Moment Coefficient
S_w = 1.958;                %m^2, Area of Wing
AR_w = 10.345;              % Aspect Ratio of Wing
b_w = sqrt(AR_w*S_w);       %m, Span of Wing
c_w = b_w/AR_w;             %m, Mean Chord of Wing
Lamda_w = 0.7;              % Taper Ratio of Wing
Inc_w = 0;                  %rad, Incidence angle of Wing
CLalpha_w = 0.095*57.3;     %1/rad, Lift Slope of Wing
S_v = 0.29*(sind(30))^2;                 %m^2, Vertical Tail Area
b_v = 0.5*sind(30);                 %m, Vertical Tail Span
AR_v = (b_v^2)/S_v;         % Aspect Ratio of Horizontal Tail
Lamda_v = 0.7;              % Horizontal Tail Taper Ratio
CLalpha_v = 0.10198*57.3;    %1/rad, Lift Slope of Vertical Tail
br_bv = 1.0;                % Ratio of Rudder span to Vertical Tail Span
cr_cv = 0.35;                % Ratio of Rudder chord to Vertical Tail chord
VV = 0.06;                  % Vertical Tail volume ratio (Assumed)
eta_v = 0.95;               % Dynamic Pressure Ratio for Vertical Tail (Assumed)
dsigma_dbeta = 0;           % Variation of crab angle with sideslip
lv = 1.0895;                %m, Vertical Tail moment arm
x_cg = 1.383;               %m, CG of the aircraft 0.1088
C_mac = -0.0037;            % Pitch Moment about Aerodynamic Center
rau = 1.225;                %kg/m^3, Density at Sea-Level
Iyy = 52.93;                %kgm^2, Moment of Inertia about Y-axis
Kf1 = 0.7;
Kf2 = 1.35;                 % Constants associated with tail moment and lift coefficients
%% Computation
V_cross = Vs*0.4;           %m/s, crosswing speed taking to be 40% of stall speed
% Configuration is not conventional so assuming it to be non-spinable
V_app = Vs*1.1;             %m/s, Approach speed taken to be 110% of the stall speed
V_total = sqrt((V_cross^2)+(V_app^2));  %m/s, total speed
Xf = 1.4;                   %m, Fuselage Approximate Center (Just taking as half as starting point)
Xv = 0.1715+2.4;            %m, Geometric Center of Tail from origin (using centroid of trapezoid relation)
S_f = 0.748;                %m^2, projected side area of the fuselage (Meesum)
S_s = 1.02*(S_f+S_v);       %m^2, Projected side area
Xa = ((lf*Df*Xf)+(S_v*Xv))/(lf*Df+S_v);     %m, Geometric Center of the aircraft
dc = Xa-x_cg;               %m, distance between aircraft center and CG
Cdy = 0.6;                  % Side Force Drag Coefficient (Assumed)
F_w = 0.5*rau*(V_cross^2)*S_s*Cdy;      %N, Side Force due to cross wind
Beta = atan(V_cross/V_app); %rad, Sideslip angle
Cn_beta = Kf2*CLalpha_v*(1-dsigma_dbeta)*eta_v*VV;  % Directional Moment Coefficient
Cy_beta = -Kf1*CLalpha_v*(1-dsigma_dbeta)*eta_v*(S_v/S_w);  % Directional Force Coefficient
Tau_r = 1.129*(cr_cv^0.4044)-0.1772;    % Rudder Effectiveness
Cy_delr = CLalpha_v*eta_v*Tau_r*br_bv*(S_v/S_w);
Cn_delr = -CLalpha_v*VV*eta_v*Tau_r*br_bv;        % Control Derivatives
[Del_r,Sigma] = Del_Beta(rau,V_cross,S_s,Cdy,V_total,S_w,b_w,Cyo,Cy_beta,Beta,Cy_delr,Cno,Cn_beta,Cn_delr,F_w,dc);
if (Del_r*57.3)<32.5
    disp("Rudder Sizing is satisfactory")
end



