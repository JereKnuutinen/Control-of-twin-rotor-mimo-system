
g = 9.81;

load polynom

P_motor_m=polynom.MainSpeed; % POLYNOMIAL FROM VOLTAGE TO ANGULAR VELOCITY (main rotor)
F_aero_m=polynom.MainForce;  % POLYNOMIAL FROM ANGULAR VELOCITY TO FORCE(main rotor)
P_motor_t=polynom.TailSpeed; % POLYNOMIAL FROM VOLTAGE TO ANGULAR VELOCITY (tail rotor)
F_aero_t=polynom.TailForce;  % POLYNOMIAL FROM VOLTAGE ANGULAR VELOCITY TO FORCE(tail rotor)
Tm = 0.8921;                  % Motor time constant of main rotor
Tt = 0.3423;                  % Motor time constant of tail rotor

%% System parameters
% Component masses
m_m = 0.0145;
m_t = 0.0155;
m_b = 0.022;
m_tr = 0.206;
m_cb = 0.068;
m_ts = 0.165;
m_ms = 0.225;
m_mr = 0.228;
% Dimensions
l_m = 0.24;
l_t = 0.25;
l_b = 0.26;
l_cb = 0.26;
r_ts = 0.1;
r_ms = 0.155;

%% horizontal
A = (m_t/2+m_tr+m_ts)*l_t;
B = (m_m/2+m_mr+m_ms)*l_m;
C = (m_b/2*l_b+m_cb*l_cb);
E = (m_m/3+m_mr+m_ms)*l_m^2+(m_t/3+m_tr+m_ts)*l_t^2;
F = m_ms*r_ms^2+m_ts/2*r_ts^2;
D = (m_b/3*l_b^2+m_cb*l_cb^2);

%==========================================================================
%INERTIAT VERTICAL
%==========================================================================
Jv1=m_tr*l_t^2;
Jv2=m_cb*l_cb^2;
Jv3=m_mr*l_m^2;
Jv4=m_t*l_t^2/3;
Jv5=m_m*l_m^2/3;
Jv6=m_b*l_b^2/3;
Jv7=m_ms*(r_ms^2/2+l_m^2);
Jv8=m_ts*(r_ts^2+l_t^2);
J_v=Jv1+Jv2+Jv3+Jv4+Jv5+Jv6+Jv7+Jv8;  


%% vertical
J_tr = 1.654e-5;
J_mr = 2.65e-5;
 
k_h = 0.0095;
k_v = 0.00545;

%% DC- motor time constants
Tm = 0.8921;
Tt = 0.3423;