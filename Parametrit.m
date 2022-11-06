
% DC-motor and aerodynamical parameters
load polynom %  THIS FILE INCLUDE THE POLYNOMIALs FITTED FOR THE EXPERIMENTAL DATA
%===========================================================================
%DATA NEEDED FOR THE SIMULATION MODEL
P_motor_m=polynom.MainSpeed; % POLYNOMIAL FROM VOLTAGE TO ANGULAR VELOCITY (main rotor)
F_aero_m=polynom.MainForce;  % POLYNOMIAL FROM ANGULAR VELOCITY TO FORCE(main rotor)
P_motor_t=polynom.TailSpeed; % POLYNOMIAL FROM VOLTAGE TO ANGULAR VELOCITY (tail rotor)
F_aero_t=polynom.TailForce;  % POLYNOMIAL FROM VOLTAGE ANGULAR VELOCITY TO FORCE(tail rotor)
Tm = 0.8921;                  % Motor time constant of main rotor
Tt = 0.3423;                  % Motor time constant of tail rotor
%==========================================================================
um=-18:0.1:18;               % VOLTAGE RANGE UNDER STUDY for main rotor
a_v_m = polyval(P_motor_m,um) ;
a_v_t = polyval(P_motor_t,um);

F_m = polyval(F_aero_m,a_v_m);
F_t = polyval(F_aero_t,a_v_t);

%Propose a linearized dynamics for the nonlinear curve. Use notations Kam
%Kat for the aerodynamics and Kum and Kut for the voltage/velocity dynamics
P_motor_m_prime = polyder(P_motor_m);
Kum = polyval(P_motor_m_prime,15) %15


P_motor_t_prime = polyder(P_motor_t);
Kut = polyval(P_motor_t_prime,0)

F_aero_m_prime = polyder(F_aero_m);
Kam = polyval(F_aero_m_prime,176) %175

F_aero_t_prime = polyder(F_aero_t);
Kat = polyval(F_aero_t_prime,0)
