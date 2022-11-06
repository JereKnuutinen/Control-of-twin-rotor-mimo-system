%---------------------------------------------------------
% continuous plant model
%
Ac = [-1/Tm 0 0 0 0 0;
    0 -1/Tt 0 0 0 0;
    0 0 0 0 1 0;
    0 0 0 0 0 1;
    l_m*Kam/J_v -J_tr/(J_v*Tt) -g*C/J_v 0 -k_v/J_v 0;
    -J_mr/(((E+F))*Tm) l_t*Kat/(E+F) 0 0 0 -k_h/(E+F)];

Bc = [Kum/Tm 0;
      0 Kut/Tt;
      0 0;
      0 0;
      0 J_tr*Kut/(J_v*Tt);
      J_mr*Kum/(((E+F))*Tm) 0];
% 
Hc = [1 0 0 0 0 0;
      0 1 0 0 0 0;
      0 0 1 0 0 0;
      0 0 0 1 0 0]

% Hc = [0 0 1 0 0 0;
%       0 0 0 1 0 0;
%       1 0 0 0 0 0;
%       0 1 0 0 0 0]


Dc= zeros(4,2);


% Create the LTI continuous model
%
sys = ss(Ac,Bc,Hc,Dc)
% step(10*sys,'x')

% Create the Discrete model with a sampling time of T=10 ms.
%
Ts = 10 * 10^(-3);
sysd = c2d(sys,Ts,'zoh');
[Phi,Gamma,Cd,Dd] = ssdata(sysd);

% step(10*sysd,'o')
%%
% Design the regulator by computing the LQR Gain matrix K
% R = diag([1/(18)^2 1/(18)^2])
R = 1/(18)^2*eye(2)
Q = diag([0.15 0.15 1/(0.2)^2 1/(0.2)^2 1 1 1/(0.4)^2 1/(0.4)^2]);
%Q = diag([0.2 0.2 1/(0.2)^2 1/(0.2)^2 0.1 0.1 1/(0.8)^2 1/(0.8)^2]);
%Q = diag([0.8 0.8 1/(0.02)^2 1/(0.02)^2 0.1 0.1 1/(1)^2 1/(1)^2]);
%Q = eye(8)

Hc2 = [0 0 1 0 0 0;
       0 0 0 1 0 0];
   
Phi_aug = [Phi zeros(6,size(Hc2,1));
             Hc2  eye(size(Hc2,1))];
Gamma_aug = [Gamma;zeros(size(Hc2,1),2)];
Cd_aug = [eye(6) zeros(6,size(Hc2,1))];

K = dlqr(Phi_aug,Gamma_aug,Q,R);
Ki = K(:,7:end)
K2 = K(:,1:end-2)

%  the Kalman filter gains
% Assume rms noise of 1% on each sensor channel
% Rv = 0.1^2 * eye(4);
% 
% % Rw
% Rw = 0.1*eye(2)
% 
% P = ss(Phi, [Gamma Gamma], Cd, [Dd Dd], Ts);
% [Observer, Ko, P] = kalman(P, Rw, Rv)

Vd = 0.1*eye(6)
Vn = 0.1^2*eye(4)
[Ko,p_est,E_estimator] = dlqe(Phi,eye(6),Cd,Vd,Vn)
%Ko = dlqr(Phi',Cd',Vd,Vn)'
sysKF = ss(Phi-Ko*Cd,[Gamma Ko],eye(6),0*[Gamma Ko],Ts)

% [Kf,P,E] = dlqe(Phi,Gamma,Cd,Rw,Rv)
% sysKF = ss(Phi-Kf*Cd,[Gamma Kf],eye(6),0*[Gamma Kf],Ts)
% pzmap(sysd)
% % figure
% % 
% hold on
% 
% pzmap(sysKF)
pole(sysKF)
pole(sysd)
%-----------------------------------------------------------
%  regulator and the closed-loop system
%
% lqg_reg = lqgreg(Observer,K2)
% feedin = [1,2];
% feedout = [1,2,3,4];
% Gcl = feedback(sysd,lqg_reg,feedin,feedout,+1)
% 
% %step(Gcl)
% % Compute and plot the initial condition response
% % Set x_1(0) = 1, all others to zero.
% 
% x0 = zeros(8,1);
% x0(3) = deg2rad(-37);
