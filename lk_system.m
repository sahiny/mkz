function [sys,x0,str,ts] = lk_system2(t,x,u,flag,init)

switch flag

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0
    [sys,x0,str,ts]=mdlInitializeSizes(init);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1
    sys=mdlDerivatives(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%
  % Unhandled flags %
  %%%%%%%%%%%%%%%%%%%
  case { 2, 4, 9 }
    sys = [];

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end
% end csfunc

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes(init)

sizes = simsizes;
sizes.NumContStates  = 4;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 4;
sizes.NumInputs      = 3;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
x0  = init;
str = [];
ts  = [0 0];

% end mdlInitializeSizes
%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(~,x,u)
  M = 1800;
  lf = 1.2;
  lr = 1.65;
  Caf = 140000/2;
  Car = 120000/2;
  Iz = 3270;

  mu = u(1);
  r_d = u(2);
  delta_f = u(3);

  A_lk = [0, 1, mu, 0; 
      0, -(Caf+Car)/M/mu, 0, ((lr*Car-lf*Caf)/M/mu - mu); 
      0, 0, 0, 1;
      0, (lr*Car-lf*Caf)/Iz/mu,  0, -(lf^2 * Caf + lr^2 * Car)/Iz/mu];
  B_lk = [0; Caf/M; 0; lf*Caf/Iz];
  E_lk = [0; 0; -1; 0];  
  
  sys = A_lk * x + B_lk * delta_f + E_lk * r_d;
  sys
% end mdlDerivatives
%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(~,x,~)

sys = x;

% end mdlOutputs
