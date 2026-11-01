clc;
syms s

%% calculations for the spee bump parametrs 

h = 0.20; %m
w = 0.5; %m
angle = atan(h); 
T_sim = 6;

% Parameters for 5 m/s speed bump

v_mid = 5; 
h_mid = 0.2; 
w_mid = 0.8; 

angle_mid = atan(h_mid / w_mid); % New angle
slope_mid = tan(angle_mid) % New slope

% Updated ramp parameters for Simulink
start_time_up_mid = 3; 
start_time_down_mid = start_time_up_mid + w_mid/v_mid; 
initial_output_up_mid = 0;
initial_output_down_mid = h_mid;

%% parameters :
%F_a =150;
Br = 1450;
%Br = 0; %for the third case
Kr = 30000;
%Kr = 00; % for the second case 
Kk = 340000;
%Kk = 0;  %for the third case
mk = 48.3;
mkv = 1/48.3;
mc = 408;
mcv = 1/408;

%% SS matrices:
SI = [s 0 0 0;
    0 s 0 0;
    0 0 s 0;
    0 0 0 s];
A_orig = [0 1 0 0;
     (-Kk-Kr)/mk -Br/mk Kr/mk Br/mk;
      0 0 0 1;
      Kr/mc Br/mc -Kr/mc -Br/mc];

B_orig = [0 0;
      Kk/mk -1/mk;
      0 0;
      0 1/mc];
B1st = B_orig(:,1); 

C_orig = [0 0 1 0];

D_orig = [0 0];

Dsing =[0];

%% controlability of origin

%Mc1 = [B_orig A_orig*B_orig A_orig^2*B_orig A_orig^3*B_orig]

Mc_orig = ctrb(A_orig, B1st)

rank_Mc_orig = rank(Mc_orig)

if rank_Mc_orig == size(A_orig, 1)
    display('The system is controllable');
else
    disp ('The system is not or just partly controllable');
end

%% obsevability of origin

%Mo1 = [C_orig C_orig*A_orig; C_orig*A_orig^2; C_orig*A_orig^3 ]

Mo_orig=obsv(A_orig,C_orig)
rank_Mo_orig=rank(Mo_orig)

if rank_Mo_orig == size(A_orig, 1)
    display('The system is observable');
else
    disp ('The system is not or just partly observable');
end

%% calculation of the original  poles of the system 
original_poles = eig(A_orig)

% manual solve fpr the poles of the system 
Char_equation = det(SI-A_orig) ;
original_poles2 = double(solve(Char_equation == 0, s))

if original_poles == original_poles2
    display('your poles are correct');
else
    disp ('your poles are NOT correct');
end

% Desired poles (selected for no overshoot and fast settling time)
p1 = -50;
p2 = -50;
p3 = -5;
p4 = -5;
desired_poles = [p1 p2 p3 p4];

%% k for the original system and the controlled one 
K_orig = acker(A_orig,B1st,original_poles)

KT = acker(A_orig ,B1st ,desired_poles);

% calculation of the kt using manual akraman formula 

alpha = poly(desired_poles);  % [1 a3 a2 a1 a0]
a3 = alpha(2);
a2 = alpha(3);
a1 = alpha(4);
a0 = alpha(5);

alpha_A = A_orig^4 + a3*A_orig^3 + a2*A_orig^2 + a1*A_orig + a0*eye(4)

MCinv = inv(Mc_orig)

KT2 = [0 0 0 1] * MCinv * alpha_A 

if KT2 == KT
    display('Your Calculations are correct');
else
    disp ('Your Calculations are Not correct');
end    

%% calculation of the new A matrix 
A_C = A_orig - B1st * KT 


% Static error correction for the uncontrolled system
s = size(A_orig,1);  
Z = [zeros([1,s]) 1];  
N2 = inv([A_orig,B1st;C_orig,Dsing])*Z'  
Nx2 = N2(1:s)  
Nu2 = N2(1+s)
comp2= Nu2 + K_orig*Nx2

% Static error correction for the controlled system
s = size(A_C,1);  
Z = [zeros([1,s]) 1];  
N = inv([A_orig,B1st;C_orig,Dsing])*Z'  
Nx = N(1:s)  
Nu = N(1+s)
comp= Nu + KT*Nx



%% state observer calculations 

alpha3 = 210;
alpha2 = 15025;
alpha1 = 420000;
alpha0 = 4000000;

% Original system characteristic coefficients
a3 = 33.5746;
a2 = 7733.9849;
a1 = 25017.2533;
a0 = 517598.344;

% Compute difference terms (observer design)
l00 = alpha0 - a0;   % = 3482401.656
l10 = alpha1 - a1;   % = 394982.7467
l20 = alpha2 - a2 ; % = 7291.0151
l30 = alpha3 - a3 ;  % = 176.4254

% Form L0^T vector
L0_T = [l30 l20 l10 l00];

% Display results
disp('L0^T = ');
disp(L0_T);


observer_poles = [-80, -80, -25, -25]; % Select fast poles for observer

% Compute Observer Gain (Lc)

Lc = acker(A_orig', C_orig', observer_poles)' % Use transpose of A and C
disp('Observer Gain Matrix (Lc):');
disp(Lc);

% Validate Observer Dynamics
% Compute the new observer system matrix (A_obs)
A_obs = A_orig - Lc * C_orig;

% Display the modified system matrix
disp('Observer System Matrix (A_obs):');
disp(A_obs);