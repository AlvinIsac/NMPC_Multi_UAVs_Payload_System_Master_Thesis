close all; clear; clc;

%% Physical parameters      
L   = 1.6;              

%% UAV1 initial conditions
pN1     = -0.8;          
vN1     = 0.0;           
pE1     = 0.0;           
vE1     = 0.0;           
pD1     = -7;            
vD1     = 0.0;           

phi1    = deg2rad(0);    
p1      = 0;             
theta1  = deg2rad(0);    
q1      = 0;             
psi1    = deg2rad(-90);  
r1      = 0;             

%% UAV2 initial conditions
pN2     = 0.8;           
vN2     = 0.0;           
pE2     = 0.0;           
vE2     = 0.0;           
pD2     = -7;            
vD2     = 0.0;           

phi2    = deg2rad(0);    
p2      = 0;             
theta2  = deg2rad(0);    
q2      = 0;             
psi2    = deg2rad(-90);  
r2      = 0;             

%% Payload initial conditions
start_Np    = (pN1 + pN2) / 2;   
start_vNp   = 0.0;               
start_Ep    = 0.8;               
start_vEp   = 0.0;               
start_Dp    = pD1 + L - 0.3;     
start_vDp   = 0.0;               

%% initial state vector

x0 = [ pN1; vN1; pE1; vE1; pD1; vD1; phi1; p1; theta1; q1; psi1; r1; ... 
       pN2; vN2; pE2; vE2; pD2; vD2; phi2; p2; theta2; q2; psi2; r2; ... 
       start_Np; start_vNp; start_Ep; start_vEp; start_Dp; start_vDp ];  

%% Clean workspace - keep only x0 
clearvars -except x0 

% auto ode45 max step size 0.001
% mpc sampling time 0.05 control out 0.001