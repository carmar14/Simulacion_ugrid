% este script es ejecutado al momento de ejecutar el modelo de simulink de
% la microgrid de esta carpeta.
% En la seccion de configuracion de simulink, En la pestaña de callBacks se
% configuran los scripts que se desean ejectar al incio, durante o al
% final de la simulacion.
clear
close all
clc
% se cargan las matrices dinamicas del MPC DMC.
load('DMC_matrices');
load('FitData')

% se carga la libreria de true time.
run('truetime-2.0\init_truetime');


%  Parámetros Motor Diesel

K1=1.15; K2=1; K3=1; Tao1=0.5; Tao2=0.125; J=0.3; P=0.1;

H_AM = tf([K1*(K2*K3)],[Tao2 1],'IODelay', Tao1); % Dinámica Actuador - Motor
H_AM_d = c2d(H_AM,0.01,'tustin')

H_E = tf([J*P],[1 (J*P)]); % Dinámica Eje
H_E_d = c2d(H_E,0.01,'tustin')

%MPC BIO
% 
% C1=22e-6;
% C2=C1;
% C3=610e-6;
% C6=2000e-6;
% C=C1+C2+C3+C6;
% R1=0.1;
% R2=R1;
% R3=1e-3;
% R4=10;
% R5=10;
% L1=40e-3;
% L2=30e-3;
% L3=5e-3;
% L4=5.2;
% Ab= [-R1/L1 0 0 0 -1/L1
%      0 -R2/L2 0 0 -1/L2
%      0 0 -R3/L3 0 -1/L3
%      0 0 0 -R5/L4 1/L4
%      1/C 1/C 1/C -1/C -1/R4];
% Bb = [1/L1 0 0 
%      0 1/L2 0
%      0 0 1/L3
%      0 0 0
%      0 0 0];
% Cb = [1 0 0 0 0
%       0 0 0 0 1];
% 
% Db = zeros(2,3);

% load('FitData.mat');
load('AyC.mat');
%tf1.Denominator{2, 1}(2)
% A=[0 1 zeros(1,10)
%    0 0 1 zeros(1,9)
%    -tf1.Denominator{1,1}(4) -tf1.Denominator{1,1}(3) -tf1.Denominator{1,1}(2) zeros(1,9)
%    zeros(1,4) 1 zeros(1,7)
%    zeros(1,5) 1 zeros(1,6)
%    zeros(1,3) -tf1.Denominator{1,2}(4) -tf1.Denominator{1,2}(3) -tf1.Denominator{1,2}(2) zeros(1,6)
%    zeros(1,7) 1 zeros(1,4)
%    zeros(1,8) 1 zeros(1,3)
%    zeros(1,6) -tf1.Denominator{2,1}(4) -tf1.Denominator{2,1}(3) -tf1.Denominator{2,1}(2) zeros(1,3)
%    zeros(1,10) 1 0
%    zeros(1,11) 1
%    zeros(1,9) -tf1.Denominator{2,2}(4) -tf1.Denominator{2,2}(3) -tf1.Denominator{2,2}(2)
%    ];
B=[0 0
   0 0
   1 0
   0 0
   0 0
   0 1
   0 0
   0 0
   1 0
   0 0
   0 0
   0 1];
% C=[tf1.Numerator{1,1}(3) tf1.Numerator{1,1}(2) tf1.Numerator{1,1}(1) tf1.Numerator{1,2}(3) tf1.Numerator{1,2}(2) tf1.Numerator{1,2}(1) zeros(1,6)
%    zeros(1,6) tf1.Numerator{2,1}(3) tf1.Numerator{2,1}(2) tf1.Numerator{2,1}(1) tf1.Numerator{2,2}(3) tf1.Numerator{2,2}(2) tf1.Numerator{2,2}(1)];
D=zeros(2,2)
sis=ss(A,B,C,D);

CSTR = sis;%ss(Ab,Bb,Cb,Db);
CSTR.InputName = {'Pin', 'Qin'};
CSTR.OutputName = {'Po','Qo'};
% CSTR.StateName = {'Ibio', 'Idie', 'Ipv', 'I5', 'Vout'};
CSTR

%  Design MPC Controller

%   MPCOBJ = mpc(PLANT,TS,P,M) specifies the control horizon, M. M is
%   either an integer (<= P) or a vector of blocking factors such that
%   sum(M) <= P.
%   MODELS.Plant = plant model (LTI or IDMODEL)
%         .Disturbance = model describing the input disturbances.
%                           This model is assumed to be driven by unit
%                          variance white noise.
%            .Noise = model describing the plant output measurement noise.
%                           This model is assumed to be driven by unit
%                          variance white noise.
%            .Nominal = structure specifying the plant nominal conditions
%                             (e.g., at which the plant was linearized).

% Create the controller object with sampling period, prediction and control horizons:
plant=CSTR;
Ts = 1e-1;
p = 100;
m = 100;
% mpcobj = mpc(plant, Ts, p, m);

% Specify actuator saturation limits as MV constraints.
% mpcobj.MV = struct('Min',{500;500},'Max',{3500;3500},'RateMin',{-100;-100});
% Simulate Using Simulink®

% To run this example, Simulink® is required.
% if ~mpcchecktoolboxinstalled('simulink')
%     disp('Simulink(R) is required to run this example.')
%     return
% end

% n11=tf1.Numerator{1,1};
% n12=tf1.Numerator{1,2};
% n21=tf1.Numerator{2,1};
% n22=tf1.Numerator{2,2};
% d11=tf1.Denominator{1,1};
% d12=tf1.Denominator{1,2};
% d21=tf1.Denominator{2,1};
% d22=tf1.Denominator{2,2};
% 
% N=[n11 n12; n21 n22];
% D=[d11 d12; d21 d22];
% [A1 B1 C1 D1]=tf2ss([n11; n12],[d11])
% [A2 B2 C2 D2]=tf2ss([n21; n22],[d11])
% [A3 B3 C3 D3]=tf2ss([n11; n12],[d12])
% [A4 B4 C4 D4]=tf2ss([n21; n22],[d12])
% [A5 B5 C5 D5]=tf2ss([n11; n12],[d21])
% [A6 B6 C6 D6]=tf2ss([n21; n22],[d21])
% [A7 B7 C7 D7]=tf2ss([n11; n12],[d22])
% [A8 B8 C8 D8]=tf2ss([n21; n22],[d22])


%Parametros 1:Biomasa 2:Diesel 3:VSI
R1=0.1;
R2=0.1;
R3=1e-3;
C1=22e-6;
C2=22e-6;
C3=610e-6;
Cl=2000e-6;
C=C1+C2+C3+Cl;
L1=40e-3;
L2=5e-3;
L3=30e-3;
L4=5.2;
R4=10;
R5=10;


%Tabla 1
cvsi=200/sqrt(2);
carga=cvsi*20;
q=0:0.001:carga*1.2;
soc=100-100*q/carga;

% se cargan las matrices dinamicas del MPC DMC.
load('DMC_matrices2');

% se carga la libreria de true time.
%run('truetime-2.0\init_truetime');

load('modeloElectrico');
load('modeloDiesel_dotros')

ssCl=[1 0 0 0 0
      0 1 0 0 0
      0 0 1 0 0
      0 0 0 0 1];
sys=ss(ssAl,ssBl,ssCl,[]);
tm=1e-4;%1e-4
sysd=c2d(sys,tm);
Ad=sysd.A;
Bd=sysd.B;
Cd=sysd.C;
po=10*[-1.92*10 -1.92*10.1 -1.92*10.3 -1.92*10.2 -1.92*10.4];
pod=exp(po*tm);
Ld=place(Ad',Cd',pod);

% step(sys)
% hold on
% step(sysd)



%Matrices para el diseño del observador de Luenberger
% Ad=ssAl;
% Bd=ssBl;
% Cd=ssCl;
% tm=1e-4;
% Ld=L';
load('datos_deteccion.mat');
% load('norm_with_attack.mat');
% load('datos_in_out.mat');
load('datos_in_out_good_well.mat')   %ATAQUES
% load('datos_in_out_good.mat')  %NORMALES

%Entradas
u1=u_fs(:,1);
u2=u_fs(:,2);
u3=u_fs(:,3);
%Salidas
I1=y_ma(:,1);
I2=y_ma(:,2);
I3=y_ma(:,3);
V_load=y_ma(:,4);

tiempo=0:tm:(length(det)-1)*tm;
tiempo=tiempo';

%UIO1 para el ataque del sensor 1 eliminamos esa fila de la matriz C

Fd1=[1e-4;1.0;1.0e-4;1.0e-4;1.0e-4];
% Fd1=[1.0e-4;1;1;1;1];
% C1=Cd(2:end,:);
% C1=Cd(1,:);
C1=Cd(2,:);
Fd2=[1.0e-4;1e-4;1;1.0e-4;1.0e-4];
% Fd2=[1; 1.0e-4;1;1;1];
% C2=[ Cd(1,:);Cd(3:end,:)];
% C2=Cd(2,:);
C2=Cd(3,:);
Fd3=[1.0e-4;1.0e-4;1e-4;1.0e-4;1];
% Fd3=[1; 1;1.0e-4;1;1];
% C3=[Cd(1:2,:);Cd(end,:)];
% C3=Cd(3,:);
C3=Cd(4,:);
% Fd4=[1.0e-4;1.0e-4;1.0e-4;1.0e-4;1];

% Fd=eye(4);
%Otra posibilidad
% Fd1=[1.0e-4;1;1;1;1];
% C1=Cd(2:end,:);   %Sin la entrada de de i1
% Fd2=[1;1.0e-4;1;1;1];
% C2=[ Cd(1,:);Cd(3:end,:)]; %Sin la entrada de de i2
% Fd3=[1;1;1.0e-4;1;1];
% C3=[Cd(1:2,:);Cd(end,:)]; %Sin la entrada de de i3
% Fd4=[1;1;1;1;1e-4];
% C4=Cd(1,:);
% C4=Cd(1:end-1,:); %Sin la entrada de de tension en la carga


%UIO2 para el ataque del sensor 2 eliminamos esa fila de la matriz C
Fd4=[1;1e-4;1e-4;1e-4;1e-4];
C4=Cd(1,:);
Fd5=[1e-4;1e-4;1;1e-4;1e-4];
C5=Cd(3,:);
Fd6=[1e-4;1e-4;1e-4;1e-4;1];
C6=Cd(4,:);

%UIO3 para el ataque del sensor 3 eliminamos esa fila de la matriz C
Fd7=[1;1e-4;1e-4;1e-4;1e-4];
C7=Cd(1,:);
Fd8=[1e-4;1;1e-4;1e-4;1e-4];
C8=Cd(2,:);
Fd9=[1e-4;1e-4;1e-4;1e-4;1];
C9=Cd(4,:);

%UIO4 para el ataque del sensor 4 eliminamos esa fila de la matriz C
Fd10=[1;1e-4;1e-4;1e-4;1e-4];
C10=Cd(1,:);
Fd11=[1e-4;1;1e-4;1e-4;1e-4];
C11=Cd(2,:);
Fd12=[1e-4;1e-4;1;1e-4;1e-4];
C12=Cd(3,:);

% UIO para medida 1
[F1,T1,K1U,H1]=uio_linear (Ad,Bd,C1,Fd1);
[F2,T2,K2U,H2]=uio_linear (Ad,Bd,C2,Fd2);
[F3,T3,K3U,H3]=uio_linear (Ad,Bd,C3,Fd3);
% UIO para medida 2
[F4,T4,K4U,H4]=uio_linear (Ad,Bd,C4,Fd4);
[F5,T5,K5U,H5]=uio_linear (Ad,Bd,C5,Fd5);
[F6,T6,K6U,H6]=uio_linear (Ad,Bd,C6,Fd6);
% UIO para medida 3
[F7,T7,K7U,H7]=uio_linear (Ad,Bd,C7,Fd7);
[F8,T8,K8U,H8]=uio_linear (Ad,Bd,C8,Fd8);
[F9,T9,K9U,H9]=uio_linear (Ad,Bd,C9,Fd9);
% UIO para medida 4
[F10,T10,K10U,H10]=uio_linear (Ad,Bd,C10,Fd10);
[F11,T11,K11U,H11]=uio_linear (Ad,Bd,C11,Fd11);
[F12,T12,K12U,H12]=uio_linear (Ad,Bd,C12,Fd12);

clc