

%  Parámetros Motor Diesel

K1=1.15; K2=1; K3=1; Tao1=0.5; Tao2=0.125; J=0.3; P=0.1;

H_AM = tf([K1*(K2*K3)],[Tao2 1],'IODelay', Tao1); % Dinámica Actuador - Motor
H_AM_d = c2d(H_AM,0.01,'tustin')

H_E = tf([J*P],[1 (J*P)]); % Dinámica Eje
H_E_d = c2d(H_E,0.01,'tustin')


