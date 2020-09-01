function Ia=solar(Va,Suns,TaC)
%Ia=solar(0:0.01:25+25*.3,1,25);

% Datos del arreglo solar MSX-60
% Calcula la corriente dado el voltaje, iluminacion y la temperatura
% Ia = solar (Va,G,T) = vector de voltaje
% Ia,Va = vector de corriente y ovoltaje
% G = numero de Suns (1 Sun = 1000 W/mˆ2)
% T = Temp en grados Celcius
k = 1.38e-23; % Constante de Boltzman’s
q = 1.60e-19; % Carga de un electron
% Entre las siguientes constantes aqui, y el modelo sera
% calculado en base a ellas. Para 1000Watt/m^2
n=1.2; % Factor de calidad del diodo, factor
% n =2 para cristalino, <2 para amorfo
Vg = 1.12; % Voltaje de la banda, 1.12eV para xtal Si,
% 1.75 para Si amorfo.
Ns = 36; % Numero de celdas en serie (diodos)
T1 = 273 + 25;
Voc_T1 = 21.06 /Ns;
% Voltaje de circuito abierto por celda a temperatura T1
Isc_T1 = 3.3;%3.80;
% Corriente de cortocircuito de la celda a temperatura T1
T2 = 273 + 75;
Voc_T2 = 17.05 /Ns;
% Voltaje de circuito abierto por celda a temperatura T2
Isc_T2 = 3.4;%3.92;
% Corriente de cortocircuito de la celda a temperatura T2
TaK = 273 + TaC; % Temperatura de trabajo del arreglo
K0 = (Isc_T2 - Isc_T1)/(T2 - T1); % Ecuacion (4)
IL_T1 = Isc_T1 * Suns; % Ecuacion (3)
IL = IL_T1 + K0*(TaK - T1); % Ecuacion (2)
I0_T1=Isc_T1/(exp(q*Voc_T1/(n*k*T1))-1);
I0= I0_T1*(TaK/T1).^(3/n).*exp(-q*Vg/(n*k).*((1./TaK)-(1/T1)));
Xv = I0_T1*q/(n*k*T1) * exp(q*Voc_T1/(n*k*T1)); % Ecuacion (8)
dVdI_Voc = - 1.15/Ns / 2;
% dV/dI a Voc por celda desde la garficas del fabricante
% Rs resistencia serie por celda
Rs = - dVdI_Voc - 1/Xv; % Ecuacion (7)
A=1;
Vt_Ta = A * k * TaK / q; % = A * kT/q
Vc = Va/Ns;
Ia = zeros(size(Vc));
% Método de Newton
for j=1:5;
Ia = Ia - (IL - Ia - I0.*( exp((Vc+Ia.*Rs)./Vt_Ta) -1))./(-1 - (I0.*( exp((Vc+Ia.*Rs)./Vt_Ta) -1)).*Rs./Vt_Ta);

end
% 
% for i=1:length(Vc)
% Ia = Ia - (IL - Ia - I0*( exp((Vc+Ia*Rs)/Vt_Ta) -1))/(-1 - (I0*( exp((Vc+Ia*Rs)/Vt_Ta) -1))*Rs/Vt_Ta);
% plot(Ia)
% drawnow;
% pause(0.1);
% hold on
% end
plot(Va,Ia)
end
