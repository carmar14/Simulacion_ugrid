clear
clc
close all

load('dr.mat')
dr1(1,:)=dr.signals.values(:,1,:);
tiempo_s=dr.time;
tiempo_s=tiempo_s';

a=1;
% r=[];
dr=dr1;
for i=1:length(tiempo_s)
    if tiempo_s(i)<=0.33
        th=1.5e3;
        
    end
    if tiempo_s(i)<=0.33
        th=1.5e4;
        %     v=2;
    else
        th=2e4;  %2e4
        %     v=3;
    end
    
    r(i)=0;
    if a==2
        r(i)=1;
    end
    % ra=r;
    
    if abs(dr(i))>th && a==1
        r(i)=1;
        a=a+1;
        %     if v==0
        %         v=1;
    elseif abs(dr(i))>th && a==2
        r(i)=0;
        a=1; %1
    end
end

subplot(2,1,1)
plot(tiempo_s,dr1)
subplot(2,1,2)
plot(tiempo_s,r)