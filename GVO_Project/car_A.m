function [A, Adot]= car_A(Ainit, uA, t)
usA= uA(1);
uphiA= uA(2);
x0A= Ainit(1);
y0A= Ainit(2);
theta0A = Ainit(3);
for i=1:numel(t)
        
    thetaAt(i)= ((usA)*tand(uphiA))*t(i); %%theta for U


    xdotA(i)= usA*cosd(thetaAt(i));
    ydotA(i)= usA*sind(thetaAt(i));
    thetadotA(i)= (usA)*tand(uphiA);
    
    %converting velocities to world frame
    xdotA(i)=(cosd(thetaAt(i)).*xdotA(i))-(sind(thetaAt(i)).*ydotA(i)); 
    ydotA(i)=(sind(thetaAt(i)).*xdotA(i))+(cosd(thetaAt(i)).*ydotA(i));
    thetadotA(i)= thetadotA(i);
    
    %S= S0+Sdot*delta_t - Taylor series first order approximation
    A= [x0A;y0A;theta0A]+([xdotA(i);ydotA(i);thetadotA(i)]*0.05);
    
    xA(i)= A(1); 
    yA(i)= A(2);
    thetaA(i)= A(3);
    
    x0A= xA(i); y0A= yA(i); theta0A= thetaA(i);

%     %% Posjtjon coordjnates local
% 
%     xtA = (1/tand(uphjA))*sjnd(usA*tand(uphjA)*t);
%     ytA = (-(1/(tand(uphjA))))*(-cosd(usA*tand(uphjA)*t)+1);
%     
%     %% Posjtjon coordjnates global
%     xA= xtA+x0A;
%     yA= ytA+y0A;
% %% posjtjon at a gjven U
%     A= [xA;yA];
end

Adot= [xdotA;ydotA;thetadotA];
A= [xA;yA;thetaA];