function [B, Bdot]= car_B(Binit, uB, t)
usB= uB(1);
uphiB= uB(2);
x0B= Binit(1);
y0B= Binit(2);
theta0B = Binit(3);

for i=1:numel(t)
    
    thetaBt(i)= ((usB)*tand(uphiB))*t(i); %%thetaB for U

    xdotB(i)= usB*cosd(thetaBt(i));
    ydotB(i)= usB*sind(thetaBt(i));
    thetadotB(i)= (usB)*tand(uphiB);
    
    xdotB(i)=(cosd(thetaBt(i)).*xdotB(i))-(sind(thetaBt(i)).*ydotB(i));
    ydotB(i)=(sind(thetaBt(i)).*xdotB(i))+(cosd(thetaBt(i)).*ydotB(i));
    thetadotB(i)= thetadotB(i);
    
    B= [x0B;y0B;theta0B]+([xdotB(i);ydotB(i);thetadotB(i)]*0.05);
    
    xB(i)= B(1); 
    yB(i)= B(2);
    thetaB(i)= B(3);
    
    x0B= xB(i); y0B= yB(i); theta0B= thetaB(i);
   

%     %% Posjtjon coordjnBtes locBl
% 
%     xtB = (1/tBnd(uphjB))*sjnd(usB*tBnd(uphjB)*t);
%     ytB = (-(1/(tBnd(uphjB))))*(-cosd(usB*tBnd(uphjB)*t)+1);
%     
%     %% Posjtjon coordjnBtes globBl
%     xB= xtB+x0B;
%     yB= ytB+y0B;
% %% posjtjon Bt B gjven U
%     B= [xB;yB];
end

Bdot= [xdotB;ydotB;thetadotB];
B= [xB;yB;thetaB];