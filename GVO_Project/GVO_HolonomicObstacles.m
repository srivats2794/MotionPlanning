clc;
clear;
t= 0:0.05:5; % prediction time (t limit)
 
 % Initial and preferred actions for Car A and Car B
 Ainit= [0;0;0]; 
 ustarA= [1,0]; 
 Binit= [2;2;0]; 
 ustarB= [0.8, 0]; 
 
 %%% Initial Conditions for tsim=0
 %goal positions
 Agoal= [2;2]; 
 Bgoal= [0;0]; 
 rA = 2; rB = 2; % radius of uncertainty
 
 % Set of all possible actions
 n=91; 
 uA= [ones(1,91);linspace(-45,45,n)];
 uB= [ones(1,91);linspace(-45,45,n)]; 
 
 
 tolerance= 0.5; %tolerance is the value to look around 0 for D square dot
 
 % Free is a flag vector that is assigned the value '0' for those actions that lead to 
 % collision. It helps find u_
 freeA= ones(1,91); 
 freeB= ones(1,91); 
 
 tsim= 1;
 
while(1) %simulation time
    for i=1:1:n
        [A,Adot]= car_A(Ainit, uA(:,i), t);
        [B,Bdot]= car_B(Binit, uB(:,i), t);
        for l=1:numel(t)
            D(l)= sqrt(((A(1,l)- B(1,l))^2)+((A(2,l) - B(2,l))^2)); % distance square
            %distance square dot for A
            DsqrdotA(l)= (2*(A(1,l)-B(1,l))*(Adot(1,l)-Bdot(1,l)))+(2*(A(2,l)-B(2,l))*(Adot(2,l)-Bdot(2,l))); 
            %distance square dot for B
            DsqrdotB(l)= (-2*(A(1,l)-B(1,l))*(Adot(1,l)-Bdot(1,l)))+(-2*(A(2,l)-B(2,l))*(Adot(2,l)-Bdot(2,l))); 
            if -tolerance<DsqrdotA(l)<tolerance %checking for near zeros in D square dot for A
               dA= D(l);
                if(dA<rA+rB)
                    freeA(i)=0;
                end
            end
            if -tolerance<DsqrdotB(l)<tolerance %checking for near zeros in D square dot for B
               dB= D(l);
                if(dB<rA+rB)
                    freeB(i)=0;
                end
            end
        end
    end
    count1=1; count2= 1 ;
    for k=1:n
        if freeA(k)==1     
            udashA(:,count1)= uA(:,k);       %eliminating all infeasible actions for A
            count1=count1+1; 
        end
        if freeB(k)==1
            udashB(:,count2)= uB(:,k);       %eliminating all infeasible actions for B
            count2=count2+1;
        end
    end

    for q=1:count1-1
        %difference in udash and ustar for car A
        udiffA(q)= sqrt(((ustarA(1) -udashA(1,q))^2) + ((ustarA(2) - udashA(2,q))^2));
    end
    for q=1:count2-1
        %difference in udash and ustar for car B
        udiffB(q)= sqrt(((ustarB(1) -udashB(1,q))^2) + ((ustarB(2) - udashB(2,q))^2));
    end

    [uminA,I1]= min(udiffA); [uminB,I2]= min(udiffB); % finding minimum value and index for udash-ustar
    uoptA= [1,udashA(2,I1)]; uoptB= [1, udashB(2,I2)]; % best action for A and B
    
    [Aact(:,tsim), Adotact]= car_A(Ainit,uoptA,tsim); %vehicle kinematics function car A
    [Bact(:,tsim), Bdotact]= car_B(Binit,uoptB,tsim); %vehicle kinematics function car B
    
    xactA= Aact(1,tsim); yactA= Aact(2,tsim); % actual XY coordinates for car A for a feasible uoptA
    xactB= Bact(1,tsim); yactB= Bact(2,tsim); % actual XY coordinates for car B for a feasible uoptB
    
    Ainit= Aact(:,tsim); %updating Ainit for next timestep
    Binit= Bact(:,tsim); %updating Binit for next timestep

    thetanewA= atand((Agoal(2)-yactA)/(Agoal(1)-xactA)); % theta required for reaching goal- A
    thetanewB= atand((Bgoal(2)-yactB)/(Bgoal(1)-xactB)); % theta required for reaching goal- B

    ustarA= [1; atand(thetanewA)];
    ustarB= [1; atand(thetanewB)];
    
    if Aact(1,tsim)== Agoal(1) && Aact(2,tsim)== Agoal(2)
        break
    end
   
    tsim= tsim+1;
end
