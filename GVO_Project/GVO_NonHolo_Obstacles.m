 t= 0:0.05:5; % prediction time (t limit)
 Ainit= [0;0;0]; ustarA= [1,0]; Binit= [2;2;0]; ustarB= [0.8, 0]; %initial conditions for tsim=0
 Agoal= [6;6]; Bgoal= [7;5]; %goal positions
 rD= 2; rC= rD; rB= rC; rA= rB; % radius of uncertainty
 uA= [ones(1,91);linspace(-90,90,91)];  uB= [ones(1,91);linspace(-90,90,91)]; %sample possible actions
 n=91; tolerance= 0.5; %tolerance is the value to look around 0 for D square dot
 freeA= ones(1,91); freeB= ones(1,91); % variable free for finding u dash
for tsim=0:20 %simulation time
    for i=1:n
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

    for q=1:count1
        %difference in udash and ustar for car A
        udiffA(count1)= sqrt(((ustarA(1) -udashA(1,count1))^2) + ((ustarA(2) - udashA(2,count1))^2));
    end
    for q=1:count2
        %difference in udash and ustar for car B
        udiffB(count2)= sqrt(((ustarB(1) -udashB(1,count2))^2) + ((ustarB(2) - udashB(2,count2))^2));
    end

    [uminA,I1]= min(udiffA); [uminB,I2]= min(udiffB); % finding minimum value and index for udash-ustar
    uoptA= [1,udashA(2,I1)]; uoptB= [1, udashB(2,I2)]; % best action for A and B
    
    [Aact, Adotact]= car_A(Ainit,uoptA,1); %vehicle kinematics function car A
    [Bact, Bdotact]= car_B(Binit,uoptB,1); %vehicle kinematics function car B
    
    xactA= Aact(1); yactA= Aact(2); % actual XY coordinates for car A for a feasible uoptA
    xactB= Bact(1); yactB= Bact(2); % actual XY coordinates for car B for a feasible uoptB
    
    Ainit= [Ainit(1)+xactA; Ainit(2)+yactA; Aact(3)]; %updating Ainit for next timestep
    Binit= [Binit(1)+xactB; Binit(2)+yactB; Bact(3)]; %updating Binit for next timestep

    thetanewA= atand((Agoal(2)-yactA)/(Agoal(1)-xactA)); % theta required for reaching goal- A
    thetanewB= atand((Bgoal(2)-yactB)/(Bgoal(1)-xactB)); % theta required for reaching goal- B

    ustarA= [1; atand(thetanewA)];
    ustarB= [1; atand(thetanewB)];
end
