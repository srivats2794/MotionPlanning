clear 
clc

campus = csvread('campus.csv',1,1, [1 1 175659 7]);
%campus = csvread('campus.csv',1,1, [1 1 50000 7]); %reading 50000 values

%index = zeros(2,1);
index = zeros(7955,1);
%index = zeros(35,1);
%index = [];
index(1,1) = 1;
j = 1;

for i = 1:(length(campus)-1)
    prev_time = campus(i,7);
    time = campus(i+1,7);   
    if time ~= prev_time
       j = j+1;
       index(j,1) = i+1;
    end
end

index(j+1,1) = length(campus);

time_to_collision = zeros(912823,1);
random_time_to_collision = zeros(912823,1);
count = 0;
block = [];


for a = 1:(length(index)-1)
    
    if a == length(index)-1
        block = campus(index(a):index(a+1),1);
    else
        block = campus(index(a):index(a+1)-1,1);
    end
    
    interactions = combnk(block,2);
    
    for b = 1:size(interactions,1)
        if a~= length(index)-1
            upper = index(a+1)-1;
        else
            upper = index(a+1);
        end
        for c = index(a):upper
            if interactions(b,1) == campus(c,1)
                first_index = c;
            end
            if interactions(b,2) == campus(c,1)
                sec_index = c;
            end
        end
        
        
        random_sec_index = find(campus(:,1)==interactions(b,2),1);
        count = count+1;
        time_to_collision(count) = ttc(campus(first_index,2), campus(first_index,3), campus(first_index,4), campus(first_index,5), campus(first_index,6), campus(sec_index,2), campus(sec_index,3), campus(sec_index,4), campus(sec_index,5), campus(sec_index,6));
        random_time_to_collision(count) = ttc(campus(first_index,2), campus(first_index,3), campus(first_index,4), campus(first_index,5), campus(first_index,6), campus(random_sec_index,2), campus(random_sec_index,3), campus(random_sec_index,4), campus(random_sec_index,5), campus(random_sec_index,6));
    end
end

[~,~,v1]=find(time_to_collision);
[~,~,v2]=find(random_time_to_collision);
subplot(1,2,1)
histogram(v1,50,'Normalization','probability');
subplot(1,2,2)
histogram(v2,50,'Normalization','probability');

function tau_final = ttc(x1, y1, vel_x1, vel_y1, radius1, x2, y2, vel_x2, vel_y2, radius2)

    r = radius1 + radius2;
    x = [x1 y1] - [x2 y2];
    c = x*x' - r*r;
    
    if c<0
        tau_final = 0;
        return
    end

    v = [vel_x1 vel_y1] - [vel_x2 vel_y2];

    a = v*v';
    b = x*v';
    discr = b^2 - a*c;   %Discriminant
  
    if discr <= 0
        tau_final = NaN;
        return
    end

    tau = (-b-sqrt(discr))/a;   %Time to collision
    
    if tau < 0
        tau_final = NaN;
        return
    end
    
    if ~isnan(tau) && tau >5
    tau_final = NaN;
    return
    end
    
    tau_final = tau;
end