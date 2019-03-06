function tau = ttc(p1,p2)
        r = p1(7)+p2(7);
        w = p1(3:4)-p2(3:4);
        c = w*w' - r^2;
        if c<0
            tau = 0;   %checking for present collision            
            return
        end    
        v =p1(5:6)-p2(5:6);
        a = v*v';
        b = w*v';
        discr = b^2-a*c;                                                          
        if discr <= 0
            tau = 0;%for imaginary roots
            return
        end
        tau = (-b-sqrt(discr))/a ; %the earliest collision
        if or(tau < 0,tau>8)  % past collision means no collision
            tau=0;            
        end
end