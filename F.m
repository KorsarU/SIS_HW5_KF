function result = F(p, dsr, dsl, b)
    x  = p(1);
    y  = p(2);
    th = p(3);
    
    dx =    ((dsr+dsl)/2)*...
            cos(th+((dsr-dsl)/(2.0*b)));
    dy =    ((dsr+dsl)/2)*...
            sin(th+((dsr-dsl)/(2.0*b)));
    dth=    (dsr-dsl)/b;
    
    result = [x+dx,y+dy,th+dth];
end
