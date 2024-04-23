function Ak = A(k)
    Ak = [1, 0.0025*k; -0.1*cos(0.3*k), 1 + (0.05^(3/2))*sin(0.5*k)*sqrt(k)];
end
