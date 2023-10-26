function[Ce_Ch] = CS2S(Tau)
Ratio = 0.5;
for i = 1:1000
    TAU = -6.624*(Ratio^4)+12.07*(Ratio^3)-8.292*(Ratio^2) + 3.295*(Ratio)+0.004942;
    if TAU<Tau
        Ratio=Ratio+0.0005;
    else
        Ratio=Ratio-0.0005;
    end
    
    if abs(TAU-Tau)<=0.001
        break;
    end
end
        Ce_Ch = Ratio;
end