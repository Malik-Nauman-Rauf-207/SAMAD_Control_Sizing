function[Delr,Sigma] = Del_Beta(rau,v_w,S_s,Cdy,v_t,S_w,b_w,Cyo,CyB,Beta,CyDr,Cno,CnB,CnDr,Fw,dc)
Sig = 0.1;
q = 0.5*rau*(v_t^2);
Func = (0.5*rau*(v_t^2)*S_w*b_w)*(Cno+CnB*(Beta-Sig)+CnDr*(((0.5*rau*(v_w^2)*S_s*Cdy)/(q*S_w*CyDr))-(Cyo/CyDr)-((CyB/CyDr)*(Beta-Sig))))+Fw*dc*cos(Sig);
dFunc = (0.5*rau*(v_t^2)*S_w*b_w)*(-CnB+CnDr*(-((CyB/CyDr)*(Beta-Sig))))-Fw*dc*sin(Sig);
S = Sig - Func/dFunc;
while ( abs(Sig-S) > 0.01 )
		Sig = S;
        Func = (0.5*rau*(v_t^2)*S_w*b_w)*(Cno+CnB*(Beta-Sig)+CnDr*(((0.5*rau*(v_w^2)*S_s*Cdy)/(q*S_w*CyDr))-(Cyo/CyDr)-((CyB/CyDr)*(Beta-Sig))))+Fw*dc*cos(Sig);
		dFunc = (0.5*rau*(v_t^2)*S_w*b_w)*(-CnB+CnDr*(-((CyB/CyDr)*(Beta-Sig))))-Fw*dc*sin(Sig);
        S = Sig - Func/dFunc;
end
Sigma = S;
Delr = ((0.5*rau*(v_w^2)*S_s*Cdy)/(q*S_w*CyDr))-(Cyo/CyDr)-((CyB/CyDr)*(Beta-Sig));