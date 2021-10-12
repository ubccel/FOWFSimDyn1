function AngleOL = AngleOLFunc(CenterDist,WakeVarRad,TurbRad)
AngleOL = zeros(1,length(WakeVarRad));
for i = 1:length(WakeVarRad)
 if CenterDist <= abs(WakeVarRad(i) - TurbRad) + 0.001
  if WakeVarRad(i) <= TurbRad
   AngleOL(i) = 2*pi;
  else
   AngleOL(i) = 0;
  end
 elseif (CenterDist > abs(WakeVarRad(i) - TurbRad)) && (CenterDist < WakeVarRad(i) + TurbRad)
  AngleOL(i) = 2*acos((WakeVarRad(i)^2 - TurbRad^2 + CenterDist^2)/(2*WakeVarRad(i)*CenterDist));
 else
  AngleOL(i) = 0;
 end
end