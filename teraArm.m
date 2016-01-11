%================================= teraArm ===============================
%
%
%
%
%================================= teraArm ===============================

%
%  Name:	teraArm.m
%
%  Author:	Roland Tull
%
%  Created:	11/06/2014
%  Modified: 12/03/2014
%
%================================= teraARm ===============================


function xx=teraArm(command,grab)
xx.Move = @Move;
xx.retire= @retire;
army=lynx6alumi();
army.gotoHome(5);
pause(5);
pos=[0 0 0 0 0];
Move(command,grab);




function oo=Move(command,grab)
t=3;

timeValues=[5 8 10 12];
for ii=1:length(command)
    if(abs(command(ii)-pos(ii))>30 && t<timeValues(1))
        t=timeValues(1);
    end 
    if(abs(command(ii)-pos(ii))>60 && t<timeValues(2))
        t=timeValues(2);
    end
    if(abs(command(ii)-pos(ii))>90 && t<timeValues(3))
        t=timeValues(3);
    end 
    if(abs(command(ii)-pos(ii))>90 && t<timeValues(4))
        t=timeValues(4);
    end 
    pos(ii)=command(ii);
end
if(grab==1)
    army.setArm([command(1);command(2);command(3);command(4);command(5);0.75],t);
elseif(grab==2)
    army.setArm([command(1);command(2);command(3);command(4);command(5);0.65],t);
elseif(grab==3)
    army.setArm([command(1);command(2);command(3);command(4);command(5);1.25],t);
else
    army.setArm([command(1);command(2);command(3);command(4);command(5)],t);
end
oo=t;
end  
function retire()
army.gotoSleep();
army.free();
end
end       