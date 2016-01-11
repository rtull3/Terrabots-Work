%----------------- teraSudomatic --------------------------
%
% This code allows a robot arm to move its end effector to 
% a location relative to its body. The orientation of the end
% effector is fixed.
function vv = teraSudomatic(x,y,z)
    daikaiju=teraArm([0 0 0 0 0],0);
    vv.move=@Move;
    vv.retire=@retire;
    vv.fwdkin=@fwdkin;
    vv.time=@times;
    l1=6.10; %length of arm segment one in inches
    l2=6.10; %length of arm segment two in inches
    l3=2.46; %length of arm segment three in inches
    h=3.14;
    time=5;
    first=Move(x,y,z);
    if(~isempty(first))
    cc=fwdkin(first)
    end

function alphas=Move(x,y,z)
    if(abs(l1-l2)<=sqrt(x^2+y^2+z^2)<=(l1+l2))
        r=sqrt(x^2+y^2); 
        a3c=((x^2+y^2+(z-h)^2-l1^2-l2^2)/(2*l1*l2)); 
        a3s=sqrt(1-a3c^2);
        a=r+l2*a3s;
        b=-2*(l1+l2*a3c);
        c=r-l2*a3s;
        d=r-l2*a3s;
        e=2*(l1+l2*a3c);
        f=r+l2*a3s;
        terms=[a b c];
        terms2=[d e f];
        disp(terms)
        disp(terms2)
        w=(-b-sqrt(b^2-4*a*c))/(2*a);
        disp(w)
        w2=(-e-sqrt(e^2-4*d*f))/(2*d);
        disp(w2)
        a2=2*atan(w)*180/pi;
        disp(a2)
        a3r=atan2(a3s,a3c);
        a2r=2*atan(w);
        if(a2r>2*pi || a2>360)
            disp(a2)
            a2r=a2r-2*pi;
            a2=a2-360;
        elseif(a2r<-2*pi)
            a2r=a2r+2*pi;
            a2=a2+360;
        end
        a1s=x/(l1*sin(a2r)+l2*sin(a2r+a3r));
        a1c=-y/(l1*sin(a2r)+l2*sin(a2r+a3r));
        a1=atan2(a1s,a1c)*180/pi;
        %solve for the next alpha using the law of cosines
        a3=atan2(a3s,a3c)*180/pi; 
        a4=180-a3-a2;
        a4=180-(a4+90);
        alphas=[a1 a2 a3];%computed alphas that follow inverse kinematics
        disp(alphas)
        %If the solution does not map to quadrants 1 and 4, find the
        %supplementry angle to alpha1. Adjust alpha2 and alpha3 so that they can
        %access quadrants 2 & 3
        if(a1>90||a1<-90)
            if(a1>0)
               a1=a1-180;
            else
               a1=a1+180;
            end
                a2=-1*a2;
                a3=-1*a3;
                a4=-1*a4;
        end
        
        allalphas=[a1 a2 a3 a4];
        %display the modified possibly modified alphas and a4
        disp(allalphas)
  
    
    
        if(a1<=90&&a1>=-90 && a2<=90&&a2>=-90 && a3<=90&&a3>=-90 ) 
            %Send the solution to the arm if physically possible
            time=daikaiju.Move([a1 a2 a3 a4 0],0); %order the movement of the arm
            pause(time); %give arm time to execute the command
        else
            disp('Invalid Coordinates not physically possible');%Error Message for invalid coordinates
            alphas = [];
        end
    else
        disp('Invalid Coordinates not theoretically possible');%Error Message for invalid coordinates
        alphas = [];
    end    
end
function retire()
    %Close the serial connection and put the robot arm in its rest positon.
    daikaiju.retire();     
end
function k=fwdkin(alphas)
     gz1= [ cosd(alphas(1)) -sind(alphas(1)) 0 0;...
           sind(alphas(1)) cosd(alphas(1)) 0 0;...
           0 0 1 h;...
           0 0 0 1];
     gx2 = [ 1 0 0 0;...
             0 cosd(alphas(2)) -sind(alphas(2)) 0;...
             0 sind(alphas(2)) cosd(alphas(2)) 0;...
             0 0 0 1];
     gx3 = [ 1 0 0 0;...
             0 cosd(alphas(3)) -sind(alphas(3)) 0;...
             0 sind(alphas(3)) cosd(alphas(3)) l1;...
             0 0 0 1];
     g4 = [ 1 0 0 0;...
            0 1 0 0;...
            0 0 1 l2;...
            0 0 0 1];
     k=gz1*gx2*gx3*g4;
end
    function t= times()
        t=time;
    end

end