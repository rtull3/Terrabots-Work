%================================
%         sudotest.m
%   This script tests the robot arms
%   inverse kinematics against its 
%   forward kinematics with a series of
%   X, Y , and Z coordinates relative to 
%   to the robot arm's base.
i=[7 6 7 -7 -3 7 8.0 7.5 5.0 2.0];
j=[7 6 7 -7 6 7 3.0 4.5 5.0 6.0];
k=[0 0 1 1 9 7 4.4 3.5 8.5 9.3];
disp('Starting')
x=teraSudomatic(i(1),j(1),k(1));
pause(5)

for v= 2:length(i)
    disp(v)
    z=x.move(i(v),j(v),k(v));
    disp(z)
    if(~isempty(z))
    cc=x.fwdkin(z)
    pause(5)
    end
end    