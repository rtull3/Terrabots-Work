%================================= lynx6 =================================
%
%
%
%
%================================= lynx6 =================================

%
%  Name:	lynx6alumi.m
%
%  Author:	Patricio A. Vela, pvela@gatech.edu
%
%  Created:	08/09/2007
%  Modified: 4/1/2015
%
%================================= lynx6 =================================
classdef lynx6alumi < handle

properties
    
  alphaIds    = [   0    1    2    3    4    5];		
				% Servo ID numbers on the SSC-32 control board.
alpha2musec = diag(1./[0.09 0.09 0.09 0.09 0.09 0.09/75]);	
				% Converts degrees to microsecs for manipulator.
      				%  This may be only kinda accurate since it 
      				%  assumes the same swing for all servos, which
      				%  is not necessarily true.  
				%  I don't really use this since the musec
				%  commands are interpolated from the xxLims
				%  variables.
      				%  Need to investigate.

alphaHome   = [   0;   0;   0;   0;   0; 1.0];		
				% Home position as a joint configuration.
%alphaSleep  = [   0,  75, -120, -75,   0, 1.0];	% Folded up.
alphaSleep  = [   0;  125;   70;  70;   0;  1.0];	% Leaning on block.
				% Sleep position as a joint configuration.
				%   This is what you put it to before powering 
      				%   down.

alphaOrient = [ -1,   1,   1,  -1,   1,  -1];		
				% To switch orientation in case servo rotates 
      				%   in the wrong direction 
      				%   (e.g. need right-hand rule).

alphaLims = [ -90,   -90, -90, -90, -90, 0.00;		
                0,  0,   0,   0,   0, 3/4;
      	       90, 90,  90,  90,  80, 1.25];
				% Limits of the joints, either angular values 
      	    			%   or linear values as in the gripper.
				%   The middle value is some measured
				%   intermediate location.
musecLims = [ 520  730  580  590  600 800;			
             1500 1480 1390 1450 1430 1500;			
             2390 2190 2290 2350 2360 2100];
				% How these limits translate to microseconds 
      				%   for the servo commands.

mm2in   = 1/25.4;
linklen;			
				% Measured link lengths of the manipulator in
				%   millimeters but converted to inches.


% Serial port setup defaults:
  serport = [];
  serialid = [];
end
%)
%
%============================ Member Functions ===========================
%
%(
methods
 %------------------------------- lynx6alumi ------------------------------
 %
 %  Constructor
 %
 %(   
function this = lynx6alumi(setup)
	
    this.linklen=[65 144 127 62.5 15]*this.mm2in;

  % Parse the setup argument now.
  if (nargin > 0)
    fnames = fieldnames(setup);
    for ii = 1:length(fnames)
      switch fnames{ii}
        case 'linklen','alphaHome','alphaOrient',
          eval(['this.' fnames{ii} ' = getfield(setup, fnames{ii});']);
        case 'musecLims',
          if (size(setup.musecLims,2) == 5)
  	        if (size(setup.musecLims, 1) == 2)
  	          this.musecLims(:,1) = setup.musecLims(:,1);
  	          this.musecLims(:,3) = setup.musecLims(:,2);
  	          this.musecLims(:,2) = mean(setup.musecLims,1);
  	        elseif (size(setup.musecLims, 1) == 3)
  	          this.musecLims = setup.musecLims;
  	        end
          end
        case 'alphaLims',
          if (size(setup.alphaLims,2) == 5)
  	        if (size(setup.alphaLims, 1) == 2)
  	          this.alphaLims(:,1) = setup.alphaLims(:,1);
  	          this.alphaLims(:,3) = setup.alphaLims(:,2);
  	          this.alphaLims(:,2) = mean(setup.alphaLims,1);
  	        elseif (size(setup.alphaLims, 1) == 3)
  	          this.alphaLims = setup.alphaLims;
  	        end
          end
        case 'COM','baud','terminator',
          eval(['this.serport.' fnames{ii} ' = getfield(setup, fnames{ii});']);
      end
    end
  end

  this.serport.com  = 'COM1';		% COM port is COM1.
  this.serport.baud = 115200;		% Baud rate is 115200 bps.
  this.serport.terminator = 'CR';	% Message terminator is Carriage Return.

  % Open up the serial port.
  ispc
  if isunix
    device = '/dev/ttyS0';
    this.serialid = fopen(device,'w+');
  elseif ispc
    this.serialid = serial(this.serport.com,'BaudRate',this.serport.baud, ...
                                       'Terminator',this.serport.terminator);
    fopen(this.serialid);
    % Note that in Windows, if the program crashes then the com port is still 
    %   open.  You will not be able to reopen the port and you won't be able to 
    %   access the serialid you had before.  If you can still run the
	%   manipulator class variable, then run the free routine.  If you no 
	%   longer have access to the class variable, the only option I know of
	%   is to restart Matlab.  Or you could try to do an 'fclose all' and
	%   see if that works.  I haven't tested it out.
    %
    %   Read documentation: 'help fclose'
  end

end
%============================ Member Functions ===========================

  %--------------------------- lynx6_gotoHome --------------------------
  %
  %
  function gotoHome(this,time)
  
    if (nargin == 0)
      time = 4;
    end
  
    this.setArm(this.alphaHome, time);
  
  end
  
  %-------------------------- lynx6_gotoSleep --------------------------
  %
  %
  function gotoSleep(this,time)
  
    if (nargin == 1)
      time = 4;
    end
  
    this.setArm(this.alphaSleep, time);
  
  end
  
  
  %-------------------------- lynx6_setServos --------------------------
  %
  %
  function setServos(this,pwmsigs, time, defaultLims)
  
    if (nargin < 2)
      time = 2;
    elseif (isempty(time))
      time = 2;
    end
  
    if (nargin < 3)
      defaultLims = true;
    end
    
    if ( (size(pwmsigs,1) ~= 6) || (size(pwmsigs,2) ~= 1) )
      disp('Incorrect dimensions, ignoring command.');
      return;
    end
    
    if (defaultLims)
      ticks = min(this.musecLims(3,:),max(this.musecLims(1,:), pwmsigs'));
    else
      ticks = min(2500, max(500, pwmsigs'));
    end
  
    cmdstr = sprintf('#%d P%d ',[this.alphaIds ; ticks]);		
    cmdstr = [cmdstr sprintf(' T%d \n', round(time*1000))];
  
    fprintf(cmdstr); 			% Print actual command to STDOUT.
    fprintf(this.serialid, cmdstr);
  
  end
  
  %---------------------------- lynx6_setArm ---------------------------
  %
  %
  function setArm(this,alpha, time)
  
    if (nargin == 1)
      time = 4;
    end
  
    if ( (size(alpha,1) == 5) && (size(alpha,2) == 1) )
      alpha(6) = this.alphaHome(6);
    elseif ( (size(alpha,1) ~= 6) || (size(alpha,2) ~= 1) )
      size(alpha)
      return;
    end
  
    alpha = min(this.alphaLims(3,:),max(this.alphaLims(1,:), alpha'));
    for ii = 1:length(this.alphaIds)
      if (this.alphaOrient(ii) == 1)
        ticks(ii) = interp1(this.alphaLims(:,ii), this.musecLims(:,ii), alpha(ii));
      else
        ticks(ii) = interp1(this.alphaLims(end:-1:1,ii), this.musecLims(:,ii), alpha(ii));
      end
    end
    ticks = round(ticks);
    cmdstr = sprintf('#%d P%d ',[this.alphaIds ; ticks]);
    cmdstr = [cmdstr sprintf(' T%d \n', round(time*1000))];
  
    fprintf(cmdstr);
    fprintf(this.serialid, cmdstr);
  
  end
  
  %------------------------------ display ------------------------------
  %
  %  Display the manipulator.
  %
  %(
  function display(this,alphadisp)
  if (nargin == 0)
    alphadisp = alpha;
  end

  vparms.home = 'straight-out';
  lynx6_display((pi/180)*alphadisp, this.linklen, vparms);
  end

  %)
  %-------------------------- lynx6_forwardkin -------------------------
  %
  %
  function g = forwardkin(this,alpha, totip)
  
  if (nargin == 1)
    totip = false;
  end
  
  g = forwardkin_straightout((pi/180)*alpha(1:5), this.linklen, totip);
  
  end
  
  %-------------------------- lynx6_inversekin -------------------------
  %
  %
  function alpha = inversekin(this, gdes, totip, solfact)
  
  if (nargin == 1)
    totip = false;
  end
  
  if (nargin < 3)
    solfact = [];
  end
  
  alpha = (180/pi)*inversekin_straightout(gdes, this.linklen, totip, solfact);
  
  end
  
  %----------------------------- lynx6_free ----------------------------
  %
  %
  function free(this)
  
    fclose(this.serialid);
  
  end

end
methods(Static)
%============================ Helper Functions ===========================

%------------------------------ forwardkin -----------------------------
%
%
%
function g = forwardkin_straightup(alpha, linklen, totip)

if (nargin == 1)
  totip = false;
end

g1 = [ cos(alpha(1)), -sin(alpha(1)),              0,          0 ; ...
       sin(alpha(1)),  cos(alpha(1)),              0,          0 ; ...
                   0,              0,              1, linklen(1) ; ...
		   0,              0,              0,          1 ];

g2 = [             1,              0,              0,          0 ; ...
                   0,  cos(alpha(2)), -sin(alpha(2)),          0 ; ...
		   0,  sin(alpha(2)),  cos(alpha(2)),          0 ; ...
		   0,              0,              0,          1 ];

g3 = [             1,              0,              0,          0 ; ...
                   0,  cos(alpha(3)), -sin(alpha(3)),          0 ; ...
		   0,  sin(alpha(3)),  cos(alpha(3)), linklen(2) ; ...
		   0,              0,              0,          1 ];

g4 = [             1,              0,              0,          0 ; ...
                   0,  cos(alpha(4)), -sin(alpha(4)),          0 ; ...
		   0,  sin(alpha(4)),  cos(alpha(4)), linklen(3) ; ...
		   0,              0,              0,          1 ];

g5 = [ cos(alpha(5)), -sin(alpha(5)),              0,          0 ; ...
       sin(alpha(5)),  cos(alpha(5)),              0,          0 ; ...
                   0,              0,              1, linklen(4) ; ...
		   0,              0,              0,          1 ];
%g5 = [ cos(alpha(5)),              0,  sin(alpha(5)),          0 ; ...
                   %0,              1,              0,          0 ; ...
      %-sin(alpha(5)),              0,  cos(alpha(5)), linklen(4) ; ...

g = g1*g2*g3*g4*g5;

if (totip)
  g6 = [ 1, 0, 0,          0;  ...
         0, 1, 0,          0;  ...
	 0, 0, 1, linklen(5);  ...
	 0, 0, 0,          1];
  g = g*g6;
end

end

     %--------------- straight out configuration ---------------

function ge = forwardkin_straightout(alpha, linklen, totip)

if (nargin == 1)
  totip = false;
end

Rx = @(alpha)[ 1, 0, 0; 0, cos(alpha), -sin(alpha); 0, sin(alpha), cos(alpha)];
Ry = @(alpha)[cos(alpha), 0, sin(alpha); 0, 1, 0; -sin(alpha), 0, cos(alpha)];
Rz = @(alpha)[cos(alpha), -sin(alpha), 0; sin(alpha), cos(alpha), 0; 0, 0, 1];

%-- This is the straight out home configuration: Rz, Rx, Rx, Rx, Ry.
ge = SE3([0;0;linklen(1)] , Rz(alpha(1))) ...
     * SE3([0; 0;  0  ] , Rx(alpha(2))) ...
     * SE3([0; linklen(2); 0] , Rx(alpha(3))) ...
     * SE3([0; linklen(3); 0] , Rx(alpha(4))) ...
     * SE3([0; 0; 0] , Ry(alpha(5)));

if (totip)
  ge = ge*SE3([0;sum(linklen(4:5));0], eye(3));
else
  ge = ge*SE3([0;linklen(4);0], eye(3));
end

end


%------------------------------ inversekin -----------------------------
%
%  Inverse kinematics for the straight-up home configuration.
%
function alpha = inversekin_straightup(gdes, linklen, totip, solfact)

if ( (nargin <= 2) || isempty(totip) )
  totip = false;
end

if (totip)
  g6 = SE3([0;0;sum(linklen(4:5))], eye(3));
else
  g6 = SE3([0;0;linklen(4)], eye(3));
end
gwrist  = gdes*inv(g6);


dwrist = getTranslation(gwrist) - [0;0;linklen(1)];
rhosq = sum(dwrist.*dwrist);

lcheck = (rhosq - linklen(2)^2 - linklen(3)^2)/(2*linklen(2)*linklen(3));
if (lcheck > 1)
  disp('Out of end-effector range.');
  alpha = [];
  return;
end

a3 = solfact(1) * acos( lcheck );

r = norm(dwrist(1:2));
a = (r + solfact(2)*linklen(3)*sin(a3));
b = -solfact(2)*(linklen(2) + linklen(3)*cos(a3));
c = r - solfact(2)*linklen(3)*sin(a3);
wcands = roots([a 2*b c]);
if (find(imag(wcands)))
  disp('Imaginary roots.  Problem');
  alpha = [];
  return;
end
a2cands = 2*atan(wcands);

if (solfact(3) == -1)		%TODO:: Have to check.
  a2 = min(a2cands);
else
  a2 = max(a2cands);
end

wrfact = linklen(2)*sin(a2) + linklen(3)*sin(a2+a3);
if (r > 10^(-6))
  a1 = atan2( dwrist(1)/wrfact, -dwrist(2)/wrfact );
else
  a1 = 0;
end

Rx = @(alpha)[ 1, 0, 0; 0, cos(alpha), -sin(alpha); 0, sin(alpha), cos(alpha)];
Ry = @(alpha)[cos(alpha), 0, sin(alpha); 0, 1, 0; -sin(alpha), 0, cos(alpha)];
Rz = @(alpha)[cos(alpha), -sin(alpha), 0; sin(alpha), cos(alpha), 0; 0, 0, 1];

%-- This is the straight up home configuration: Rz, Rx, Rx, Rx, Rz.
g1 = SE3( [0 ; 0 ; linklen(1)] , Rz(a1) );
g2 = SE3( [0 ; 0 ;      0    ] , Rx(a2) );
g3 = SE3( [0 ; 0 ; linklen(2)] , Rx(a3) );
g4bar = SE3( [0 ; 0 ; linklen(3)] , eye(3) );

gwact = g1*g2*g3*g4bar;
gorient = inv(g1*g2*g3*g4bar)*gwrist;

gwrist;
gorient;
gwact;
Rorient = getRotation(gorient);
a5 = atan2(-Rorient(1,2), Rorient(1,1));
a4 = atan2(-Rorient(2,3), Rorient(3,3));

alpha = [a1; a2; a3; a4; a5];

end

%
%  Inverse kinematics for the straight-out home configuration.
%
function alpha = inversekin_straightout(gdes, linklen, totip, soln)

%--(1) Parse the arguments.

if ( (nargin <= 2) || isempty(totip) )
  totip = false;
end

if ((nargin < 4) || isempty(soln))
  soln = 0;
end

if (totip)
  g6 = SE3([0;sum(linklen(4:5));0], eye(3));
else
  g6 = SE3([0;linklen(4);0], eye(3));
end

test = false;
if (isscalar(soln))
  test = false;
elseif ((size(soln,2) == 1) && ismember(size(soln,1),[5 6]))
  test = true;
end

epsilon = 1e-10;		% Tolerance for verifying wrist config.
alpha = zeros([5 1]);		% Instantiate the solution vector.

Rx = @(alpha)[ 1, 0, 0; 0, cos(alpha), -sin(alpha); 0, sin(alpha), cos(alpha)];
Ry = @(alpha)[cos(alpha), 0, sin(alpha); 0, 1, 0; -sin(alpha), 0, cos(alpha)];
Rz = @(alpha)[cos(alpha), -sin(alpha), 0; sin(alpha), cos(alpha), 0; 0, 0, 1];

%--(2)  Break out the wrist portion only.  Test for validity.
gdes
g6
inv(g6)
whos
gwrist  = gdes*inv(g6);
pwrist = getTranslation(gwrist) - [0;0;linklen(1)];
rad = norm(pwrist);

% Test for solution lying outside of workspace and correct.
lsq = linklen(2)^2 + linklen(3)^2;
lcheck = (rad^2 - lsq)/(2*prod(linklen(2:3)));
if ( lcheck > 1 )
  disp('Reach is out-of-bounds.  Scaling back.');
end
while ( lcheck > 1 )
  pwrist = 0.95*pwrist;
  rad = norm(pwrist);
  lcheck = (rad^2 - lsq)/(2*prod(linklen(2:3)));
end

%--(3)  Work out the inverse kinematics for the wrist part.
if (rad > 10^(-6))
  alpha(1) = atan2(-pwrist(1), pwrist(2));
else
  alpha(1) = 0;
end
alpha(3) = acos( (rad^2 - linklen(2)^2 - linklen(3)^2) /...
                      (2*linklen(2)*linklen(3)) );
if (~test && bitand(soln,1))
  % If passed a scalar, then pick the specified solution.
  alpha(3) = -alpha(3);
elseif (test)
  % If passed a joint configuration, then pick closest solution.
  apos = norm(soln(3) - alpha(3));
  aneg = norm(soln(3) + alpha(3));
  if (aneg < apos)
    alpha(3) = -alpha(3);
  end
end

rad = norm(pwrist(1:2));
a = rad + linklen(2) + linklen(3)*cos(alpha(3));
b = 2*linklen(3)*sin(alpha(3));
c = rad - linklen(2) - linklen(3)*cos(alpha(3));
rootvals = roots([a b c]);

alpha(2) = 2*atan(rootvals(1));
gfwrist = SE3([0;0;linklen(1)], Rz(alpha(1))) ...
     * SE3([0;0;0], Rx(alpha(2))) ...
     * SE3([0;linklen(2);0], Rx(alpha(3))) ...
     * SE3([0;linklen(3);0], eye(3)); ...
gorient = inv(gfwrist)*gwrist;

% Verify that the solution chosen is correct, else pick other one.
errDist = norm(getTranslation(gorient));
if (errDist >= epsilon)
  alpha(2) = 2*atan(rootvals(2));
  gfwrist = SE3([0;0;linklen(1)], Rz(alpha(1))) ...
       * SE3([0;0;0], Rx(alpha(2))) ...
       * SE3([0;linklen(2);0], Rx(alpha(3))) ...
       * SE3([0;linklen(3);0], eye(3)); ...
  gorient2 = inv(gfwrist)*gwrist;
  errDist2 = norm(getTranslation(gorient));
  if (errDist2 > errDist)
    alpha(2) = 2*atan(rootvals(1));
  else
    gorient = gorient2;
  end
end

%--(4) Done with wrist part, now work on the hand portion.
Rhand = getRotation(gorient);
alpha(4) = atan2(Rhand(3,2), Rhand(2,2));
alpha(5) = atan2(Rhand(1,3), Rhand(1,1));

end

%================================= lynx6 =================================  

end
end    