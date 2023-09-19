% MATLAB example

%> @ingroup yarp_devices_examples_m
%> \defgroup exampleRemoteControlBoardM exampleRemoteControlBoard.m
%>
%> @brief This example connects to a remote control board to move in Joint space.
%>
%> <b>Requires YARP 3.0.</b>
%>
%> <b>Legal</b>
%>
%> Copyright: (C) 2012 Universidad Carlos III de Madrid
%>
%> Author: Juan G Victores
%>
%> CopyPolicy: Released under the terms of the LGPLv2.1 or later, see license/LGPL.TXT
%>
%> <b>Running</b> (assuming correct installation)
%>
%>\verbatim [MATLAB console] exampleRemoteControlBoard.m \endverbatim
%> <b>Modify</b>
%>
%> This file can be edited at $YARP_DEVICES_ROOT/examples/matlab/exampleRemoteControlBoard.m
%>

disp 'WARNING: requires a running instance of RaveBot (i.e. testRaveBot or cartesianServer)'

LoadYarp();  % imports YARP and connects to YARP network

options = yarp.Property();  % create an instance of Property, a nice YARP class for storing name-value (key-value) pairs
options.put('device','remote_controlboard');  % we add a name-value pair that indicates the YARP device
options.put('remote','/robotName/partName');  % we add info on to whom we will connect
options.put('local','/exampleRemoteControlBoard');  % we add info on how we will call ourselves on the YARP network

dd = yarp.PolyDriver(options);  % create a YARP multi-use driver with the given options
if isequal(dd.isValid(),1)
    disp '[success] robot available';
else
    disp '[warning] robot NOT available, does it exist?';
end

pos = dd.viewIPositionControl();  % make a position controller object we call 'pos'
if isequal(pos,[])
    disp '[warning] position interface NOT available, does it exist?';
else
    disp '[success] position interface available';
end

vel = dd.viewIVelocityControl();  % make a velocity controller object we call 'vel'
if isequal(vel,[])
    disp '[warning] velocity interface NOT available, does it exist?';
else
    disp '[success] velocity interface available';
end

enc = dd.viewIEncoders();  % make an encoder controller object we call 'enc'
if isequal(enc,[])
    disp '[warning] encoder interface NOT available, does it exist?';
else
    disp '[success] encoder interface available';
end

mode = dd.viewIControlMode();  % make a mode controller object we call 'mode'
if isequal(mode,[])
    disp '[warning] control mode interface NOT available, does it exist?';
else
    disp '[success] control mode interface available';
end

axes = enc.getAxes();
for i = 1:axes
    mode.setControlMode(i-1, yarp.Vocab_encode('pos'));  % use the object to set the device to position mode (as opposed to velocity mode) (note: stops the robot)
end

disp 'test positionMove(1,35) -> moves motor 1 (start count at motor 0) to 35 degrees';
pos.positionMove(1,35);

disp 'test delay(5)';
yarp.Time.delay(5);

v = yarp.DVector(axes);  % create a YARP vector of doubles the size of the number of elements read by enc, call it 'v'
enc.getEncoders(v);  % read the encoder values and put them into 'v'
disp (strcat('v[1] is: ',num2str(v.get(1))));  % print element 1 of 'v', note that motors and encoders start at 0

for i = 1:axes
    mode.setControlMode(i-1, yarp.Vocab_encode('vel'));  % use the object to set the device to velocity mode (as opposed to position mode)
end

disp 'test velocityMove(0,10) -> moves motor 0 (start count at motor 0) at 10 degrees per second';
vel.velocityMove(0,10);

disp 'test delay(5)';
yarp.Time.delay(5);

vel.velocityMove(0,0);
dd.close();
disp 'bye!';
yarp.Network.fini();  % disconnect from the YARP network

