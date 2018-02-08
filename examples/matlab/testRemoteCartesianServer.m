% MATLAB example

%> @ingroup asibot_examples_m
%> \defgroup testRemoteCartesianServerM testRemoteCartesianServer.m
%>
%> @brief This example connects to a running \ref cartesianServer module using the Python implementation
%> of the \ref CartesianClient library to move in cartesian Space.
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
%> From within MATLAB, navigate to the ASIBOT MATLAB examples path and run the program:
%>
%>\verbatim [MATLAB console] testRemoteCartesianServer.m \endverbatim
%> <b>Modify</b>
%>
%> This file can be edited at repos/asibot-main/example/matlab/testRemoteCartesianServer.m
%>

disp 'WARNING: requires a running instance of cartesianServer'

LoadYarp  % imports YARP and  connect to YARP network

simCart = CartesianClient  % make a cartesian controller object we call 'simCart'
simCart.open('/ravebot')  % make the 'simCart' establish connections

disp 'asking for inversion of [0.1, 0.0, 0.9, 90.0, 0.0]'
inversion = [0.1, 0.0, 0.9, 90.0, 0.0]
coords = yarp.DVector
res = simCart.inv(inversion,coords)
[coords.get(0) coords.get(1) coords.get(2) coords.get(3) coords.get(4)]

targets = [0.1, 0.0, 0.9, 90.0, 0.0]
disp 'Movj to targets: {0.1, 0.0, 0.9, 90.0, 0.0}...'
simCart.movj(targets)

disp 'Delaying 5 seconds...'
yarp.Time.delay(5)

disp 'Stopping...'
simCart.stop()

newtargets = [0.4, 0.0, 0.9, 90.0, 0.0]
disp 'Movl to newtargets: {0.4, 0.0, 0.9, 90.0, 0.0}...'
simCart.movl(newtargets)

disp 'Delaying 20 seconds...'
yarp.Time.delay(20)

disp 'asking for current position'
coords = yarp.DVector
res = simCart.stat(coords)
[coords.get(0) coords.get(1) coords.get(2) coords.get(3) coords.get(4)]

disp 'Bye!'

simCart.close()  % close the 'simCart' connections

