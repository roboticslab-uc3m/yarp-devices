##
# @ingroup yarp_devices_examples_py
# @defgroup exampleOnlineTrajectoryRemotePushPy exampleOnlineTrajectoryRemotePush.py
# @brief See @ref exampleOnlineTrajectoryRemotePush

import argparse
import math
import time
import yarp

parser = argparse.ArgumentParser(description='perform an online trajectory via position commands, pushing new setpoints at the sender\'s discretion')
parser.add_argument('--remote', default='/teo/leftArm', type=str, help='remote port')
parser.add_argument('--joint', default=5, type=int, help='joint id')
parser.add_argument('--speed', default=2.0, type=float, help='trajectory speed (deg/s)')
parser.add_argument('--target', default=-20.0, type=float, help='target position (deg)')
parser.add_argument('--period', default=50, type=int, help='command period (ms)')
args = parser.parse_args()

yarp.Network.init()

if not yarp.Network.checkNetwork():
    print('Please start a yarp name server first')
    raise SystemExit

options = yarp.Property()
options.put('device', 'remote_controlboard')
options.put('local', '/exampleOnlineTrajectoryRemotePush')
options.put('remote', args.remote)

dd = yarp.PolyDriver(options)

if not dd.isValid():
    print('Remote device not available')
    raise SystemExit

mode = dd.viewIControlMode()
enc = dd.viewIEncoders()
posd = dd.viewIPositionDirect()

if not mode.setControlMode(args.joint, yarp.VOCAB_CM_POSITION_DIRECT):
    print('Unable to set position direct mode')
    raise SystemExit

time.sleep(0.1) # hacky, but we need to make sure remote data arrived prior to calling getEncoder()
initialPos = enc.getEncoder(args.joint)

print('Current enc value: %d' % initialPos)
input('Press ENTER to start')
print('Moving joint %d to %d degrees...' % (args.joint, args.target))

distance = args.target - initialPos
current = initialPos
i = 1
start = time.time()

while abs(distance) > abs(current - initialPos):
    current = current + math.copysign(args.speed * args.period * 0.001, distance)
    print("[%d] New target: %f" % (i, current))
    posd.setPosition(args.joint, current)
    i += 1
    # https://stackoverflow.com/a/25251804/10404307
    time.sleep(args.period * 0.001 - ((time.time() - start) % (args.period * 0.001)))

dd.close()
yarp.Network.fini()
