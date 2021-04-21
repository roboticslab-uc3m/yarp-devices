##
# @ingroup yarp_devices_examples_py
# @defgroup exampleOnlineTrajectoryRemotePullPy exampleOnlineTrajectoryRemotePull.py
# @brief See @ref exampleOnlineTrajectoryRemotePull

import argparse
import math
import time
import yarp

parser = argparse.ArgumentParser(description='perform an online trajectory via position commands attending a remote callback')
parser.add_argument('--robot', default='/teo', type=str, help='robot port')
parser.add_argument('--part', default='/leftArm', type=str, help='part port')
parser.add_argument('--joint', default=5, type=int, help='joint id')
parser.add_argument('--speed', default=2.0, type=float, help='trajectory speed (deg/s)')
parser.add_argument('--target', default=-20.0, type=float, help='target position (deg)')
args = parser.parse_args()

class SyncCallback(yarp.BottleCallback):
    def __init__(self, initialPos, speed, cmd):
        super().__init__()
        self.count = 0
        self.e0 = initialPos
        self.v = speed
        self.offset = 0.0
        self.command = cmd

    def onRead(self, b, reader):
        if b.size() == 2:
            currentTime = b.get(0).asInt32() + b.get(1).asInt32() * 1e-9

            if self.count == 0:
                self.offset = currentTime

            t = currentTime - self.offset
            e = self.e0 + self.v * t

            print('[%d] New target: %f' % (self.count, e))
            self.command(e)
            self.count += 1

yarp.Network.init()

if not yarp.Network.checkNetwork():
    print('Please start a yarp name server first')
    raise SystemExit

options = yarp.Property()
options.put('device', 'remote_controlboard')
options.put('local', '/exampleOnlineTrajectoryRemotePull')
options.put('remote', args.robot + args.part)

dd = yarp.PolyDriver(options)

if not dd.isValid():
    print('Remote device not available')
    raise SystemExit

mode = dd.viewIControlMode()
enc = dd.viewIEncoders()
posd = dd.viewIPositionDirect()

syncPort = yarp.BufferedPortBottle()
syncPort.setReadOnly()

if not syncPort.open('/exampleOnlineTrajectoryRemotePull/sync:i'):
    print('Unable to open local sync port')
    raise SystemExit

if not yarp.Network.connect(args.robot + '/sync:o', syncPort.getName(), 'fast_tcp'):
    print('Unable to connect to remote sync port')
    raise SystemExit

if not mode.setControlMode(args.joint, yarp.VOCAB_CM_POSITION_DIRECT):
    print('Unable to set position direct mode')
    raise SystemExit

time.sleep(0.1) # hacky, but we need to make sure remote data arrived prior to calling getEncoder()
initialPos = enc.getEncoder(args.joint)

print('Current enc value: %d' % initialPos)
input('Press ENTER to start')
print('Moving joint %d to %d degrees...' % (args.joint, args.target))

distance = args.target - initialPos
velocity = math.copysign(args.speed, distance)

callback = SyncCallback(initialPos, velocity, lambda pos: posd.setPosition(args.joint, pos))
syncPort.useCallback(callback)

lastRef = yarp.DVector(1)

while posd.getRefPosition(args.joint, lastRef) and abs(lastRef[0] - initialPos) < abs(distance):
    time.sleep(0.01)

syncPort.interrupt()
syncPort.close()

dd.close()
yarp.Network.fini()
