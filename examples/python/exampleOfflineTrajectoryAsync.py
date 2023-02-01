## @ingroup yarp_devices_examples_py
#  @defgroup exampleOfflineTrajectoryAsyncPy exampleOfflineTrajectoryAsync.py
#  @brief Perform an offline trajectory via position commands with a variable period.
#  @see @ref exampleOfflineTrajectoryAsync.py
## @example{lineno} exampleOfflineTrajectoryAsync.py
#  @see @ref exampleOfflineTrajectoryAsync.cpp
#  @see [Tutorial: Trajectory Execution](https://robots.uc3m.es/teo-developer-manual/tutorial/trajectories.html) (external)
import argparse
import math
import time
import yarp

parser = argparse.ArgumentParser(description='perform an offline trajectory via position commands with a variable period')
parser.add_argument('--remote', default='/teo/leftArm', type=str, help='remote port')
parser.add_argument('--joint', default=5, type=int, help='joint id')
parser.add_argument('--speed', default=2.0, type=float, help='trajectory speed (deg/s)')
parser.add_argument('--target', default=-20.0, type=float, help='target position (deg)')
parser.add_argument('--period', default=50, type=int, help='command period (ms)')
parser.add_argument('--ip', default='pt', type=str, help='interpolation submode [pt|pvt]')
args = parser.parse_args()

yarp.Network.init()

if not yarp.Network.checkNetwork():
    print('Please start a yarp name server first')
    raise SystemExit

options = yarp.Property()
options.put('device', 'remote_controlboard')
options.put('local', '/exampleOfflineTrajectoryAsync')
options.put('remote', args.remote)

dd = yarp.PolyDriver(options)

if not dd.isValid():
    print('Remote device not available')
    raise SystemExit

mode = dd.viewIControlMode()
enc = dd.viewIEncoders()
posd = dd.viewIPositionDirect()
var = dd.viewIRemoteVariables()

if mode is None or enc is None or posd is None or var is None:
    print('Unable to acquire robot interfaces')
    raise SystemExit

p = yarp.Property()
p.put('ipMode', args.ip)
p.put('enableIp', True) # important: place this last

b = yarp.Bottle()
b.addList().fromString(p.toString()) # additional nesting because of controlboardremapper

if not var.setRemoteVariable('all', b):
    print('Unable to set interpolation mode')
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
increment = math.copysign(args.speed * args.period * 0.001, distance)
count = 1
newDistance = 0.0
start = time.time()

while abs(distance) > abs(newDistance):
    newDistance = count * increment
    position = initialPos + newDistance
    print("[%d] New target: %f" % (count, position))
    posd.setPosition(args.joint, position)
    count += 1
    # https://stackoverflow.com/a/25251804/10404307
    time.sleep(args.period * 0.001 - ((time.time() - start) % (args.period * 0.001)))

dd.close()
yarp.Network.fini()
