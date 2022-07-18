## @ingroup yarp_devices_examples_py
#  @defgroup exampleRemoteJr3Py exampleRemoteJr3.py
#  @brief This example connects to a remote @ref Jr3 device.

## @example{lineno} exampleRemoteJr3.py

import time
import yarp

yarp.Network.init()

if not yarp.Network.checkNetwork():
    print('[error] Please try running yarp server')
    quit()

options = yarp.Property()
options.put('device', 'multipleanalogsensorsclient')
options.put('remote', '/jr3')
options.put('local', '/exampleRemoteJr3')

device = yarp.PolyDriver(options)

if not device.isValid():
    print('Device not available')
    quit()

sensor = device.viewISixAxisForceTorqueSensors()

channels = sensor.getNrOfSixAxisForceTorqueSensors()
print('Channels:', channels)

for ch in range(channels):
    name = sensor.getSixAxisForceTorqueSensorName(ch)
    print('Channel %d has name: %s' % (ch, name))

status = -1
retry = 0
MAX_RETRIES = 10

while status != yarp.MAS_OK:
    status = yarp.MAS_OK # = 0

    for ch in range(channels):
        status += sensor.getSixAxisForceTorqueSensorStatus(ch)

    retry += 1
    print('Waiting for sensor to be ready... retry', retry)

    if retry == MAX_RETRIES:
        print('[error] Sensor initialization failure, max number of retries exceeded')
        quit()

    time.sleep(0.1)

n = 0
MAX_ITERS = 500

print('Performing %d read iterations' % MAX_ITERS)

out = yarp.Vector()

while n < MAX_ITERS:
    n += 1

    for ch in range(channels):
        timestamp = sensor.getSixAxisForceTorqueSensorMeasure(ch, out)
        print("[%d] [%f] Channel %d: %s" % (n, timestamp, ch, out.toString()));

    time.sleep(0.01)

print('Done')
