import yarp
import Tkinter as tk

yarp.Network.init()

root = tk.Tk()

handSensors = tk.LabelFrame(root, text="Hand sensors")
handSensors.pack(fill="both", expand="yes", side="left")

forearmSensors = tk.LabelFrame(root, text="Forearm sensors")
forearmSensors.pack(fill="both", expand="yes", side="left")

armSensors = tk.LabelFrame(root, text="Arm sensors")
armSensors.pack(fill="both", expand="yes", side="left")

sensors = []
parts = [handSensors, forearmSensors, armSensors]

for part in range(3):
    labels = []
    sensors.append(labels)
    
    for sensor in range(16):
        label = tk.Label(parts[part], text="s%d: " % (sensor + 1))
        label.pack()
        labels.append(label)

class DataProcessor(yarp.PortReader):
    def read(self, connection):
        print("in DataProcessor.read")
        
        if not connection.isValid():
            print("Connection shutting down")
            return False
        
        b = yarp.Bottle()
        
        print("Trying to read from connection")
        ok = b.read(connection)
        
        if not ok:
            print("Failed to read input")
            return False
        
        print("Received [%s]" % b.toString())
        
        i = 0
        
        for part in range(3):
            for sensor in range(16):
                sensors[part][sensor]['text'] = 's%d: %d' % (sensor + 1, b.get(i).asDouble())
                i += 1
        
        return True

p = yarp.Port()
r = DataProcessor()
p.setReader(r)
p.open("/sensor_reader_gui");

root.mainloop()

p.close()

yarp.Network.fini();
