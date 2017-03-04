## Now what can I do?

Now that you have installed the basic yarp-devices repository, you're probably wondering what to do.

### Initializing the communication servers

Our current implementation uses <a class="el" href="http://eris.liralab.it/yarpdoc/what_is_yarp.html">YARP</a> for communication. Basic use of YARP requires the use of a centralized server. This server associates the low-level implementation of the communication ports with the names we give them. Before executing any program, please launch a yarp server:

```bash
[kinematics-dynamics, terminal 1] yarp server
```

Only one yarp server is required per network.

### Initializing a robot that opens YARP controlboard ports

You can use the testBodyBot program.
