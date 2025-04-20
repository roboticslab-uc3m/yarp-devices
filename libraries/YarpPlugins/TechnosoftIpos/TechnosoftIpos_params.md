|  Group  |       Parameter        |  Type  |      Units       | Default Value | Required |            Description            |             Notes             |
|:-------:|:----------------------:|:------:|:----------------:|:-------------:|:--------:|:---------------------------------:|:-----------------------------:|
|         |         canId          |  int   |                  |               |   yes    |            CAN bus ID             |             1-127             |
|         |     useEmbeddedPid     |  bool  |                  |     true      |    no    |         use embedded PID          |                               |
|         |          name          | string |                  |               |   yes    |             axis name             |                               |
|         |          type          | string |                  |               |   yes    |         joint type vocab          |       atrv, atpr, unkn        |
|         |          max           | double |     deg or m     |               |   yes    |         upper joint limit         |                               |
|         |          min           | double |     deg or m     |               |   yes    |         lower joint limit         |                               |
|         |         maxVel         | double |   deg/s or m/s   |               |   yes    |         maximum velocity          |                               |
|         |        refSpeed        | double |   deg/s or m/s   |               |   yes    |          reference speed          |                               |
|         |    refAcceleration     | double | deg/s^2 or m/s^2 |               |   yes    |      reference acceleration       |                               |
|         |        extraTr         | double |                  |      1.0      |    no    |       additional reduction        |       see gearbox group       |
|         |     samplingPeriod     | double |        s         |               |   yes    |    controller sampling period     |                               |
|         |        reverse         |  bool  |                  |               |   yes    |   reverse motor encoder counts    |                               |
|         |    heartbeatPeriod     | double |        s         |               |   yes    |       CAN heartbeat period        |                               |
|         |       syncPeriod       | double |        s         |               |   yes    |      CAN SYNC message period      |                               |
|         |   initialControlMode   | string |                  |      idl      |    no    |    initial control mode vocab     |                               |
|         |       sdoTimeout       | double |        s         |     0.02      |    no    |          CAN SDO timeout          |                               |
|         |   driveStateTimeout    | double |        s         |      2.0      |    no    |      CAN drive state timeout      |                               |
|         |    tpdo1InhibitTime    | double |        s         |      0.0      |    no    |      CAN TPDO1 inhibit time       |                               |
|         |    tpdo1EventTimer     | double |        s         |      0.0      |    no    |       CAN TPDO1 event timer       |                               |
|         |    tpdo2InhibitTime    | double |        s         |      0.0      |    no    |      CAN TPDO2 inhibit time       |                               |
|         |    tpdo2EventTimer     | double |        s         |      0.0      |    no    |       CAN TPDO2 event timer       |                               |
|         |     monitorPeriod      | double |        s         |      0.0      |    no    |       monitor thread period       |                               |
| driver  |      peakCurrent       | double |        A         |               |   yes    |        drive peak current         |                               |
|  motor  |           k            | double |      N*m/A       |               |   yes    |          motor constant           |                               |
| gearbox |           tr           | double |                  |               |   yes    |             reduction             |                               |
| encoder |     encoderPulses      | double |                  |               |   yes    |   encoder pulses per revolution   |                               |
|         |         ipMode         | string |                  |      pt       |    no    |         IP mode (pt, pvt)         |       only embedded PID       |
|         |       ipPeriodMs       |  int   |        ms        |      50       |    no    |             IP period             |       only embedded PID       |
|         |        enableIp        |  bool  |                  |     false     |    no    |          enable IP mode           |       only embedded PID       |
|         |       enableCsv        |  bool  |                  |     false     |    no    |          enable CSV mode          |     embedded/external PID     |
|         | initialInteractionMode | string |                  |     unkn      |    no    |  initial interaction mode vocab   | stif, comp; only external PID |
|         |       stiffness        | double |       N\*m       |      0.0      |    no    |        impedance stiffness        |       only external PID       |
|         |        damping         | double |     N\*m\*s      |      0.0      |    no    |         impedance damping         |       only external PID       |
|         |    impedanceOffset     | double |     N\*m\*s      |      0.0      |    no    |         impedance offset          |       only external PID       |
|         |      minStiffness      | double |       N\*m       |      0.0      |    no    |    minimum impedance stiffness    |       only external PID       |
|         |      maxStiffness      | double |       N\*m       |      0.0      |    no    |    maximum impedance stiffness    |       only external PID       |
|         |       minDamping       | double |     N\*m\*s      |      0.0      |    no    |     minimum impedance damping     |       only external PID       |
|         |       maxDamping       | double |     N\*m\*s      |      0.0      |    no    |     maximum impedance damping     |       only external PID       |
|         |           kp           | double |                  |      0.0      |    no    |          position PID Kp          |       only external PID       |
|         |           ki           | double |                  |      0.0      |    no    |          position PID Ki          |       only external PID       |
|         |           kd           | double |                  |      0.0      |    no    |          position PID Kd          |       only external PID       |
|         |         maxInt         | double |                  |      0.0      |    no    | position PID saturation threshold |       only external PID       |
|         |       maxOutput        | double |                  |      0.0      |    no    |   position PID maximum output     |       only external PID       |
|         |         offset         | double |                  |      0.0      |    no    |        position PID offset        |       only external PID       |
|         |         scale          | double |                  |      1.0      |    no    |        position PID scale         |       only external PID       |
|         |       stictionUp       | double |                  |      0.0      |    no    |     position PID stiction up      |       only external PID       |
|         |      stictionDown      | double |                  |      0.0      |    no    |    position PID stiction down     |       only external PID       |
|         |          kff           | double |                  |      0.0      |    no    |     position PID feed-forward     |       only external PID       |
|         |       errorLimit       | double |       deg        |      0.0      |    no    |     position PID error limit      |       only external PID       |
