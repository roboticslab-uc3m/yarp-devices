| Group | Parameter |      Type      |  Units  | Default Value | Required |          Description           | Notes |
|:-----:|:---------:|:--------------:|:-------:|:-------------:|:--------:|:------------------------------:|:-----:|
|       |  joints   |      int       |         |       0       |    no    |   number of controlled axes    |       |
|       |   block   |      bool      |         |     false     |    no    |     commands should block      |       |
|       |   home    |     double     |   deg   |      0.0      |    no    |     zero position (global)     |       |
|       |  homeVel  |     double     |  deg/s  |      0.0      |    no    |     zero velocity (global)     |       |
|       |  homeAcc  |     double     | deg/s^2 |      0.0      |    no    |   zero acceleration (global)   |       |
|       |   park    |     double     |   deg   |      0.0      |    no    |     park position (global)     |       |
|       |  parkVel  |     double     |  deg/s  |      0.0      |    no    |     park velocity (global)     |       |
|       |  parkAcc  |     double     | deg/s^2 |      0.0      |    no    |   park acceleration (global)   |       |
|       |   homes   | vector<double> |   deg   |               |    no    |   zero position (individual)   |       |
|       |  homeVels | vector<double> |  deg/s  |               |    no    |   zero velocity (individual)   |       |
|       |  homeAccs | vector<double> | deg/s^2 |               |    no    | zero acceleration (individual) |       |
|       |   parks   | vector<double> |   deg   |               |    no    |   park position (individual)   |       |
|       |  parkVels | vector<double> |  deg/s  |               |    no    |   park velocity (individual)   |       |
|       |  parkAccs | vector<double> | deg/s^2 |               |    no    | park acceleration (individual) |       |
