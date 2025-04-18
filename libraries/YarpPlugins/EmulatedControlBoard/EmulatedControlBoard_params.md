| Group |    Parameter     |      Type      |    Units     |   Default Value    | Required |             Description             |     Notes      |
|:-----:|:----------------:|:--------------:|:------------:|:------------------:|:--------:|:-----------------------------------:|:--------------:|
|       |       axes       |      int       |              |         5          |    no    |      number of axes to control      |                |
|       |      jmcMs       |      int       |      ms      |         20         |    no    |    period of JMC periodic thread    |                |
|       |       mode       |     string     |              |                    |   yes    |            control mode             | pos, vel, posd |
|       |    genInitPos    |     double     |   m or deg   |        0.0         |    no    |  general initialization positions   |                |
|       |   genJointTol    |     double     |   m or deg   |        0.25        |    no    |      general joint tolerances       |                |
|       |   genMaxLimit    |     double     |   m or deg   |       180.0        |    no    |         general max limits          |                |
|       |   genMinLimit    |     double     |   m or deg   |       -180.0       |    no    |         general min limits          |                |
|       |   genRefSpeed    |     double     | m/s or deg/s |        7.5         |    no    |          general ref speed          |                |
|       | genEncRawExposed |     double     |              | 0.0174532925199433 |    no    |        general EncRawExposed        |                |
|       | genVelRawExposed |     double     |              | 0.0174532925199433 |    no    |        general VelRawExposed        |                |
|       |     initPoss     | vector<double> |   m or deg   |                    |    no    | individual initialization positions |                |
|       |    jointTols     | vector<double> |   m or deg   |                    |    no    |    individual joint tolerances      |                |
|       |    maxLimits     | vector<double> |   m or deg   |                    |    no    |        individual max limits        |                |
|       |    minLimits     | vector<double> |   m or deg   |                    |    no    |        individual min limits        |                |
|       |    refSpeeds     | vector<double> | m/s or deg/s |                    |    no    |        individual ref speeds        |                |
|       |  encRawExposeds  | vector<double> |              |                    |    no    |      individual EncRawExposed       |                |
|       |  velRawExposeds  | vector<double> |              |                    |    no    |      individual VelRawExposed       |                |
