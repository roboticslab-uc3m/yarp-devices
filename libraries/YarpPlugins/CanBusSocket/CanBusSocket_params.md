| Group |      Parameter      |    Type     | Units | Default Value | Required |                Description                 | Notes |
|:-----:|:--------------------|:-----------:|:-----:|:-------------:|:--------:|:------------------------------------------:|:-----:|
|       |        port         |   string    |       |     can0      |    no    |            CAN socket interface            |       |
|       |       bitrate       |     int     |  bps  |       0       |    no    |                CAN bitrate                 |       |
|       |    blockingMode     |    bool     |       |     true      |    no    |           blocking mode enabled            |       |
|       |   allowPermissive   |    bool     |       |     false     |    no    |         read/write permissive mode         |       |
|       |     rxTimeoutMs     |     int     |  ms   |       1       |    no    |                 RX timeout                 |       |
|       |     txTimeoutMs     |     int     |  ms   |       0       |    no    |                 TX timeout                 |       |
|       | filterFunctionCodes |    bool     |       |     true      |    no    | filter mask ignores CANopen function codes |       |
|       |     filteredIds     | vector<int> |       |               |    no    |             filtered node IDs              |       |
