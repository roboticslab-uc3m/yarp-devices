| Group |    Parameter    |    Type     | Units | Default Value | Required |               Description              | Notes |
|:-----:|:----------------|:-----------:|:-----:|:-------------:|:--------:|:--------------------------------------:|:-----:|
|       |      port       |   string    |       |  /dev/pcan0   |    no    |            CAN device path             |       |
|       |     bitrate     |     int     |  bps  |    1000000    |    no    |              CAN bitrate               |       |
|       |  blockingMode   |    bool     |       |     true      |    no    |         blocking mode enabled          |       |
|       | allowPermissive |    bool     |       |     false     |    no    |       read/write permissive mode       |       |
|       |   rxTimeoutMs   |     int     |  ms   |       1       |    no    |               RX timeout               |       |
|       |   txTimeoutMs   |     int     |  ms   |       0       |    no    |               TX timeout               |       |
|       | preserveFilters |    bool     |       |     true      |    no    | don't clear acceptance filters on init |       |
|       |   filteredIds   | vector<int> |       |               |    no    |           filtered node IDs            |       |
