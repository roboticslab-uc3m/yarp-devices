| Group |     Parameter     |    Type     |  Units  | Default Value | Required |              Description               |        Notes        |
|:-----:|:-----------------:|:-----------:|:-------:|:-------------:|:--------:|:--------------------------------------:|:-------------------:|
|       |       canId       |     int     |         |       0       |    no    |               CAN bus ID               |        1-127        |
|       |        name       |   string    |         |               |    no    |              sensor name               |                     |
|       |      filter       |   double    |   Hz    |      0.0      |    no    |  cutoff frequency for low-pass filter  |     0.0-655.35      |
|       |    ackTimeout     |   double    |    s    |     0.25      |    no    |        CAN acknowledge timeout         |                     |
|       |    fullScales     | vector<int> | N, daNm |               |    no    |       full scales for each axis        |    3\*N, 3\*daNm    |
|       |    asyncPeriod    |   double    |    s    |      0.0      |    no    | period of asynchronous publishing mode |    0.0: disabled    |
|       |   monitorPeriod   |   double    |    s    |      0.1      |    no    |         monitor thread period          |                     |
|       | diagnosticsPeriod |   double    |    s    |      0.0      |    no    |           diagnostics period           | less than 8 seconds |
