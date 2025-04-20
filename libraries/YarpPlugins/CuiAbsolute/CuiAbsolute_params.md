| Group | Parameter  |  Type  | Units | Default Value | Required |             Description              |   Notes    |
|:-----:|:----------:|:------:|:-----:|:-------------:|:--------:|:------------------------------------:|:----------:|
|       |   canId    |  int   |       |               |    yes   |              CAN bus ID              |   1-127    |
|       |  reverse   |  bool  |       |               |    yes   |           reverse counting           |            |
|       |  timeout   | double |   s   |     0.25      |    no    |         acquisition timeout          |            |
|       | maxRetries |  int   |       |      10       |    no    | maximum number of retries on timeout |            |
|       |    mode    | string |       |               |    yes   |             publish mode             | push, pull |
|       | pushDelay  |  int   |       |       0       |    no    |           push mode delay            |   0-255    |
