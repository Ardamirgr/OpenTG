# ECU Operating Mode Flowchart

```mermaid
flowchart TD

    A[ECU POWER ON]
    B[SELF TEST]
    C[SELF TEST OK<br/>Stored Memory]
    D[SELF TEST FAIL<br/>Wait CUTOFF]
    E[NEXT CUT-IN<br/>Perform SELF TEST AGAIN]
    F[START TIMER<br/>45 min RUN]
    G{45 min REACHED?}
    H[NORMAL MODE<br/>Enabled]
    I[< 45 min MODE<br/>Running]
    J[CUTOFF EVENT]
    K{CHECK OPERATING MODE}
    L[RESET CHECK<br/>Wait seconds]
    M[ITD TIMER<br/>2:30 + 30rnd]
    N{RESET CHECK OK?}
    O[CUT-IN<br/>after 6 sec]

    A --> B

    B -->|PASS| C
    B -->|FAIL| D

    C --> F
    D --> E
    E --> B

    F --> G

    G -->|YES| H
    G -->|NO| I

    H --> J
    I --> J

    J --> K

    K -->|<45 min| L
    K -->|NORMAL MODE| M

    L --> N

    N -->|YES| O
    N -->|NO| M

    O --> F
```
