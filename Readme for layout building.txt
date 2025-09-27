For the touge/racing setup

- note that all objects created with builder has dmg enabled
- Needs to have 4 starting positions, use obj, control -> Start Position, and start position number from 1 - 4. 1 and 2 are used for forward, and 3 and 4 are used for backwards
- To finish, put a marshal -> insim checkpoint -> finish line. Put one for each for forward and backwards.
- for mini checkpoints, put marshal -> insim checkpoint -> 1st checkpoint, every 40m or so. Insim will count these up.
- Use marshal -> insim circle -> circle index: 0 for 1st split time, index: 1 for 2nd split time, etc, to 4 total (not sure if we can go above, need to test)

You have to have 4 checkpoints in order for the finish line to work!