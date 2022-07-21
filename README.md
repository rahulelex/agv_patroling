# AGV patroling
## Use case of this script
This python node (agv_patrol.py) can be used to patrol robot from starting point to sent goal point.

## Prerequisite
Robot should be in state that it can navigate to any goal position.

## To start patroling user need to follow two steps.
1. Run agv-patrol.launch file and give agv name and map name as an argument.
2. Then publish goal position on topic /patrol to start patroling.

## Additional feature
You can publish another goal point again on topic /patrol. Then after completing complete round, robot will start patroling from starting position to newly goal point provided.

## TODO
- Add a feature to partrol multiple locations from starting position to the multiple locations provided.
