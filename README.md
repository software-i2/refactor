# refactored code for reach arm driver
previous code was too messy :<

### architecture :  
| package | use |
| --- | --- |
| n_driver | driver code ros nodes |
| n_kine | fk and ik |
| n_check | check for reach-ability and collisions |
| n_config | contains arm.yaml, the only config file |
| n_ctrl | controller to plan and execute plan |
| n_desc | urdf and meshes |
| n_task | commands subtasks for grabbing |
| n_cloud | process ply and json files |


### workflow :
```bash
./scripts/build.sh
```

```bash
./scripts/shell.sh
```

```bash
rosrun n_pcloud scene.py --scene 000XXX --at "X X X" --out data/field.bin --candidates data/candidates_000XXX.txt
```

```bash
roslaunch tools/sim.launch scene:=000XXX
```


```bash
tools/classify/run.py 000XXX
```

```bash
tools/arm.sh go
```

<br><br>
there are 2 main workflows happening:
1. point cloud processing
2. motion planning
<br>

### point cloud processing :
1. take in a ply and a json file
2. take each pose's voxel and exempted its neighbours (5mm sphere)
3. for the left over points, conduct flying-pixel filtering, keep pixel only if 5 of its 8 neighbour are within 4mm
4. voxelise at 5mm, voxels are labelled free, unknown, obstacle
5. handle poses and exempted regions are labelled as target
6. run the checks on the handle poses, remove points out of reach and below floor
7. leftover points are stored in candidates.txt
8. (handle/rope filtering) using distance from backdrop
9. keeps handles and overwrites candidates.txt

### motion planning :
1. load candidates.txt
2. for each point, and 5mm apart points along the recommended approaching heading, conduct ik
3. remove handle poses that are unreachable, too off from recommended approaching heading, would cause the arm to hit the floor, 80mm standoff not achievable, violates joint limits, hit obstacles.
4. if it passes, calculate cost of each path and choose cheapest candidate


### task execution :
1. open jaw
2. move to standoff
3. advance to handle
4. close jaw
5. trace back to base
