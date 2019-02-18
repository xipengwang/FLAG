# FLAG: Feature-based Localization between Air and Ground

===================================================
Copyright APRIL Lab.

https://april.eecs.umich.edu/

[FLAG: Feature-based Localization between Air and Ground](https://april.eecs.umich.edu/papers/details.php?name=wang2017icra)

This souce code contains lidar-based FLAG. It includes graph-based localziation method with more robust data association (pairwise consistency check and max-mixture model).

[Video](https://april.eecs.umich.edu/public/users/xipengw/videos/FLAG2.mp4)

--------------------
# Build the code
    install LCM: https://lcm-proj.github.io/
    git clone https://github.com/xipengwang/FLAG
    cd FLAG
    make

# Map generator
     ./FLAG-map-generator -f ../resc/bbb_floorplan.pnm -m ../resc/mission-gen.config [--out-file landmarks.csv]
     
# Lidar-to-corners
     ./velodyne-to-corner [--lidar-channel]
     
# Localization
     ./FLAG-pf -f ../resc/bbb_floorplan.pnm -m ../resc/mission-gen.config -i landmarks.csv
     ./FLAG-graph -f ../resc/bbb_floorplan.pnm -m ../resc/mission-gen.config -i landmarks.csv
