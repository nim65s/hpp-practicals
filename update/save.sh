#!/bin/bash

cd $DEVEL_DIR/src/hpp-practicals/script/
mv motion_planner.py ../../motion_planner_g1.py
git checkout motion_planner.py
mv rrt.py ../../rrt_g1.py
git checkout rrt.py


cd $DEVEL_DIR/src/hpp-practicals/script/ik
mv ik.py ../../ik_g1.py
git checkout ik.py


