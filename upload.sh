#!/bin/bash
# scp -r $PWD mobi_theta:/home/mobi/Desktop/hye/

#scp -r ./**/ mobi_theta:/home/mobi/catkin_ws/src
# ssh mobi_theta "cd /home/mobi/catkin_ws/ && source /home/mobi/catkin_ws/devel/setup.bash && catkin_make"



ssh ros "screen -X -S pozyx-fake kill"
ssh ros "screen -X -S web-backend kill"
scp -r ./**/ ros:/home/florian/catkin_ws/src
ssh ros "cd /home/florian/catkin_ws/ && source /home/florian/catkin_ws/devel/setup.bash && catkin_make"
ssh ros "source /home/florian/catkin_ws/devel/setup.bash && screen -S pozyx-fake -d -m rosrun pozyx-fake pozyx-fake.py"
ssh ros "source /home/florian/catkin_ws/devel/setup.bash && screen -S web-backend -d -m rosrun web-backend web-backend.py"