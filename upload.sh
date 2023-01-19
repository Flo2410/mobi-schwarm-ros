#!/bin/bash
# scp -r $PWD mobi_theta:/home/mobi/Desktop/hye/

#scp -r ./**/ mobi_theta:/home/mobi/catkin_ws/src
# ssh mobi_theta "cd /home/mobi/catkin_ws/ && source /home/mobi/catkin_ws/devel/setup.bash && catkin_make"

#host = ros
#remote_path = "/home/florian/catkin_ws"

host=mobi_theta
remote_path="/home/mobi/catkin_ws"


# ssh $host "screen -X -S pozyx-translator kill"
# ssh $host "screen -X -S pozyx-fake kill"
# ssh $host "screen -X -S web-backend kill"
# ssh $host "screen -X -S mobi-imu kill"
scp -r ./**/ $host:$remote_path/src
ssh $host "cd $remote_path/ && source $remote_path/devel/setup.bash && catkin_make"
# ssh $host "source $remote_path/devel/setup.bash && screen -S mobi-imu -d -m rosrun mobi-imu theta-imu.py"
# ssh $host "source $remote_path/devel/setup.bash && screen -S pozyx-fake -d -m rosrun pozyx-fake pozyx-fake.py"
# ssh $host "source $remote_path/devel/setup.bash && screen -S pozyx-translator -d -m rosrun pozyx-translator pozyx-mqtt.py"
# ssh $host "source $remote_path/devel/setup.bash && screen -S web-backend -d -m rosrun web-backend web-backend.py"