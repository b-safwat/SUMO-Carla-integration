conda activate macad
CARLA_PATH=/home/bassel/repos/CARLA_0.9.5
CLIENT_PATH=/media/bassel/Entertainment/sumo/CARLA_0.9.10/WindowsNoEditor/PythonAPI/SUMO-Carla-integration/CARLA_PythonCode
#!/bin/bash
for i in {0..3}
do
    echo "Run for Cam $i"
    sh $CARLA_PATH/CarlaUE4.sh Town01 &
    sleep 10

    python $CLIENT_PATH/car_generator_forTown01.py -id $i

    pkill CarlaUE4
    sleep 5
done


#sh /home/bassel/repos/CARLA_0.9.5/CarlaUE4.sh Town01
#python /media/bassel/Entertainment/sumo/CARLA_0.9.10/WindowsNoEditor/PythonAPI/SUMO-Carla-integration/CARLA_PythonCode/car_generator_forTown01.py 1
#
#python /media/bassel/Entertainment/sumo/CARLA_0.9.10/WindowsNoEditor/PythonAPI/SUMO-Carla-integration/CARLA_PythonCode/car_generator_forTown01.py 0

