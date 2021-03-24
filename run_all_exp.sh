conda activate macad

CARLA_PATH=/home/bassel/repos/CARLA_0.9.5
CLIENT_PATH=/media/bassel/Entertainment/sumo/CARLA_0.9.10/WindowsNoEditor/PythonAPI/SUMO-Carla-integration/CARLA_PythonCode
BSM_DIR=/media/bassel/Entertainment/sumo/CARLA_0.9.10/WindowsNoEditor/PythonAPI/SUMO-Carla-integration/CARLA_PythonCode/workplace/Car50/VehicleData_BSM

SCENARIOS=("Town01_Car50.log"
#            "Town01_Car50_2.log"
#            "Town01_Car50_3.log"
#            "Town01_Car50_4.log"
#            "Town01_Car100_2.log"
#            "Town01_Car100_3.log"
#            "Town01_Car100_4.log"
#            "Town01_Car100.log"
#            "Town01_Car150_2.log"
#            "Town01_Car150_3.log"
#            "Town01_Car150_4.log"
#            "Town01_Car150.log"
#            "Town01_Car200_2.log"
#            "Town01_Car200_3.log"
#            "Town01_Car200_4.log"
#            "Town01_Car200.log")
            "Town02_Car50.log")
#            "Town02_Car50_2.log"
#            "Town02_Car50_3.log"
#            "Town02_Car50_4.log"
#            "Town02_Car100_2.log"
#            "Town02_Car100_3.log"
#            "Town02_Car100_4.log"
#            "Town02_Car100.log"
#            "Town02_Car150_1.log"
#            "Town02_Car150_2.log"
#            "Town02_Car150_3.log"
#            "Town02_Car150_4.log"
#            "Town02_Car150.log"
#            "Town02_Car200_2.log"
#            "Town02_Car200.log")

initial_dir=$PWD

for SCENARIO in ${SCENARIOS[*]}; do
  for i in {0..49}
  do
      echo "Run for Cam $i"
      sh $CARLA_PATH/CarlaUE4.sh Town01 -fps=10 &
      sleep 10

      echo $SCENARIO
      python $CLIENT_PATH/car_generator_forTown01.py -scenario $SCENARIO -id $i

      pkill CarlaUE4

      echo "Object detections for car $i"
      cd /media/bassel/Entertainment/sumo/CARLA_0.9.10/WindowsNoEditor/PythonAPI/SUMO-Carla-integration/darknet
      python $CLIENT_PATH/../darknet/detect_objects.py -p $BSM_DIR/BSM_$i.txt
      cd $initial_dir

      echo "removing unneeded files"
      rm /media/bassel/Entertainment/sumo/CARLA_0.9.10/WindowsNoEditor/PythonAPI/SUMO-Carla-integration/CARLA_PythonCode/Town01/Car50/data/$i/front/*.png
      rm /media/bassel/Entertainment/sumo/CARLA_0.9.10/WindowsNoEditor/PythonAPI/SUMO-Carla-integration/CARLA_PythonCode/Town01/Car50/data/$i/frontD/*.png
      rm /media/bassel/Entertainment/sumo/CARLA_0.9.10/WindowsNoEditor/PythonAPI/SUMO-Carla-integration/CARLA_PythonCode/Town01/Car50/data/$i/front_Right60/*.png
      rm /media/bassel/Entertainment/sumo/CARLA_0.9.10/WindowsNoEditor/PythonAPI/SUMO-Carla-integration/CARLA_PythonCode/Town01/Car50/data/$i/front_Right60D/*.png
      rm /media/bassel/Entertainment/sumo/CARLA_0.9.10/WindowsNoEditor/PythonAPI/SUMO-Carla-integration/CARLA_PythonCode/Town01/Car50/data/$i/front_Left300/*.png
      rm /media/bassel/Entertainment/sumo/CARLA_0.9.10/WindowsNoEditor/PythonAPI/SUMO-Carla-integration/CARLA_PythonCode/Town01/Car50/data/$i/front_Left300D/*.png

      sleep 5
  done

  echo "Scenario $SCENARIO Ended!"
done



#sh /home/bassel/repos/CARLA_0.9.5/CarlaUE4.sh Town01
#python /media/bassel/Entertainment/sumo/CARLA_0.9.10/WindowsNoEditor/PythonAPI/SUMO-Carla-integration/CARLA_PythonCode/car_generator_forTown01.py 1
#
#python /media/bassel/Entertainment/sumo/CARLA_0.9.10/WindowsNoEditor/PythonAPI/SUMO-Carla-integration/CARLA_PythonCode/car_generator_forTown01.py 0

