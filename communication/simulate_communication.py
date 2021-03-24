import json
import random

import math
from glob import glob
import xml.etree.ElementTree as ET
import numpy as np

'''
Given time and location, get the received BSM from all vehicles at the time given 
only if this vehicle is located at [locX-75, locY+75]

To do the first part: 
1- dictionary for all BSMs: key --> vehicle, Value -> (start, end)
    a- read xml file to get start and end of each vehicle
    b- output a dictionary with a vehicle id as a key and value is the start and end tuple

2- Function that given a time, returns the list of vehicles having start, end including this time

3- For the time component: 
    a- Initialize a range [time-10, time[
    b- check dictionary for vehicles sending in this range
    c- From the dictionary get broadcasts from the vehicles IDs from previous step 
    d- Filter these vehicles based on their location

'''

def euclidean_dist(pt1, pt2):
    return np.sqrt((pt1[0]-pt2[0])*(pt1[0]-pt2[0]) + (pt1[1]-pt2[1])*(pt1[1]-pt2[1]))


class CommunicationSimulator:
    def __init__(self, scenario_path):
        traffic_filepath = "/media/bassel/Entertainment/sumo/CARLA_0.9.10/WindowsNoEditor/PythonAPI/SUMO-Carla-integration/SUMO_TrafficLog/Town01_Car50.log"
        self.vehicles_lifetime_dict = self.parseXML_forAllVehicles(traffic_filepath, 1)
        self.bsms = self.read_all_bsms(scenario_path)
        self.scenario = traffic_filepath.split("/")[-1].split("_")[1].split(".")[0]
        self.town = traffic_filepath.split("/")[-1].split("_")[0]

    def parseXML_forAllVehicles(self, xmlfile):
        tree = ET.parse(xmlfile)
        root = tree.getroot()
        ret_info = {}

        for item in root.findall('timestep'):
            collectedtime = float(item.attrib['time'])

            for edge in item.findall('vehicle'):
                collectedID = int(edge.attrib['id'])

                if collectedID in ret_info:
                    ret_info[collectedID] = [min(ret_info[collectedID], collectedtime),
                                             max(ret_info[collectedID], collectedtime)]
                else:
                    ret_info[collectedID] = collectedtime

        return ret_info

    def read_all_bsms(self, scenario_path):
        bsm_all_vehicle = glob.glob(scenario_path+"/*.txt")
        bsm_all_vehicle.sort()
        bsm_list=[]

        for bsm in bsm_all_vehicle:
            with open(bsm) as fr:
                bsm_list.append([line.strip().split("\t") for line in fr.readlines()])

        return bsm_list

    def vehicles_sending_at_time(self, time, delta=0.1):
        '''
        Function that given a time, returns the list of vehicles having start, end including this time
        '''
        vehicles_ids_included = []

        for vehicle_id, start_end in self.vehicles_lifetime_dict:
            if (time > start_end[0] and time <= start_end[1]) \
                    or ((time-delta) >= start_end[0] and (time-delta)<start_end[1]):
                if vehicle_id == 46 or vehicle_id == 48:
                    continue
                vehicles_ids_included.append(vehicle_id)

        return vehicles_ids_included

    def get_cpm_length(self, vid, at_time, delta=0.1):
        base_dir = "/media/bassel/Entertainment/sumo/CARLA_0.9.10/WindowsNoEditor/PythonAPI/SUMO-Carla-integration/CARLA_PythonCode/"
        front_file_path = base_dir + self.town + "/" + self.scenario + "/data/" + str(vid) + "/front/"
        # right_file_path = base_dir + self.town + "/" + self.scenario + "/data/" + str(vid) + "/front_Right60/"
        # left_file_path = base_dir + self.town + "/" + self.scenario + "/data/" + str(vid) + "/front_left300/"

        cpms_files = glob(front_file_path+"rgb*.txt")
        cpms_files.sort()
        length = 0

        for cpm_file in cpms_files:
            t = float(cpms_files.split("/")[-1].split("rgb_")[1].split(".")[0])

            if t >= at_time - delta and t < at_time:
                with open(cpm_file) as fr:
                    cpm_body = json.load(fr)

                length += 16 * len(cpm_body) * 2
                cpm_file = cpm_file.replace("front", "front_Right60")

                with open(cpm_file) as fr:
                    cpm_body = json.load(fr)

                length += 16 * len(cpm_body) * 2
                cpm_file = cpm_file.replace("front_Right60", "front_left300")

                with open(cpm_file) as fr:
                    cpm_body = json.load(fr)

                length += 16 * len(cpm_body) * 2

        return length

    def receive_CPM(self, at_time, at_location):
        '''
        Note that the time received shall be adjusted as follow: starting_time_in_BSM + vehicle_start_time_in_XML - 2nd time in BSM
        '''
        vehicles_ids_broadcasting = self.vehicles_sending_at_time(at_time)
        interference_messages = []
        vid_sending_cpms = []

        for vid in vehicles_ids_broadcasting:
            dist = euclidean_dist(self.bsms[vid][1:3], at_location)

            if dist <= 500:
                interference_messages.append(vid)
            if dist <= 300:
                vid_sending_cpms.append(vid)

        count_transmitters = 0
        s = 0

        for vid in interference_messages:
            cpm_len = self.get_cpm_length(vid, at_time)

            if cpm_len != 0: # or RL decides not to send
                s += cpm_len
                count_transmitters += 1

        l = count_transmitters# get average CPM length
        s /= count_transmitters# get average CPM length
        g = 6000000 #6 MB
        t = 0.1

        p = math.exp(-(l*s)/(g*t))

        # packeit received successfully
        successful_received_cpms = []

        for i in range(len(vid_sending_cpms)):
            n = random.random()
            if n > p:
                successful_received_cpms.append(vid_sending_cpms[i])

        cpms = self.get_cpms(successful_received_cpms)
        return cpms


if __name__ == '__main__':
    # scenario_path = '/media/bassel/Entertainment/sumo/CARLA_0.9.10/WindowsNoEditor/PythonAPI/SUMO-Carla-integration/CARLA_PythonCode/workplace/Car50/VehicleData_BSM'
    # receive_BSM(0, 0, 0)
    CommunicationSimulator()


