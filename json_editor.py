import os
import json
import numpy as np
import shutil
import argparse

class Json_Editor():
    def __init__(self, nagent):
        self.nagent = nagent

    def reset_json(self, filename='sample.json'):
        with open(filename,'r+') as file:
            # First we load existing data into a dict.
            file_data = json.load(file)
            file_data['Vehicles'] = {}
            # Sets file's current position at offset.
            file.seek(0)
            # convert back to json.
            json.dump(file_data, file, indent = 4)
            file.truncate()     # remove remaining part
            file.close()
                
    def modify(self):
        self.reset_json()
        if os.path.exists('sample.json'):
            with open('sample.json', 'r+') as file:

                #First we load existing data into a dict.
                file_data = json.load(file)
                drone = {}
                x_initial=-30
                y_initial=30
                positions=[]
                pose_x=x_initial
                pose_y=y_initial
                for i in range(self.nagent):
                    pose_x=x_initial-4*int(i/10)
                    pose_y=y_initial+4*int(i-int(i/10)*10)
                    positions.append([pose_x,pose_y])

                for n in range(self.nagent):
                    drone['Drone'+str(n+1)] = {"VehicleType": "SimpleFlight", "X": positions[n][0], "Y": positions[n][1], "Z": 0}

                # Join new_data with file_data
                file_data['Vehicles'].update(drone)

                #file_data["Vehicles"]["Drone1"].update(drone)

                # Sets file's current position at offset.
                # convert back to json.
                file.seek(0)
                json.dump(file_data, file, indent = 4)
                file.close()

        os.system('cp sample.json ~/Documents/AirSim/settings.json')




if __name__ == "__main__":
    nagent = 4
    js_modifier = Json_Editor(nagent)
    js_modifier.modify()
    