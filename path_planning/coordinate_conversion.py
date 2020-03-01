#!/usr/bin/env python

__author__ = "Elisei Shafer"

import pymap3d
import pickle
import csv
import os
import textwrap
from string import Template


class AuvPath:


    def __init__(self, lat_0, lon_0, local_path=None):
        self.lat_0 = lat_0
        self.lon_0 = lon_0
        self.local_path = local_path
        self.geodetic_path = None
        self.xml = None

    def load_pickled_path(self,file_path):

        with open(file_path, 'rb') as f:
            path = pickle.load(f)
            self.local_path = path

    def save_csv(self, robot_path, file_path):
        with open(file_path, 'w') as f:
            wr = csv.writer(f)
            for p in robot_path:
                wr.writerow(p)

    def load_csv(self, file_path):
        data = []
        with open(file_path, 'r') as f:
            data_reader = csv.reader(f, quoting=csv.QUOTE_NONNUMERIC)
            for row in data_reader:
                data.append(row)

        self.local_path = data

    def get_geodetic_path(self):
        self.geodetic_path = [pymap3d.ned2geodetic(p[0], p[1], 3, self.lat_0, self.lon_0, 0)
                              for p in self.local_path]
        return self.geodetic_path

    def create_xml(self, file_path):
        sparus_xml = SparusXml()
        if self.geodetic_path is None:
            self.get_geodetic_path()
        sparus_xml.create_full_xml(self.geodetic_path, self.local_path)
        self.xml = sparus_xml.xml
        with open(file_path, 'w') as f:
            f.write(self.xml)


class SparusXml:

    def __init__(self):
        self.xml = None
        self.waypoint_template = Template(textwrap.dedent('''
                                  <mission_step>
                                    <maneuver type="waypoint">
                                      <position>
                                        <latitude>$lat</latitude>
                                        <longitude>$lon</longitude>
                                        <z>4.0</z>
                                        <altitude_mode>False</altitude_mode>
                                      </position>
                                      <speed>0.5</speed>
                                      <tolerance>
                                        <x>2.0</x>
                                        <y>2.0</y>
                                        <z>1.0</z>
                                      </tolerance>
                                    </maneuver>
                                  </mission_step>'''))
        self.section_template = Template(textwrap.dedent('''
                                  <mission_step>
                                    <maneuver type="section">
                                      <initial_position>
                                        <latitude>$lat_0</latitude>
                                        <longitude>$lon_0</longitude>
                                        <z>4.0</z>
                                        <altitude_mode>False</altitude_mode>
                                      </initial_position>
                                      <final_position>
                                        <latitude>$lat_1</latitude>
                                        <longitude>$lon_1</longitude>
                                        <z>2.0</z>
                                        <altitude_mode>False</altitude_mode>
                                      </final_position>
                                      <speed>$speed</speed>
                                      <tolerance>
                                        <x>2.0</x>
                                        <y>2.0</y>
                                        <z>1.0</z>
                                      </tolerance>
                                    </maneuver>
                                  </mission_step>'''))

    def create_full_xml(self, robot_path, local_path):

        self.xml = '<mission>\n'
        self.xml += self.waypoint_template.substitute(lat=robot_path[0][0],
                                                      lon=robot_path[0][1])
        for i, p_1 in enumerate(robot_path):
            if i != 0:
                self.xml += self.section_template.substitute(lat_0=p_0[0],
                                                             lon_0=p_0[1],
                                                             lat_1=p_1[0],
                                                             lon_1=p_1[1],
                                                             speed=local_path[i][3])
            p_0 = p_1
        self.xml += self.waypoint_template.substitute(lat=robot_path[-1][0],
                                                      lon=robot_path[-1][1])
        self.xml += '</mission>\n'


def main(lat_0, lon_0, file_path):

    auvpath = AuvPath(lat_0,lon_0)
    auvpath.load_pickled_path(file_path + '.pkl')
    auvpath.get_geodetic_path()
    auvpath.save_csv(auvpath.geodetic_path, file_path + '.csv')
    auvpath.create_xml(file_path + '.xml')



if __name__ == '__main__':
    path_file = 'path'
    distance_obstacle = 45
    lat_0, lon_0, _ =pymap3d.ned2geodetic(-distance_obstacle, 0, 0, 32.841242, 34.973986, 0)
    main(lat_0, lon_0,path_file)
