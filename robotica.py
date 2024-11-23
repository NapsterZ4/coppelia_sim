"""
robotica.py

Provides the communication between CoppeliaSim robotics simulator and
external Python applications via the ZeroMQ remote API.

Copyright (C) 2024 Javier de Lope

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

import numpy as np
import cv2
import time

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


class Coppelia:

    def __init__(self):
        self.default_idle_fps = None
        print('*** connecting to coppeliasim')
        client = RemoteAPIClient()
        self.sim = client.getObject('sim')

    def start_simulation(self):
        self.default_idle_fps = self.sim.getInt32Param(
            self.sim.intparam_idle_fps
        )
        self.sim.setInt32Param(self.sim.intparam_idle_fps, 0)
        self.sim.startSimulation()

    def stop_simulation(self):
        self.sim.stopSimulation()
        while self.sim.getSimulationState() != self.sim.simulation_stopped:
            time.sleep(0.1)
        self.sim.setInt32Param(self.sim.intparam_idle_fps,
                               self.default_idle_fps)
        print('*** done')

    def is_running(self):
        return self.sim.getSimulationState() != self.sim.simulation_stopped


class P3DX:
    num_sonar = 16
    sonar_max = 1.0

    def __init__(self, sim, robot_id, use_camera=False, use_lidar=False):
        self.sim = sim
        print('*** getting handles', robot_id)
        self.left_motor = self.sim.getObject(f'/{robot_id}/leftMotor')
        self.right_motor = self.sim.getObject(f'/{robot_id}/rightMotor')
        self.sonar = []
        for i in range(self.num_sonar):
            self.sonar.append(self.sim.getObject(f'/{robot_id}/'
                                                 f'ultrasonicSensor[{i}]'))
        if use_camera:
            self.camera = self.sim.getObject(f'/{robot_id}/camera')
        if use_lidar:
            self.lidar = self.sim.getObject(f'/{robot_id}/lidar')

    def get_sonar(self):
        readings = []
        for i in range(self.num_sonar):
            res, dist, _, _, _ = self.sim.readProximitySensor(self.sonar[i])
            readings.append(dist if res == 1 else self.sonar_max)
        return readings

    def get_image(self):
        img, resX, resY = self.sim.getVisionSensorCharImage(self.camera)
        img = np.frombuffer(img, dtype=np.uint8).reshape(resY, resX, 3)
        img = cv2.flip(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), 0)
        return img

    def get_lidar(self):
        data = self.sim.getStringSignal('PioneerP3dxLidarData')
        if data is None:
            return []
        else:
            return self.sim.unpackFloatTable(data)

    def set_speed(self, left_speed, right_speed):
        self.sim.setJointTargetVelocity(self.left_motor, left_speed)
        self.sim.setJointTargetVelocity(self.right_motor, right_speed)
