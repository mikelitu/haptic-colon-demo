import Sofa.Core as SC
import Sofa.Simulation as SS
import numpy as np
import re
from utils import split_and_filter
import sys
import config
import random

experiment_forces = {}

class ControlCatheter(SC.Controller):
    def __init__(self, *args, **kwargs):
        SC.Controller.__init__(self, *args, **kwargs)
        self.limit = 0.53
        self.node = kwargs["node"]

        self.beam_node = self.node["InstrumentCombined"]
        self.beam = self.node["InstrumentCombined"]["DOFs"]
        self.instrument_controller = self.node["InstrumentCombined"]["m_ircontroller"]
        self.device = self.node["GeomagicDevice"]
        self.mass = 3.0
        self.dt = self.node.dt.value
        self.attached = False
        self.button_state = self.device["button1"].value
        self.extending = False
        self.force_sensor = ForceSensor(self.node["Colon"]["Collision"]["colon_col"])
        self.exp_position = [self.node["Colon"]["Collision"]["colon_col"]["position"].value[4102],
                             self.node["Colon"]["Collision"]["colon_col"]["position"].value[4120],
                             self.node["Colon"]["Collision"]["colon_col"]["position"].value[4194]]
        random.shuffle(self.exp_position)
        self.exp_number = 0
        self.start_exp = False
    
    def onAnimateBeginEvent(self, event):

        pos = self.beam["position"].value[-1][1]
        exp_position = self.exp_position[self.exp_number]

        if self.button_state != self.device["button1"].value and pos < self.limit:
            self.button_state = self.device["button1"].value
            if self.button_state:
                self.instrument_controller["speed"].value = 0.6
                self.instrument_controller.init()
            else:
                self.instrument_controller["speed"].value = 0.0
            
        if pos > self.limit or self.attached:
            self.instrument_controller["speed"].value = 0.0
            if not self.attached:
                self.create_omni_attachment()
                self.attached = True
                self.draw_objective(exp_position, [0.3, 0.3, 1.0, 1.0])
            else:
                self.generate_force(self.beam["position"].value[-1][:3], self.omni["position"].value[0][:3])
                if np.linalg.norm(self.beam["position"].value[-1][:3] - exp_position) < 0.1 and not self.start_exp:
                    self.exp_time = 0
                    self.start_exp = True
                    self.exp_forces = []
                    self.draw_objective(exp_position, [0.1, 0.1, 1.0, 1.0])
                
                contact_force = self.force_sensor.step()

        
        if self.start_exp:
            if self.exp_time < 1.0:
                self.exp_time += self.dt
                contact_force = self.force_sensor.step()
                self.exp_forces.append(contact_force)
                if contact_force >= 1.5 and contact_force < 5.0:
                    self.draw_objective(exp_position, [0.1, 1.0, 0.1, 1.0])
                elif contact_force >= 5.0:
                    self.draw_objective(exp_position, "yellow")
                else:
                    self.draw_objective(exp_position, [0.1, 0.1, 1.0, 1.0])

            else:
                config.experiment_forces.append(self.exp_forces)
                self.exp_number += 1
                if self.exp_number == len(self.exp_position):
                    config.done = True
                else:
                    self.draw_objective(self.exp_position[self.exp_number], [0.3, 0.3, 1.0, 1.0])
                    self.start_exp = False
                
        pass
    
    
    def create_omni_attachment(self):
        self.node["Instrument"]["Collision"].activated = True
        self.node["Instrument"]["Collision"].init()
        self.omni = self.node["Instrument"]["instrumentState"]
    
    def generate_force(self, cur_pos, des_pos):
        force = self._calculate_force(cur_pos, des_pos)
        with self.beam_node["force"].forces.writeableArray() as fB:
            fB[0] = [force[0], force[1], force[2], 0., 0., 0.]
    
    def _calculate_force(self, cur_pos, des_pos):
        diff_pos = np.array(des_pos - cur_pos)
        force = self.mass * (diff_pos/(self.dt)**2) 
        return force / np.linalg.norm(force)
    
    def draw_objective(self, position, color):
        if hasattr(self, "objective"):
            self.node.removeChild(self.objective)

        self.objective = self.node.addChild("Objective")
        self.objective.addObject("MeshOBJLoader", name="sphere", filename="mesh/sphere.obj")
        self.objective.addObject("OglModel", name="Visual", translation=position,  src="@sphere", scale=0.065, color=color)
        self.objective.init()
        SS.initVisual(self.objective)

class ForceSensor(object):
    def __init__(self, MO, numberDOFs = 3):
        self.MO = MO
        self.numberNodes = len(MO["position"].value)
        self.number_dofs = numberDOFs
        print("Init collision monitor")
    
    def get_forces(self):

        self.sortedCollisionMatrix = [np.array([0, 0, 0]) for _ in range(self.numberNodes)]

        self.collisionMatrix = self.MO.constraint.value.splitlines()
        self.collisionMatrix = [split_and_filter(line) for line in self.collisionMatrix]
            
        for i in range(len(self.collisionMatrix)):
            n_contacts = len(self.collisionMatrix[i]) // 6
            for j in range(n_contacts):
                limit = 6 * j
                contact_id = int(self.collisionMatrix[i][limit + 1])
                self.sortedCollisionMatrix[contact_id] = self.sortedCollisionMatrix[contact_id] + np.array([float(self.collisionMatrix[i][limit + 3 + a]) for a in range(self.number_dofs)])
        

        return self.sortedCollisionMatrix

    def step(self):
        col_mat = self.get_forces()
        return 0.6 * np.linalg.norm(col_mat)
        
    
    