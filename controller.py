import Sofa.Core as SC
import numpy as np

class ControlCatheter(SC.Controller):
    def __init__(self, *args, **kwargs):
        SC.Controller.__init__(self, *args, **kwargs)
        self.limit = 0.65
        self.node = kwargs["node"]
        self.beam_node = self.node["InstrumentCombined"]
        self.beam = self.node["InstrumentCombined"]["DOFs"]
        self.instrument_controller = self.node["InstrumentCombined"]["m_ircontroller"]
        self.mass = 1.0
        self.dt = self.node.dt.value
        self.attached = False
    
    def onAnimateBeginEvent(self, event):
        pos = self.beam["position"].value[-1][1]
        if pos > self.limit or self.attached:
            self.instrument_controller["speed"].value = 0.0
            if not self.attached:
                print("Create the attachment of the beam with the haptic device.")
                self.create_omni_attachment()
                self.attached = True
            else:
                self.generate_force(self.beam["position"].value[-1][:3], self.omni["position"].value[0][:3])
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