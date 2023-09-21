import Sofa.Core as SC

class ControlCatheter(SC.Controller):
    def __init__(self, *args, **kwargs):
        SC.Controller.__init__(self, *args, **kwargs)
        self.limit = 0.15
        node = kwargs["node"]
        self.beam = node["InstrumentCombined"]["DOFs"]
        self.init_pos = self.beam["position"].value[-1][1]
        self.instrument_controller = node["InstrumentCombined"]["m_ircontroller"]
    
    def onAnimateBeginEvent(self, event):
        pos = self.beam["position"].value[-1][1]
        if pos > self.limit:
            print("Catheter stopped!")
            self.instrument_controller["speed"].value = 0.0
        pass