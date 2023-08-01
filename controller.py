import Sofa.Core as SC

class MoveSphere(SC.Controller):
    def __init__(self, *args, **kwargs):
        SC.Controller.__init__(self, *args, **kwargs)
        self.sphere = kwargs["sphere"]
    
    def onAnimateBeginEvent(self, event):
        print(self.sphere["position"].value)