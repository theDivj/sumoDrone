"""  Static class to support inter-object communication """
class GlobalClasses:
    """ globals to support communication between Drones,EVs with control centre and simulation """
    cc = None   # ControlCentre
    ss = None   # Simulation
    ch = None   # ChargeHubs

    def __init__(self,cc,ss,ch):
        GlobalClasses.cc = cc
        GlobalClasses.ss = ss
        GlobalClasses.ch = ch
