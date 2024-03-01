"""  Static class to support inter-object communication """
import random
class GlobalClasses:
    """ globals to support communication between Drones,EVs with control centre and simulation """
    cc = None   # ControlCentre
    ss = None   # Simulation
    ch = None   # ChargeHubs

    useRandom = False
    droneKmPerHr = 60.0

    def __init__(self,cc,ss,ch,randomSeed,droneKmPerHr):
        GlobalClasses.cc = cc
        GlobalClasses.ss = ss
        GlobalClasses.ch = ch

        if randomSeed != 0:
            random.seed(randomSeed)
            GlobalClasses.useRandom = True

        GlobalClasses.droneKmPerHr = droneKmPerHr

    @classmethod
    def getDroneSpeed(cls):
        """getter for droneKmPerHr"""
        return GlobalClasses.droneKmPerHr

    @classmethod
    def getRandom(cls):
        """return a random no"""
        return random.random()

    @classmethod
    def usingRandom(cls):
        """returns True if we are using random nos, False otherwise"""
        return GlobalClasses.useRandom
