"""  Static class to support inter-object communication """
import random


class GlobalClasses:
    """ globals to support communication between Drones, EVs with control centre and simulation """
    cc = None   # ControlCentre
    ss = None   # Simulation
    ch = None   # ChargeHubs

    modelRendezvous = True  # whether we estimate a rendezvous point for drone/ev
    onlyChargeOnce = True   # whether we are allowed to charge EVs more than once in a simulation
    chargePrint = False     # Whether to print a charging log

    chargeLog = None        # Filename for the charging log
    dronePrint = False      # Whether to print a drone activity log
    droneLog = None         # Filename for the drone log

    droneKmPerHr = 60.0     # default drone speed - to allow command line override
    useRandom = False       # whether to generate 'random' charge requests

    def __init__(self, cc, ss, ch):
        GlobalClasses.cc = cc
        GlobalClasses.ss = ss
        GlobalClasses.ch = ch

    @classmethod
    def getDroneSpeed(cls):
        """getter for droneKmPerHr"""
        return GlobalClasses.droneKmPerHr

    @classmethod
    def getRandom(cls):
        """return a random no!"""
        return random.random()

    @classmethod
    def setGlobals(cls, droneKmPerHr, randomSeed, droneLog, chargeLog, onlyChargeOnce, modelRendezvous):
        """initialise globals used across drone,ev,controlcentre"""
        GlobalClasses.droneKmPerHr = droneKmPerHr
        GlobalClasses.onlyChargeOnce = onlyChargeOnce
        GlobalClasses.modelRendezvous = modelRendezvous

        if randomSeed != 0:
            random.seed(randomSeed)
            GlobalClasses.useRandom = True

        if droneLog is not None:
            GlobalClasses.droneLog = droneLog
            GlobalClasses.dronePrint = True

        if chargeLog is not None:
            GlobalClasses.chargeLog = chargeLog
            GlobalClasses.chargePrint = True

    @classmethod
    def usingRandom(cls):
        """returns True if we are using random nos, False otherwise"""
        return GlobalClasses.useRandom
