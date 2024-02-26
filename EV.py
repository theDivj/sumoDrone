"""Electric Vehicle classes"""
from enum import Enum
import traci

from GlobalClasses import GlobalClasses as GG

class EV:
    """Class shadowing EVs in the simulation ie SUMO controls the drone movement - basically just a state model
        - note myPosition is only set (ie valid) in states where we are interacting with a drone
            also the -e option puts a limit on the no of EVs that are shadowed
    """
    class EVState(Enum):
        """enumeration of EV state model """
        DRIVING = 1
        CHARGEREQUESTED = 2
        WAITINGFORRENDEZVOUS = 3
        WAITINGFORDRONE = 4
        CHARGINGFROMDRONE = 5
        LEFTSIMULATION = 6
        NULL = 7

    # thresholds to trigger and stop vehicle charging  - unrealistic levels to facilitate demo - note sumo starts EVs with actual = 50% maximum battery capacity
    #    realistic levels might be computed from the distance the vehicle is expected to travel, the remaining charge and some contingency of remaining charge?
    chargeNeededThreshold = 30000.   # 30KW
    chargeDoneThreshold = 32000.     # 32KW

    # average distance vehicle will travel per Wh
    kmPerWh = 6.5/1000.   #  very general average used to compute vehicle range

    evCount = 0         # count of EVs
    evChargeSteps = 0   # total steps when EVs were charging
    evChargeGap = 0.0   # total charge gap
    evChargeCount = 0   # total no of full charges

    def __init__(self,evID):
        self.myID = evID
        self.myState = EV.EVState.DRIVING
        self.myPosition = (0.,0.)
        self.myRendezvous = (0.,0.)
        self.myDrone = None
        self.myColour = traci.vehicle.getColor(self.myID)
        self.myChargeCount = 0
        self.myChargeSteps = 0
        self.myChaseSteps = 0
        self.myCapacity = EV.chargeDoneThreshold
        EV.evCount += 1

    def __del__(self):
        pass

    def __lt__(self,other):
        return self.myID < other.myID

    def __str__(self):
        return self.myID

    def allocate(self,drone,rvPos):
        """save the drone/ev relationship and set rendezvous position if necessary"""
        if GG.ss.modelRendezvous:
            self.myRendezvous = rvPos
        self.myDrone = drone

    def captureStats(self):
        """Add statistics for this vehicle into class variables - called when EV leaves"""
        EV.evChargeGap += (EV.chargeDoneThreshold - self.myCapacity)
        EV.evChargeCount += self.myChargeCount
        EV.evChargeSteps += self.myChargeSteps

    def getID(self):
        """getter function for EV identity"""
        return self.myID

    def getMyPosition(self):
        """getter function for x,y position"""
        return self.myPosition

    def leftSimulation(self):
        """State change"""
        self.myState = EV.EVState.LEFTSIMULATION

    def setMyPosition(self):
        """get real EV position from simulation and set my variable"""
        if self.myState == EV.EVState.WAITINGFORRENDEZVOUS:
            self.myDrone.notifyChase(False,self.myChaseSteps)   # failed chase
        self.myPosition = traci.vehicle.getPosition(self.myID)

    def stopCharging(self,clearDrone):
        """state change back to driving, remove the drone"""
        self.myState = EV.EVState.DRIVING
        if clearDrone:
            self.myDrone = None

    def update(self):
        """Implementation of EV state model - called in each simulation step"""
        match self.myState:
            case EV.EVState.DRIVING:
                if self.myDrone is not None:   # if drone triggers move to this state by breaking off it does not clear reference to itself
                    self.myCapacity = float(traci.vehicle.getParameter(self.myID, "device.battery.actualBatteryCapacity"))
                    GG.cc.notifyEVState(self,self.myState,self.myDrone.getID() + "x",self.myCapacity)
                    self.myDrone = None

                if (self.myChargeCount < 1 ) or  ( not GG.ss.onlyChargeOnce ) :
                    self.myCapacity = float(traci.vehicle.getParameter(self.myID, "device.battery.actualBatteryCapacity"))
                    if self.myCapacity < EV.chargeNeededThreshold:
                        self.setMyPosition()
                        traci.vehicle.setColor(self.myID,(255,0,0,255))   # red
                        self.myState = EV.EVState.CHARGEREQUESTED
                        GG.cc.requestCharge(self,self.myCapacity)

            case EV.EVState.CHARGEREQUESTED:
                if self.myDrone:
                    if GG.ss.modelRendezvous:
                      self.myDrone.update(self.myRendezvous)
                      self.myChaseSteps = 0
                      self.myState = EV.EVState.WAITINGFORRENDEZVOUS
                    else:
                      self.setMyPosition()
                      self.myDrone.update(self.myPosition)
                      self.myState = EV.EVState.WAITINGFORDRONE

            case EV.EVState.WAITINGFORRENDEZVOUS:
                if self.myDrone:
                    if self.myDrone.update(self.myRendezvous):
                        self.myState = EV.EVState.WAITINGFORDRONE

            case EV.EVState.WAITINGFORDRONE:
                self.setMyPosition()
                self.myChaseSteps += 1
                if self.myDrone.update(self.myPosition):
                    if self.myState == EV.EVState.WAITINGFORDRONE:       #  drone.update could have called EV.stopCharging
                        self.myDrone.notifyChase(True,self.myChaseSteps)
                        traci.vehicle.setColor(self.myID,(0,255,0,255))  # green
                        self.myState = EV.EVState.CHARGINGFROMDRONE
                        self.myCapacity = float(traci.vehicle.getParameter(self.myID, "device.battery.actualBatteryCapacity"))
                        GG.cc.notifyEVState(self,self.myState,self.myDrone.getID(),self.myCapacity)
                    else:   # failed chase because drone broke off and changed my state via EV.stopCharging
                        self.myDrone.notifyChase(False,self.myChaseSteps)
                        traci.vehicle.setColor(self.myID,self.myColour)
                        self.myDrone = None


            case EV.EVState.CHARGINGFROMDRONE:
                self.setMyPosition()
                self.myCapacity = float(traci.vehicle.getParameter(self.myID, "device.battery.actualBatteryCapacity"))
                chWh = self.myDrone.update(self.myPosition)
                if chWh is not False:
                    self.myCapacity += chWh
                    traci.vehicle.setParameter(self.myID, "device.battery.actualBatteryCapacity", self.myCapacity )
                    self.myChargeSteps += 1
                    if self.myCapacity >= EV.chargeDoneThreshold:     # we've finished charging
                        traci.vehicle.setColor(self.myID,self.myColour)
                        self.myState = EV.EVState.DRIVING
                        GG.cc.notifyEVState(self,self.myState,self.myDrone.getID(),self.myCapacity)
                        self.myDrone.notifyEVFinished(self.myState)
                        self.stopCharging(True)
                        self.myChargeCount += 1

            case EV.EVState.LEFTSIMULATION:
                self.captureStats()
                if self.myDrone:
                    GG.cc.notifyEVState(self,self.myState,self.myDrone.getID(),self.myCapacity)
                    self.myDrone.notifyEVFinished(self.myState)
                    self.myDrone = None
                else:
                    GG.cc.notifyEVState(self,self.myState,"",self.myCapacity)

                self.myState = EV.EVState.NULL

            case EV.EVState.NULL:
                pass
