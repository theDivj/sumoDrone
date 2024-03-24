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
        CHARGEBROKENOFF = 6             #  EV never reaches this state - only used for logging
        LEFTSIMULATION = 7
        NULL = 8

    # thresholds to trigger and stop vehicle charging  - unrealistic levels to facilitate demo - note sumo starts EVs with actual = 50% maximum battery capacity
    # realistic levels might be computed from the distance the vehicle is expected to travel, the remaining charge and some contingency of remaining charge?
    chargeNeededThreshold = 30000.   # 30KW
    chargeDoneThreshold = 32000.     # 32KW
    evChargeRequestWh = 2000.        # size of charge request - awaiting config of realistic levels
    pRandomVariation = 0.30          # +ve/-ve variation as percent of evChargeRequestWh

    # average distance vehicle will travel per Wh
    kmPerWh = 6.5 / 1000.   # default average used to compute vehicle range

    evCount = 0         # count of EVs
    evChargeSteps = 0   # total steps when EVs were charging
    evChargeGap = 0.0   # total charge gap
    evChargeCount = 0   # total no of full charges

    def __init__(self, evID, kmPerWh=0.0):
        if kmPerWh <= 0.0:
            self.myKmPerWh = EV.kmPerWh
        else:
            self.myKmPerWh = kmPerWh

        self.myID = evID
        self.myState = EV.EVState.DRIVING
        self.myPosition = (0., 0.)
        self.myRendezvous = (0., 0.)
        self.myDrone = None
        self.myColour = traci.vehicle.getColor(self.myID)
        self.myChargeCount = 0
        self.myChargeSteps = 0
        self.myChaseSteps = 0
        self.myCapacity = EV.chargeDoneThreshold    # we only shadow this when charging (state EV.EVState.CHARGINGFROMDRONE)

        self.myChargeNeededThreshold = EV.chargeNeededThreshold
        self.myevChargeRequestWh = EV.evChargeRequestWh

        self.setEVOverrides(self.myID)

        if GG.usingRandom():
            variation = EV.pRandomVariation * self.myevChargeRequestWh
            variation *= ((2. * GG.getRandom()) - 1.)
            self.myChargeNeededThreshold = EV.chargeNeededThreshold + variation + (1000. * GG.getRandom()) - 500
            self.myevChargeRequestWh += variation

        # We want to support ev requesting enough charge to get to destination
        # or to a charge point - meaning that a charge request will have a varying amount
        # allowing allocation of a drone with sufficient capacity to satisfy the request
        #    (setLastChargeRequest implementation needs extension to calculate specific charge needed)
        # Drone is now responsible for stopping charging after delivering requested amount
        self.myChargeDone = self.myChargeNeededThreshold + self.myevChargeRequestWh
        self.myLastChargeRequest =  self.myevChargeRequestWh
        EV.evCount += 1

    def __del__(self):
        pass

    def __lt__(self, other):
        return self.myID < other.myID

    def __str__(self):
        return self.myID

    def allocate(self, drone, rvPos):
        """save the drone/ev relationship and set rendezvous position - may be None"""
        self.myRendezvous = rvPos
        self.myDrone = drone

    def captureStats(self):
        """Add statistics for this vehicle into class variables - called when EV leaves"""
        EV.evChargeGap += (self.myChargeDone - self.myCapacity)
        EV.evChargeCount += self.myChargeCount
        EV.evChargeSteps += self.myChargeSteps

    def getID(self):
        """getter function for EV identity"""
        return self.myID

    def getMyKmPerWh(self):
        """getter for my average usage"""
        return self.myKmPerWh

    def getMyPosition(self):
        """getter function for x, y position"""
        return self.myPosition

    def leftSimulation(self):
        """State change"""
        self.myState = EV.EVState.LEFTSIMULATION

    def setEVOverrides(self,myID):
        """ check to see if we have an override defined for charge request - could be in type or vehicle definition - vehicle takes precedence"""
        vType = traci.vehicle.getTypeID(myID)
        overrideChargeWh = traci.vehicletype.getParameter(vType,"chargeRequestWh")
        vehicleOverrideChargeWh = traci.vehicle.getParameter(myID, "chargeRequestWh")
        oWh = 0
        if len(overrideChargeWh) > 1:
           oWh = float(overrideChargeWh)
        if len(vehicleOverrideChargeWh) > 1:
           oWh = float(vehicleOverrideChargeWh)
        if oWh > 1:
           self.myevChargeRequestWh = oWh

        overrideChargeRequestThresholdWh = traci.vehicletype.getParameter(vType,"chargeRequestThresholdWh")
        vehicleChargeRequestThresholdWh = traci.vehicle.getParameter(myID, "chargeRequestThresholdWh")
        oWh = 0.
        if len(overrideChargeRequestThresholdWh) > 1:
           oWh = float(overrideChargeRequestThresholdWh)
        if len(vehicleChargeRequestThresholdWh) > 1:
           oWh = float(vehicleChargeRequestThresholdWh)
        if oWh > 5000:
           self.myChargeNeededThreshold = oWh



    def setLastChargeRequest(self):
        """Work out how much charge (in Wh) is needed"""
        # currently varies if randomseed is passed in runString - otherwise just the EV value
        self.myLastChargeRequest = self.myevChargeRequestWh
        self.myChargeDone = self.myCapacity + self.myevChargeRequestWh   # not actually used at the moment - drone cuts off after delivering requested charge

    def setMyPosition(self):
        """get real EV position from simulation and set my variable"""
        if self.myState == EV.EVState.WAITINGFORRENDEZVOUS:     # never called from this state so
            self.myDrone.notifyChase(False, self.myChaseSteps)  # must be failed chase
        self.myPosition = traci.vehicle.getPosition(self.myID)

    def stopCharging(self, remainingCharge):
        """state change triggered by drone or ev leaving"""
        if self.myState == EV.EVState.CHARGINGFROMDRONE:
            if remainingCharge > 0:        # drone broke off so make request for remaining charge
                self.myState = EV.EVState.CHARGEREQUESTED
                GG.cc.notifyEVState(self,EV.EVState.CHARGEBROKENOFF, self.myDrone, self.myCapacity)
                GG.cc.requestCharge(self, self.myCapacity, remainingCharge)
                self.myDrone = None
            else:       # charge completed so log charge
                self.myCapacity = float(traci.vehicle.getParameter(self.myID, "device.battery.actualBatteryCapacity"))
                self.myState = EV.EVState.DRIVING
                GG.cc.notifyEVState(self, self.myState, self.myDrone, self.myCapacity)
                self.myDrone = None
        else:   # not yet got to EV
            self.myState = EV.EVState.DRIVING
            self.myDrone = None

    def update(self):
        """Implementation of EV state model - called in each simulation step"""
        match self.myState:
            case EV.EVState.DRIVING:
                if (self.myChargeCount < 1) or (not GG.onlyChargeOnce):
                    self.myCapacity = float(traci.vehicle.getParameter(self.myID, "device.battery.actualBatteryCapacity"))
                    if self.myCapacity < self.myChargeNeededThreshold:
                        self.setMyPosition()
                        traci.vehicle.setColor(self.myID, (255, 0, 0, 255))   # red
                        self.myState = EV.EVState.CHARGEREQUESTED
                        self.setLastChargeRequest()
                        GG.cc.requestCharge(self, self.myCapacity, self.myLastChargeRequest)

            case EV.EVState.CHARGEREQUESTED:
                if self.myDrone:
                    if GG.modelRendezvous:
                      self.myDrone.update(self.myRendezvous)
                      self.myChaseSteps = 0
                      self.myState = EV.EVState.WAITINGFORRENDEZVOUS
                    else:
                      self.setMyPosition()
                      self.myDrone.update(self.myPosition)
                      self.myState = EV.EVState.WAITINGFORDRONE

            case EV.EVState.WAITINGFORRENDEZVOUS:
                if self.myDrone:
                    if self.myDrone.update(self.myRendezvous)[0]:
                        self.myState = EV.EVState.WAITINGFORDRONE

            case EV.EVState.WAITINGFORDRONE:
                self.setMyPosition()
                self.myChaseSteps += 1
                if self.myDrone:
                    if self.myDrone.update(self.myPosition)[0]:
                        if self.myState == EV.EVState.WAITINGFORDRONE:       # drone.update could have called EV.stopCharging
                            self.myDrone.notifyChase(True, self.myChaseSteps)
                            traci.vehicle.setColor(self.myID, (0, 255, 0, 255))  # green
                            self.myState = EV.EVState.CHARGINGFROMDRONE
                            self.myCapacity = float(traci.vehicle.getParameter(self.myID, "device.battery.actualBatteryCapacity"))
                            GG.cc.notifyEVState(self, self.myState, self.myDrone, self.myCapacity)
                            self.myChargeDone = self.myCapacity + self.myLastChargeRequest # not quite right yet the charge will include usage whilst rendezvousing and charging
                        else:   # failed chase because drone broke off and changed my state via EV.stopCharging
                            self.myDrone.notifyChase(False, self.myChaseSteps)
                            traci.vehicle.setColor(self.myID, self.myColour)
                            self.myDrone = None

            case EV.EVState.CHARGINGFROMDRONE:
                self.setMyPosition()
                self.myCapacity = float(traci.vehicle.getParameter(self.myID, "device.battery.actualBatteryCapacity"))
                uStatus, chWh = self.myDrone.update(self.myPosition)
                if uStatus is False:   # either charge is finished or drone has broken off
                    if self.myState == EV.EVState.CHARGEREQUESTED:     # drone broke off before charge completed
                        self.myDrone = None
                    if self.myState == EV.EVState.DRIVING:   # Charge finished
                        traci.vehicle.setColor(self.myID, self.myColour)
                        self.myDrone = None
                        self.myChargeCount += 1
                else:
                    self.myCapacity += chWh
                    traci.vehicle.setParameter(self.myID, "device.battery.actualBatteryCapacity", self.myCapacity)
                    self.myChargeSteps += 1

            case EV.EVState.LEFTSIMULATION:
                self.captureStats()
                if self.myDrone:
                    GG.cc.notifyEVState(self, self.myState, self.myDrone, self.myCapacity)
                    self.myDrone.notifyEVFinished(self.myState)
                    self.myDrone = None
                else:
                    GG.cc.notifyEVState(self, self.myState, self.myDrone, self.myCapacity)

                self.myState = EV.EVState.NULL

            case EV.EVState.NULL:
                pass
