"""Drone module"""
import math
from enum import Enum
import traci
from GlobalClasses import GlobalClasses as GG
from EV import EV

class Drone:
    """Drone class - main parameters based on based on Ehang 184 which has top speed of 60km/h, battery capacity of 14.4 KW giving 23 mins flight time"""
    class DroneState(Enum):
        """Enumeration for Drone state model"""
        PARKED = 1
        FLYINGTORENDEZVOUS = 2
        FLYINGTOEV = 3
        CHARGINGEV = 4
        FLYINGTOCHARGE = 5
        CHARGINGDRONE = 6
        FLYINGTOPARK = 7
        NULL = 8

    droneIDCount = 0
    parkAtHome = False      # option to force parking/charging back to the charge hub where drone started

    # class variables
    droneKMperh = 60.0      # drone cruising speed
    droneMperSec = droneKMperh/3.6
    droneChargeWh = 30000.            # capacity of battery used to charge ev's  - guess based on Ehang 184 load capacity
    droneFlyingWh = 14400.            # capacity of battery used to power drone
    droneFlyingWhperTimeStep = droneFlyingWh / ( 23 * 60. )   # power usage based on Ehang 184 which has battery capacity of 14.4 KW giving 23 mins flight time
    droneChargeContingencyp = 0.05    # minimum contingency level %
    droneChargeViablep  = 0.3         # minimum viable level %
    minDroneCharge = droneChargeContingencyp    * droneChargeWh     # Thresholds to break off charging/flying
    minDroneFlyingWh = droneChargeContingencyp  * droneFlyingWh
    viableDroneCharge = droneChargeViablep      * droneChargeWh     # thresholds to allow allocation - ie enough charge to be useful
    viableDroneFlyingWh = droneChargeViablep    * droneFlyingWh
    droneStepMperTimeStep = droneMperSec                            # How far (metres) the drone will travel in one time step (adjust for timeStep when simulation starts)
    droneStepM2 = droneStepMperTimeStep * droneStepMperTimeStep     # precompute - used in distance calculations  (adjust for timeStep when simulation starts)
    WhEVChargeRatePerTimeStep = 25000. / 3600                       # 25KW   rate of vehicle charge from drone  (adjust for timeStep when simulation starts)
    WhDroneRechargePerTimeStep = 75000. / 3600                      # 75KW   rate of drone charge when parked  (adjust for timeStep when simulation starts)

    def __init__(self, pos):
        Drone.droneIDCount += 1
        self.myID = "d" + str(Drone.droneIDCount)
        self.myPosition = pos
        self.myParkPosition = self.myPosition
        self.myCharge = Drone.droneChargeWh
        self.myFlyingCharge = Drone.droneFlyingWh
        self.myViableCharge = True
        self.myState = Drone.DroneState.PARKED
        self.myEV = None
        # logging variables
        self.myFlyingCount = 0            # used to compute distance travelled
        self.myFullCharges = 0             #  count of complete charges
        self.myBrokenCharges = 0           #  count of charges broken off - by me out of charge
        self.myBrokenEVCharges = 0         #  count of charges broken off - by EV (leaving)
        self.myFlyingWh = 0.0              # wH i've used flying
        self.myChargingWh = 0.0            # wH i've used charging EVs
        self.myChargeMeFlyingCount = 0.0   # wh i've charged my flying battery
        self.myChargeMeCount = 0.0         # wh i've used charging my EV charging battery
        self.myChaseCount = 0              # count of complete chases - ie got from rendezvous to ev
        self.myBrokenChaseCount = 0        # count of broken chases where we didn't get to ev before it left sim
        self.myChaseSteps = 0              # count of steps in all complete chases - used with myChaseCount to compute average
        # finally create the POI representing our drone
        traci.poi.add(self.myID, pos[0], pos[1], color=(0,0,255,255), layer=250, imgFile=".\\drone.png", width=10, height=10)

    def __lt__(self,other):
        return int(self.myID[1:]) < int(other.myID[1:])

    def __str__(self):
        return self.myID

    @classmethod
    def stepSecsAdjust(cls,stepSecs):
        """ adjust timestep class variables for the actual timestep = no change when timeStep = 1sec"""
        Drone.droneStepMperTimeStep *= stepSecs
        Drone.droneStepM2 = Drone.droneStepMperTimeStep * Drone.droneStepMperTimeStep  # precompute - used in distance calculations
        Drone.droneFlyingWhperTimeStep *= stepSecs
        Drone.WhEVChargeRatePerTimeStep *= stepSecs
        Drone.WhDroneRechargePerTimeStep *= stepSecs

    def allocate(self,ev):
        """allocate this instance to an EV"""
        self.myEV = ev
        traci.poi.setParameter(self.myID,"status","allocated to " + ev.getID())
        if GG.ss.modelRendezvous:
            self.myState = Drone.DroneState.FLYINGTORENDEZVOUS
        else:
            self.myState = Drone.DroneState.FLYINGTOEV
        return True

    def chargeMe(self):
        """Drone at hub - charge if needed"""
        if self.myCharge < Drone.droneChargeWh:
            self.myCharge += Drone.WhDroneRechargePerTimeStep
            self.myChargeMeCount += 1
        if self.myFlyingCharge < Drone.droneFlyingWh:
            self.myFlyingCharge += Drone.WhDroneRechargePerTimeStep
            self.myChargeMeFlyingCount += 1
        elif self.myCharge >= Drone.droneChargeWh:
            #self.myCharge = Drone.droneChargeWh
            #self.myFlyingCharge = Drone.droneFlyingWh
            self.myState = Drone.DroneState.NULL
            self.setViableCharge()

    def fly(self,pos):
        """move the drone along a straight line to pos by the amount Drone can move in a timeStep
            returns True if we've arrived at pos, False otherwise
        """
        dx,dy = self.myPosition
        px,py = pos

        ddy = py-dy
        ddx = px-dx
        try:
            dydx = ddy/ddx
            x = abs( math.sqrt( self.droneStepM2 / (1.0 + dydx * dydx ) ) )
            y = abs( x * dydx )
        except (ValueError,ZeroDivisionError):    #  assume either x or y positions are equal
            if dx == px:
                if dy == py:
                    x = 0.
                    y = 0.
                else:
                    x = 0.
                    y = abs(ddy)
            else:   # assume dy = py
                x = abs(ddx)
                y = 0.

        if abs(ddx) <= x:  # will reach px in this step so set to ~exact distance
           x = px + 0.001
        elif ddx > 0:
           x += dx
        else:
          x = dx - x

        if abs(ddy) <= y:   ## will reach py in this step so set to ~exact distance
          y = py + 0.001
        elif ddy > 0:
          y += dy
        else:
          y = dy - y

        traci.poi.setPosition(self.myID,x,y)
        self.myPosition = (x,y)

        if ( abs(x - px) + abs( y - py ) ) < 5.0:    # we've arrived at px,py  - arbitrary 5m - two car kengths
            return True
        return False                            #  we haven't got anywhere yet!

    def getID(self):
        """getter for Drone SUMO ID"""
        return self.myID

    def getMyPosition(self):
        """getter for position"""
        return self.myPosition

    def logLine(self,activity):
        """Output discrete changes in charge levels for this drone"""
        x,y = self.myPosition
        if self.myEV is not None:
            evID = self.myEV.getID()
            lane = traci.vehicle.getLaneID(evID)
            lanePos = traci.vehicle.getLanePosition(evID)
        else:
            evID = ""
            lane = ""
            lanePos = ""
        print("{:.1f}\t{}\t{}\t{}\t{}\t{:.4f}\t{:.4f}\t{:.4f}\t{:.4f}\t{:.4f}\t{}".format\
          (GG.ss.timeStep,self.myID,evID,lane,lanePos,x,y,self.myChargingWh,self.myCharge,self.myFlyingCharge,activity), file=GG.ss.droneLog)

    def notifyChase(self,chaseOK,chaseSteps):
        """from EV updating chases by this drone"""
        if chaseOK:
            self.myChaseCount += 1
            self.myChaseSteps += chaseSteps
        else:
            self.myBrokenChaseCount += 1

    def notifyEVFinished(self,evState):
        """EV tells us that it is charged or has left simulation so free self up"""
        self.setMyParkPosition()
        self.myState = Drone.DroneState.FLYINGTOPARK
        traci.poi.setParameter(self.myID,"status","Flying to park")
        GG.cc.notifyDroneState(self)
        self.myEV = None

        match evState:
            case EV.EVState.LEFTSIMULATION:
                self.myBrokenEVCharges += 1
                self.setMyParkPosition()
                self.myState = Drone.DroneState.FLYINGTOPARK

            case EV.EVState.DRIVING:
                self.myFullCharges += 1

    def parkingUpdate(self):
        """secondary update - invoked when Control centre is managing drone"""
        match self.myState:
            case Drone.DroneState.FLYINGTOPARK | Drone.DroneState.FLYINGTOCHARGE | Drone.DroneState.PARKED | Drone.DroneState.CHARGINGDRONE | Drone.DroneState.NULL:
                pass
            case _:
                self.myState = Drone.DroneState.FLYINGTOPARK
        self.update(self.myParkPosition)

    def setMyParkPosition(self):
        """configure my parking/charging position"""
        if not Drone.parkAtHome:   # then we park at nearest hub
            (x,y,e,p),distance = GG.ch.nearestHubLocation(self.myPosition)
            self.myParkPosition = (x,y)

    def setViableCharge(self):
        """Check charge levels and see if we are viable - ie can be allocated"""
        if self.myCharge >= Drone.viableDroneCharge  and self.myFlyingCharge >= Drone.viableDroneFlyingWh:
            if not self.myViableCharge:
                self.myViableCharge = True
                GG.cc.notifyDroneState(self)  # cc only interested when we become viable
                traci.poi.setColor(self.myID,(0,0,255,255))
        else:
            self.myViableCharge = False

    def update(self,pos):
        """primary update - invoked when EV is managing drone"""
        updateStatus = True
        match self.myState:
            case Drone.DroneState.PARKED:
                self.chargeMe()            # add charge upto limit because we can
                if GG.ss.dronePrint:
                    self.logLine("Parked")

            case Drone.DroneState.FLYINGTORENDEZVOUS:
                self.usePower("fly")
                if self.fly(pos):
                    self.myState = Drone.DroneState.FLYINGTOEV
                    if GG.ss.dronePrint:
                        self.logLine("Arrived at rendezvous")
                else:
                    updateStatus = False
                    if GG.ss.dronePrint:
                        self.logLine("flying to rendezvous")

            case Drone.DroneState.FLYINGTOEV:               # note that previous version did not count power/distance used in arriving at ev
                self.usePower("fly")
                if self.fly(pos):
                    if self.myEV is None:
                        print("assuming ev left sim",GG.ss.timeStep,self.myID)
                        self.notifyEVFinished(EV.EVState.LEFTSIMULATION)
                    else:
                        traci.poi.setParameter(self.myID,"status","charging:" + self.myEV.getID())
                        self.myState = Drone.DroneState.CHARGINGEV
                        if GG.ss.dronePrint:
                            self.logLine("arrived at ev")
                else:
                    updateStatus = False
                    if GG.ss.dronePrint:
                        self.logLine("flying to ev")

            case Drone.DroneState.CHARGINGEV:
                self.fly(pos)
                self.usePower("chargeEV")
                updateStatus = Drone.WhEVChargeRatePerTimeStep
                if GG.ss.dronePrint:
                    self.logLine("charging EV")

            case Drone.DroneState.CHARGINGDRONE:
                self.chargeMe()
                if GG.ss.dronePrint:
                    self.logLine("charging self")

            case Drone.DroneState.FLYINGTOCHARGE:   # note that previous version did not count power/distance used in arriving at hub
                self.usePower("")
                if self.fly(self.myParkPosition):
                    traci.poi.setParameter(self.myID,"status","parked - needs charge")
                    traci.poi.setColor(self.myID,(0,255,0,255))
                    self.myState = Drone.DroneState.CHARGINGDRONE
                    if GG.ss.dronePrint:
                        self.logLine("arrived at charge hub")
                else:
                    updateStatus = False
                    if GG.ss.dronePrint:
                        self.logLine("flying to charge hub")

            case Drone.DroneState.FLYINGTOPARK:     # note that previous version did not count power/distance used in arriving at hub
                self.usePower("fly")
                if self.fly(self.myParkPosition):
                    traci.poi.setParameter(self.myID,"status","Parked")
                    self.myState = Drone.DroneState.PARKED
                    if GG.ss.dronePrint:
                        self.logLine("arrived at hub")
                else:
                    updateStatus = False
                    if GG.ss.dronePrint:
                        self.logLine("flying to hub")

            case Drone.DroneState.NULL:  # am charged so do nothing until allocated
                pass

        return updateStatus

    def usePower(self,mode):
        """Am flying or charging an EV so adjust my charge levels"""
        breakOff = False
        match mode:
            case "fly":
                self.myFlyingCharge -= Drone.droneFlyingWhperTimeStep
                self.myFlyingWh += Drone.droneFlyingWhperTimeStep
                self.myFlyingCount += 1
                if self.myFlyingCharge < Drone.minDroneFlyingWh:
                    breakOff = True
            case "chargeEV":
                self.myCharge -= Drone.WhEVChargeRatePerTimeStep
                self.myChargingWh += Drone.WhEVChargeRatePerTimeStep
                if self.myCharge < Drone.minDroneCharge:
                    breakOff = True
            case _:
                if self.myState == Drone.DroneState.FLYINGTOCHARGE:       # we're flying back to charge, only time we get here for now
                    self.myFlyingCount += 1
                    self.myFlyingCharge -= Drone.droneFlyingWhperTimeStep

        # problem - one of my batteries below contingency
        if breakOff:
            if self.myEV:
                self.myEV.stopCharging(False)
                self.myEV = None
                self.myBrokenCharges += 1
            self.myViableCharge = False
            self.setMyParkPosition()
            traci.poi.setColor(self.myID,(255,0,0,255))
            traci.poi.setParameter(self.myID,"status","Flying to charge")
            self.myState = Drone.DroneState.FLYINGTOCHARGE
            GG.cc.notifyDroneState(self)
            return False
        return True

    def viable(self):
        """helper function to check whether we are able to be diverted"""
        match self.myState:
            case Drone.DroneState.FLYINGTOCHARGE:
                return False
            case Drone.DroneState.CHARGINGDRONE:
                return self.myViableCharge
            case _:
                return True
