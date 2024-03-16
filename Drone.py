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

    # Drone specific class variables
    droneKMperh = 60.0      # drone cruising speed - will be overridden by global / runstring value
    droneChargeWh = 30000.            # capacity of battery used to charge ev's  - based on Ehang 184 load capacity
    droneFlyingWh = 14400.            # capacity of battery used to power drone
    droneFlyingWhperTimeStep = droneFlyingWh / (23 * 60.)   # power usage based on Ehang 184 which has battery capacity of 14.4 KW giving 23 mins flight time
    droneChargeContingencyp = 0.05    # minimum contingency level %
    droneChargeViablep = 0.3          # minimum viable level %
    WhEVChargeRatePerTimeStep = 25000. / 3600       # 25KW   rate of vehicle charge from drone  (adjust for timeStep when simulation starts)
    WhDroneRechargePerTimeStep = 75000. / 3600      # 75KW   rate of drone charge when parked  (adjust for timeStep when simulation starts)

    # Derived class variables
    droneMperSec = droneKMperh / 3.6
    droneStepMperTimeStep = droneMperSec                            # How far (metres) the drone will travel in one time step (adjust for timeStep when simulation starts)
    droneStepM2 = droneStepMperTimeStep * droneStepMperTimeStep     # precompute - used in distance calculations  (adjust for timeStep when simulation starts)
    minDroneCharge = droneChargeContingencyp    * droneChargeWh     # Thresholds to break off charging/flying
    minDroneFlyingWh = droneChargeContingencyp  * droneFlyingWh
    viableDroneCharge = droneChargeViablep      * droneChargeWh     # thresholds to allow allocation - ie enough charge to be useful
    viableDroneFlyingWh = droneChargeViablep    * droneFlyingWh

    dummyEVCreated = False          # safety flag - in case we call dummyEVHide twice

    def __init__(self, pos, droneType="ehang184"):
        Drone.droneIDCount += 1
        Drone.setDroneType(droneType)
        self.myID = "d" + str(Drone.droneIDCount)
        self.myPosition = pos
        self.myParkPosition = self.myPosition
        self.myParkEP = self.myPosition
        self.myCharge = Drone.droneChargeWh
        self.myFlyingCharge = Drone.droneFlyingWh
        self.myViableCharge = True
        self.myState = Drone.DroneState.NULL
        self.myEV = None

        # reset drone speed factors with any runstring override
        Drone.droneKMperh = GG.getDroneSpeed()
        Drone.droneMperSec = Drone.droneKMperh / 3.6
        Drone.droneStepMperTimeStep = Drone.droneMperSec                            # How far (metres) the drone will travel in one time step (adjust for timeStep when simulation starts)
        Drone.droneStepM2 = Drone.droneStepMperTimeStep * Drone.droneStepMperTimeStep     # precompute - used in distance calculations  (adjust for timeStep when simulation starts)
        Drone.createDummyEV()

        # logging variables
        self.myFlyingCount = 0            # used to compute distance travelled
        self.myFullCharges = 0             # count of complete charges
        self.myBrokenCharges = 0           # count of charges broken off - by me out of charge
        self.myBrokenEVCharges = 0         # count of charges broken off - by EV (leaving)
        self.myFlyingWh = 0.0              # wH i've used flying
        self.myChargingWh = 0.0            # wH i've used charging EVs
        self.myChargeMeFlyingCount = 0.0   # wh i've charged my flying battery
        self.myChargeMeCount = 0.0         # wh i've used charging my EV charging battery
        self.myChaseCount = 0              # count of complete chases - ie got from rendezvous to ev
        self.myBrokenChaseCount = 0        # count of broken chases where we didn't get to ev before it left sim
        self.myChaseSteps = 0              # count of steps in all complete chases - used with myChaseCount to compute average
        self.myRequestedCharge = 0         # the amount of charge requested by the EV
        self.myDummyEVInserted = False     # whether the dummy EVs have been inserted
        # finally create the POI representing our drone
        traci.poi.add(self.myID, pos[0], pos[1], color=(0, 0, 255, 255), layer=250, imgFile="drone.png", width=10, height=10)

    def __lt__(self, other):
        return int(self.myID[1:]) < int(other.myID[1:])

    def __str__(self):
        return self.myID

    @classmethod
    def createDummyEV(cls):
        """Create an EV type to use at charging stations whilst drone is charging"""
        if Drone.dummyEVCreated:
            return
        traci.vehicletype.copy("DEFAULT_VEHTYPE","Drone")
        traci.vehicletype.setWidth("Drone","0.1")
        traci.vehicletype.setLength("Drone","0.1")
        traci.vehicletype.setMinGap("Drone","0.0")
        traci.vehicletype.setParameter("Drone", "has.battery.device", "True")
        traci.vehicletype.setEmissionClass("Drone", "Energy/unknown");
        Drone.dummyEVCreated = True

    @classmethod
    def setDroneType(cls, droneType="ehang184"):
        """Support different drone definitions - initially to give us a drone that doesn't need charging"""

        Drone.droneKMperh = GG.getDroneSpeed()  # get speed override if any

        # EV charging battery size is constrained by drone carrying capacity * average battery energy density (currently ~150Wh/Kg)
        match droneType:
            case "ehang184":
                Drone.droneChargeWh = 30000.                      # capacity of battery used to charge ev's, based on Ehang 184 load capacity - 200Kg
                Drone.droneFlyingWh = 14400.                      # capacity of battery used to power drone
                Drone.droneFlyingWhperTimeStep = Drone.droneFlyingWh / (23 * 60.)   # Ehang 184 has battery capacity of 14.4 KW giving 23 mins flight time
                Drone.droneChargeContingencyp = 0.05              # minimum contingency level %
                Drone.droneChargeViablep = 0.3                    # minimum viable level %
                Drone.WhEVChargeRatePerTimeStep = 25000. / 3600   # 25KW   rate of vehicle charge from drone  (adjust for timeStep when simulation starts)
                Drone.WhDroneRechargePerTimeStep = 75000. / 3600  # 75KW   rate of drone charge when parked  (adjust for timeStep when simulation starts)

            case "ehang184x":            # ehang 184 with artificially increased battery sizes so they don't need recharging
                Drone.droneChargeWh = 3000000.     # 100 * actual
                Drone.droneFlyingWh = 14400000.
                Drone.droneFlyingWhperTimeStep = 14400 / (23 * 60.)
                Drone.droneChargeContingencyp = 0.05              # minimum contingency level %
                Drone.droneChargeViablep = 0.3                    # minimum viable level %
                Drone.WhEVChargeRatePerTimeStep = 25000. / 3600   # 25KW   rate of vehicle charge from drone  (adjust for timeStep when simulation starts)
                Drone.WhDroneRechargePerTimeStep = 75000. / 3600  # 75KW   rate of drone charge when parked  (adjust for timeStep when simulation starts)

        Drone.droneMperSec = Drone.droneKMperh / 3.6
        Drone.droneStepMperTimeStep = Drone.droneMperSec                            # How far (metres) the drone will travel in one time step (adjust for timeStep when simulation starts)
        Drone.droneStepM2 = Drone.droneStepMperTimeStep * Drone.droneStepMperTimeStep     # precompute - used in distance calculations  (adjust for timeStep when simulation starts)
        Drone.minDroneCharge = Drone.droneChargeContingencyp    * Drone.droneChargeWh     # Thresholds to break off charging/flying
        Drone.minDroneFlyingWh = Drone.droneChargeContingencyp  * Drone.droneFlyingWh
        Drone.viableDroneCharge = Drone.droneChargeViablep      * Drone.droneChargeWh     # thresholds to allow allocation - ie enough charge to be useful
        Drone.viableDroneFlyingWh = Drone.droneChargeViablep    * Drone.droneFlyingWh

    @classmethod
    def stepSecsAdjust(cls, stepSecs):
        """ adjust timestep class variables for the actual timestep = no change when timeStep = 1sec"""
        Drone.droneStepMperTimeStep *= stepSecs
        Drone.droneStepM2 = Drone.droneStepMperTimeStep * Drone.droneStepMperTimeStep  # precompute - used in distance calculations
        Drone.droneFlyingWhperTimeStep *= stepSecs
        Drone.WhEVChargeRatePerTimeStep *= stepSecs
        Drone.WhDroneRechargePerTimeStep *= stepSecs

    def allocate(self, ev, requestedCharge ):
        """allocate this instance to an EV"""
        if self.myState in (Drone.DroneState.CHARGINGDRONE, Drone.DroneState.PARKED):
            self.dummyEVHide()
        self.myEV = ev
        self.myRequestedCharge = requestedCharge
        traci.poi.setParameter(self.myID, "status", "allocated to " + ev.getID())
        if GG.modelRendezvous:
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
        elif self.myCharge >= Drone.droneChargeWh:  # fully charged so move to null state, avoiding calls to chargeMe
            # self.myCharge = Drone.droneChargeWh
            # self.myFlyingCharge = Drone.droneFlyingWh
            self.dummyEVHide()
            self.myState = Drone.DroneState.NULL
            self.setViableCharge()

    def dummyEVInsert(self):
        """If we are generating charge station output add dummy EVs to the charge station for the drone batteries - whilst the drone is there"""
        if GG.ss.useChargeHubs:
            e,p = self.myParkEP
            lane = e + "_0"
            dummyFB = self.myID + "-FB"
            traci.vehicle.add(dummyFB,e,"Drone")
            traci.vehicle.setParameter(dummyFB, "device.battery.maximumBatteryCapacity", Drone.droneFlyingWh)
            traci.vehicle.setParameter(dummyFB, "device.battery.actualBatteryCapacity", self.myFlyingCharge)
            traci.vehicle.setEmissionClass(dummyFB, "Energy/unknown");
            traci.vehicle.moveTo(dummyFB,lane,p)
            traci.vehicle.setStop(dummyFB, e, pos=p, duration=10000.0, flags=1)

            dummyCB = self.myID + "-CB"
            traci.vehicle.add(dummyCB,e,"Drone")
            traci.vehicle.setParameter(dummyCB, "device.battery.maximumBatteryCapacity", Drone.droneChargeWh)
            traci.vehicle.setParameter(dummyCB, "device.battery.actualBatteryCapacity", self.myCharge)
            traci.vehicle.setEmissionClass(dummyCB, "Energy/unknown");
            traci.vehicle.moveTo(dummyCB,lane,p + 0.5)
            traci.vehicle.setStop(dummyCB, e, pos=p + 0.5, duration=10000.0, flags=1)
            self.myDummyEVInserted = True

    def dummyEVHide(self):
        """remove the dummy EVs"""
        if GG.ss.useChargeHubs and self.myDummyEVInserted:
            e,p = self.myParkEP
            dummyFB = self.myID + "-FB"
            traci.vehicle.resume(dummyFB)
            traci.vehicle.remove(dummyFB)
            dummyCB = self.myID + "-CB"
            traci.vehicle.resume(dummyCB)
            traci.vehicle.remove(dummyCB)
            self.myDummyEVInserted = False

    def fly(self, pos):
        """move the drone along a straight line to pos by the amount Drone can move in a timeStep,
            returns True if we've arrived at pos, False otherwise
        """
        dx, dy = self.myPosition
        px, py = pos

        ddy = py - dy
        ddx = px - dx
        x = 0.
        y = 0.
        if ( ddx == 0 or ddy == 0 ): 
            if (ddx == 0):
                if (ddy == 0):
                    x = 0.
                    y = 0.
                else:
                    x = 0.
                    y = abs(ddy)
            else: # ddy == 0
                    x = abs(ddx)
                    y = 0.0
            
        else:
            x = abs(math.sqrt(self.droneStepM2 / (1.0 + pow(ddy/ddx ,2))))
            y = abs(x * ddy/ddx)
 
        if abs(ddx) <= x:  # will reach px in this step so set to ~exact distance
           x = px + 0.001
        elif ddx > 0:
           x += dx
        else:
          x = dx - x

        if abs(ddy) <= y:   # # will reach py in this step so set to ~exact distance
          y = py + 0.001
        elif ddy > 0:
          y += dy
        else:
          y = dy - y

        traci.poi.setPosition(self.myID, x, y)
        self.myPosition = (x, y)

        if (abs(x - px) + abs(y - py)) < 5.0:    # we've arrived at px, py  - arbitrary 5m - two car kengths
            return True
        return False                            # we haven't got anywhere yet!

    def getID(self):
        """getter for Drone SUMO ID"""
        return self.myID

    def getMyPosition(self):
        """getter for position"""
        return self.myPosition

    def logLine(self, activity):
        """Output discrete changes in charge levels for this drone"""
        x, y = self.myPosition
        if self.myEV is not None:
            evID = self.myEV.getID()
            lane = traci.vehicle.getLaneID(evID)
            lanePos = float(traci.vehicle.getLanePosition(evID))
        else:
            evID = ""
            lane = ""
            lanePos = 0
        print("{:.1f}\t{}\t{}\t{}\t{:.4f}\t{:.4f}\t{:.4f}\t{:.4f}\t{:.4f}\t{:.4f}\t{}".format
              (GG.ss.timeStep, self.myID, evID, lane, lanePos, x, y, self.myChargingWh, self.myCharge, self.myFlyingCharge, activity), file=GG.droneLog)

    def notifyChase(self, chaseOK, chaseSteps):
        """from EV updating chases by this drone"""
        if chaseOK:
            self.myChaseCount += 1
            self.myChaseSteps += chaseSteps
        else:
            self.myBrokenChaseCount += 1

    def notifyEVFinished(self, evState):
        """EV tells us that it is charged or has left simulation so free self up"""
        match evState:
            case EV.EVState.LEFTSIMULATION:
                self.myBrokenEVCharges += 1
                self.setMyParkPosition()
                if self.myState not in (Drone.DroneState.CHARGINGDRONE, Drone.DroneState.PARKED):
                    self.park()
                else:
                    self.myEV = None

            case EV.EVState.DRIVING:
                self.myFullCharges += 1
                self.park()

    def park(self):
        """charge finished so park drone"""
        self.setMyParkPosition()
        self.myState = Drone.DroneState.FLYINGTOPARK
        traci.poi.setParameter(self.myID, "status", "Flying to park")
        GG.cc.notifyDroneState(self)
        self.myEV = None

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
            (x, y, e, p), distance = GG.ch.nearestHubLocation(self.myPosition)
            self.myParkPosition = (x, y)
            self.myParkEP = (e,p)

    def setViableCharge(self):
        """Check charge levels and see if we are viable - ie can be allocated"""
        if self.myCharge >= Drone.viableDroneCharge and self.myFlyingCharge >= Drone.viableDroneFlyingWh:
            if not self.myViableCharge:
                self.myViableCharge = True
                GG.cc.notifyDroneState(self)  # cc only interested when we become viable
                traci.poi.setColor(self.myID, (0, 0, 255, 255))
        else:
            self.myViableCharge = False

    def update(self, pos):
        """primary update - invoked directly when EV is managing drone"""
        updateStatus = True
        match self.myState:
            case Drone.DroneState.PARKED:
                self.chargeMe()            # add charge upto limit because we can
                if GG.dronePrint:
                    self.logLine("Parked")

            case Drone.DroneState.FLYINGTORENDEZVOUS:
                if self.usePower("fly"):
                    if self.fly(pos):
                        if self.myEV is not None:   # usePower might have broken off after using power and set myEV to None
                            self.myState = Drone.DroneState.FLYINGTOEV
                            if GG.dronePrint:
                                self.logLine("Arrived at rendezvous")
                    else:
                        updateStatus = False
                        if GG.dronePrint:
                            self.logLine("flying to rendezvous")
                else:
                    self.fly(pos)
                    if GG.dronePrint:
                        self.logLine("breaking off")
                    updateStatus = False

            case Drone.DroneState.FLYINGTOEV:
                if self.usePower("fly"):
                    if self.fly(pos):
                        if self.myEV is not None:   # usePower might have broken off after using power and set myEV to None
                            traci.poi.setParameter(self.myID, "status", "charging:" + self.myEV.getID())
                            self.myState = Drone.DroneState.CHARGINGEV
                            if GG.dronePrint:
                                self.logLine("arrived at ev")
                    else:
                        updateStatus = False
                        if GG.dronePrint:
                            self.logLine("flying to ev")
                else:
                    self.fly(pos)
                    if GG.dronePrint:
                        self.logLine("breaking off")
                    updateStatus = False

            case Drone.DroneState.CHARGINGEV:
                self.fly(pos)                             # 'fly' in this case is just moving with attached to the ev
                if self.usePower("chargeEV"):             # False when charge broken off or completed
                    updateStatus = Drone.WhEVChargeRatePerTimeStep
                else:
                    updateStatus = False
                if GG.dronePrint:
                    self.logLine("charging EV")

            case Drone.DroneState.CHARGINGDRONE:
                self.chargeMe()
                if GG.dronePrint:
                    self.logLine("charging self")

            case Drone.DroneState.FLYINGTOCHARGE:
                self.usePower("")
                if self.fly(self.myParkPosition):
                    traci.poi.setParameter(self.myID, "status", "parked - needs charge")
                    traci.poi.setColor(self.myID, (0, 255, 0, 255))
                    self.myState = Drone.DroneState.CHARGINGDRONE
                    self.dummyEVInsert()
                    if GG.dronePrint:
                        self.logLine("arrived at charge hub")
                else:
                    updateStatus = False
                    if GG.dronePrint:
                        self.logLine("flying to charge hub")

            case Drone.DroneState.FLYINGTOPARK:     # note that previous version did not count power/distance used in arriving at hub
                self.usePower("fly")
                if self.fly(self.myParkPosition):
                    traci.poi.setParameter(self.myID, "status", "Parked")
                    self.myState = Drone.DroneState.PARKED
                    self.dummyEVInsert()
                    if GG.dronePrint:
                        self.logLine("arrived at hub")
                else:
                    updateStatus = False
                    if GG.dronePrint:
                        self.logLine("flying to hub")

            case Drone.DroneState.NULL:  # am charged so do nothing until allocated
                pass

        return updateStatus

    def usePower(self, mode):
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
                self.myRequestedCharge -= Drone.WhEVChargeRatePerTimeStep
                if self.myRequestedCharge <= 0:    #  we've charged the requested amount
                    self.myEV.stopCharging(0)   # don't clear, full charge
                    self.myFullCharges += 1
                    self.myRequestedCharge = 0
                    self.park()
                    return False      # ie we're not returning a charge
                if self.myCharge < Drone.minDroneCharge:
                    breakOff = True
            case _:
                if self.myState == Drone.DroneState.FLYINGTOCHARGE:       # we're flying back to charge, only time we get here for now
                    self.myFlyingCount += 1
                    self.myFlyingCharge -= Drone.droneFlyingWhperTimeStep

        # problem - one of my batteries below contingency
        if breakOff:
            if self.myEV:
                request = int(self.myRequestedCharge + 1)
                self.myEV.stopCharging(request)   # tell EV how much charge we still need to apply to fulfil original request
                self.myEV = None
                self.myBrokenCharges += 1

            self.myRequestedCharge = 0
            self.myViableCharge = False
            self.setMyParkPosition()
            traci.poi.setColor(self.myID, (255, 0, 0, 255))
            traci.poi.setParameter(self.myID, "status", "Flying to charge")
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
