"""Drone module"""
import math
from enum import Enum
import traci
from GlobalClasses import GlobalClasses as GG
from DroneType import DroneType
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
        NULLState = 8

    droneIDCount = 0
    parkAtHome = False      # option to force parking/charging back to the charge hub where drone started
    d0Type = DroneType()

    dummyEVCreated = False          # safety flag - in case we call dummyEVHide twice

    def __init__(self, pos, poi, dt):
        if len(poi) < 2:
            Drone.droneIDCount += 1
            self.myDt = Drone.d0Type
            self.myID = "d" + str(Drone.droneIDCount)
        else:
            Drone.droneIDCount += 1
            self.myDt = dt
            self.myID = poi

        # reset drone speed factors with any runstring override
        self.myDt.droneKMperh = GG.getDroneSpeed()
        self.myDt.setDerived(GG.ss.stepSecs)

        self.myPosition = pos
        self.myParkPosition = self.myPosition
        self.myParkEP = self.myPosition
        self.myCharge = self.myDt.droneChargeWh
        self.myFlyingCharge = self.myDt.droneFlyingWh
        self.myViableCharge = True
        self.myState = Drone.DroneState.NULLState
        self.myEV = None

        # logging variables
        self.myFlyingCount = 0            # used to compute distance travelled
        self.myFullCharges = 0             # count of complete charges
        self.myBrokenCharges = 0           # count of charges broken off - by me out of charge
        self.myBrokenEVCharges = 0         # count of charges broken off - by EV (leaving)
        self.myEVChargingCount = 0         # Count of timesteps charging EVs
        self.myChargeMeFlyingCount = 0.0   # wh i've charged my flying battery
        self.myChargeMeCount = 0.0         # wh i've used charging my EV charging battery
        self.myChaseCount = 0              # count of complete chases - ie got from rendezvous to ev
        self.myBrokenChaseCount = 0        # count of broken chases where we didn't get to ev before it left sim
        self.myChaseSteps = 0              # count of steps in all complete chases - used with myChaseCount to compute average
        self.myRequestedCharge = 0         # the amount of charge requested by the EV
        self.myDummyEVInserted = False     # whether the dummy EVs have been inserted
        # finally create the POI representing our drone
        if self.myID != poi:
            traci.poi.add(self.myID, pos[0], pos[1], color=self.myDt.droneColour, layer=250, imgFile=self.myDt.droneImageFile, width=self.myDt.droneWidth, height=self.myDt.droneHeight)
        else:
            traci.poi.setImageFile(self.myID, self.myDt.droneImageFile)
            traci.poi.setColor(self.myID, self.myDt.droneColour)
            traci.poi.setWidth(self.myID, self.myDt.droneWidth)
            traci.poi.setHeight(self.myID, self.myDt.droneHeight)

        if GG.ss.useChargeHubs and not Drone.dummyEVCreated:
            Drone.createDummyEV()

    def printDroneType(self):
        """ helper function to list current droneType values"""
        print("droneKMperh:\t", self.myDt.droneKMperh)
        print("droneChargeWh:\t", self.myDt.droneChargeWh)
        print("droneFlyingWh:\t", self.myDt.droneFlyingWh)
        print("droneFlyingWhperTimeStep:\t", self.myDt.droneFlyingWhperTimeStep)
        print("droneChargeContingencyp:\t", self.myDt.droneChargeContingencyp)
        print("droneChargeViablep:\t", self.myDt.droneChargeViablep)
        print("WhEVChargeRatePerTimeStep:\t", self.myDt.WhEVChargeRatePerTimeStep)
        print("WhDroneRechargePerTimeStep:\t", self.myDt.WhDroneRechargePerTimeStep)
        print("droneImageFile:\t", self.myDt.droneImageFile)
        print("droneColour:\t", self.myDt.droneColour)
        print("droneWidth:\t", self.myDt.droneWidth)
        print("droneHeight:\t", self.myDt.droneHeight)
        print("droneMperSec:\t", self.myDt.droneMperSec)
        print("droneStepMperTimeStep:\t", self.myDt.droneStepMperTimeStep)
        print("droneStepM2:\t", self.myDt.droneStepM2)
        print("minDroneCharge:\t", self.myDt.minDroneCharge)
        print("minDroneFlyingWh:\t", self.myDt.minDroneFlyingWh)
        print("viableDroneCharge:\t", self.myDt.viableDroneCharge)
        print("viableDroneFlyingWh:\t", self.myDt.viableDroneFlyingWh, "\n")


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
        traci.vehicletype.setMinGap("Drone","0.1")
        traci.vehicletype.setParameter("Drone", "has.battery.device", "True")
        traci.vehicletype.setEmissionClass("Drone", "Energy/unknown")
        Drone.dummyEVCreated = True

    @classmethod
    def setDroneType(cls, droneType="ehang184"):
        """Support different drone definitions - initially to give us a drone that doesn't need charging"""

        Drone.d0Type.droneKMperh = GG.getDroneSpeed()  # get speed override if any

        # EV charging battery size is constrained by drone carrying capacity * average battery energy density (currently ~150Wh/Kg)
        match droneType:
            case "ehang184":
                Drone.d0Type.droneChargeWh = 30000.                      # capacity of battery used to charge ev's, based on Ehang 184 load capacity - 200Kg
                Drone.d0Type.droneFlyingWh = 14400.                      # capacity of battery used to power drone
                Drone.d0Type.droneFlyingWhperTimeStep = Drone.d0Type.droneFlyingWh / (23 * 60.)   # Ehang 184 has battery capacity of 14.4 KW giving 23 mins flight time

                Drone.d0Type.droneChargeContingencyp = 0.05              # minimum contingency level %
                Drone.d0Type.droneChargeViablep = 0.3                    # minimum viable level %
                Drone.d0Type.WhEVChargeRatePerTimeStep = 25000 / 3600.   # 25KW   rate of vehicle charge from drone  (adjust for timeStep when simulation starts)
                Drone.d0Type.WhDroneRechargePerTimeStep = 75000 / 3600.  # 75KW   rate of drone charge when parked  (adjust for timeStep when simulation starts)

            case "ehang184x":            # ehang 184 with artificially increased battery sizes so they don't need recharging
                Drone.d0Type.droneChargeWh = 3000000.     # 100 * actual
                Drone.d0Type.droneFlyingWh = 14400000.
                Drone.d0Type.droneFlyingWhperTimeStep = 14400. / (23 * 60.)
                Drone.d0Type.droneChargeContingencyp = 0.05              # minimum contingency level %
                Drone.d0Type.droneChargeViablep = 0.3                    # minimum viable level %
                Drone.d0Type.WhEVChargeRatePerTimeStep = 25000 / 3600.   # 25KW   rate of vehicle charge from drone  (adjust for timeStep when simulation starts)
                Drone.d0Type.WhDroneRechargePerTimeStep = 75000 / 3600.  # 75KW   rate of drone charge when parked  (adjust for timeStep when simulation starts)

        Drone.d0Type.droneMperSec = Drone.d0Type.droneKMperh / 3.6
        Drone.d0Type.droneStepMperTimeStep = Drone.d0Type.droneMperSec                            # How far (metres) the drone will travel in one time step (adjust for timeStep when simulation starts)
        Drone.d0Type.droneStepM2 = Drone.d0Type.droneStepMperTimeStep * Drone.d0Type.droneStepMperTimeStep     # precompute - used in distance calculations  (adjust for timeStep when simulation starts)
        Drone.d0Type.minDroneCharge = Drone.d0Type.droneChargeContingencyp    * Drone.d0Type.droneChargeWh     # Thresholds to break off charging/flying
        Drone.d0Type.minDroneFlyingWh = Drone.d0Type.droneChargeContingencyp  * Drone.d0Type.droneFlyingWh
        Drone.d0Type.viableDroneCharge = Drone.d0Type.droneChargeViablep      * Drone.d0Type.droneChargeWh     # thresholds to allow allocation - ie enough charge to be useful
        Drone.d0Type.viableDroneFlyingWh = Drone.d0Type.droneChargeViablep    * Drone.d0Type.droneFlyingWh

    @classmethod
    def setDroneTypeFromPOI(cls, zeroDrone):
        """ Update the default DroneType - d0Type , containing drone behavior varuables
               from a definition in an additional file - if it exists.
             Then if the --z option is set create drones from definitions in the file"""
        POIlist = traci.poi.getIDList()
        if len(POIlist) > 0:
            for poi in POIlist:
                if poi == "d0":
                    dWidth = int(traci.poi.getWidth(poi))
                    if dWidth > 1: Drone.d0Type.droneWidth = dWidth
                    dHeight = int(traci.poi.getHeight(poi))
                    if dHeight > 1: Drone.d0Type.droneHeight = dHeight
                    dColor = traci.poi.getColor(poi)
                    if len(dColor) > 1: Drone.d0Type.droneColour = dColor
                    dImageFile = traci.poi.getImageFile(poi)
                    if len(dImageFile) > 1: Drone.d0Type.droneImageFile = dImageFile

                    dDroneKMperh = traci.poi.getParameter(poi, "droneKMperh")
                    if len(dDroneKMperh) > 1: Drone.d0Type.droneKMperh = float(dDroneKMperh)

                    dDroneChargeWh = traci.poi.getParameter(poi, "droneChargeWh")
                    if len(dDroneChargeWh) > 1: Drone.d0Type.droneChargeWh = float(dDroneChargeWh)

                    dDroneFlyingWh = traci.poi.getParameter(poi, "droneFlyingWh")
                    if len(dDroneFlyingWh) > 1: Drone.d0Type.droneFlyingWh = float(dDroneFlyingWh)

                    dDroneFlyingMinutes = traci.poi.getParameter(poi, "droneFlyingMinutes")
                    if len(dDroneFlyingMinutes) > 1: Drone.d0Type.droneFlyingWhperTimeStep = Drone.d0Type.droneFlyingWh /(60. * int(dDroneFlyingMinutes))

                    dDroneChargeContingencyp = traci.poi.getParameter(poi, "droneChargeContingencyp")
                    if len(dDroneChargeContingencyp) > 1: Drone.d0Type.droneChargeContingencyp = float(dDroneChargeContingencyp)

                    dDroneChargeViablep = traci.poi.getParameter(poi, "droneChargeViablep")
                    if len(dDroneChargeViablep) > 1: Drone.d0Type.droneChargeViablep = float(dDroneChargeViablep)

                    dWhEVChargeRate = traci.poi.getParameter(poi, "WhEVChargeRate")
                    if len(dWhEVChargeRate) > 1: Drone.d0Type.WhEVChargeRatePerTimeStep = int(dWhEVChargeRate)/3600.

                    dWhDroneRechargeRate = traci.poi.getParameter(poi, "WhDroneRechargeRate")
                    if len(dWhDroneRechargeRate) > 1: Drone.d0Type.WhDroneRechargePerTimeStep = int(dWhDroneRechargeRate)/3600.

                    traci.poi.remove(poi)
                    if not zeroDrone:
                        return 1  # ie we've set the d0Type so can return

        poiDroneCount = 0
        POIlist = traci.poi.getIDList()
        if zeroDrone and len(POIlist) > 0:
            for poi in POIlist:
                if traci.poi.getType(poi) == "drone" :  # weve removed the d0 drone
                    DT = DroneType()

                    dWidth = int(traci.poi.getWidth(poi))
                    if dWidth > 1: DT.droneWidth = dWidth
                    dHeight = int(traci.poi.getHeight(poi))
                    if dHeight > 1: DT.droneHeight = dHeight
                    dColor = traci.poi.getColor(poi)
                    if len(dColor) > 1: DT.droneColour = dColor
                    dImageFile = traci.poi.getImageFile(poi)
                    if len(dImageFile) > 1: DT.droneImageFile = dImageFile

                    dDroneKMperh = traci.poi.getParameter(poi, "droneKMperh")
                    if len(dDroneKMperh) > 1: DT.droneKMperh = float(dDroneKMperh)

                    dDroneChargeWh = traci.poi.getParameter(poi, "droneChargeWh")
                    if len(dDroneChargeWh) > 1: DT.droneChargeWh = float(dDroneChargeWh)

                    dDroneFlyingWh = traci.poi.getParameter(poi, "droneFlyingWh")
                    if len(dDroneFlyingWh) > 1: DT.droneFlyingWh = float(dDroneFlyingWh)

                    dDroneFlyingMinutes = traci.poi.getParameter(poi, "droneFlyingMinutes")
                    if len(dDroneFlyingMinutes) > 1: DT.droneFlyingWhperTimeStep = DT.droneFlyingWh /(60. * int(dDroneFlyingMinutes))

                    dDroneChargeContingencyp = traci.poi.getParameter(poi, "droneChargeContingencyp")
                    if len(dDroneChargeContingencyp) > 1: DT.droneChargeContingencyp = float(dDroneChargeContingencyp)

                    dDroneChargeViablep = traci.poi.getParameter(poi, "droneChargeViablep")
                    if len(dDroneChargeViablep) > 1: DT.droneChargeViablep = float(dDroneChargeViablep)

                    dWhEVChargeRate = traci.poi.getParameter(poi, "WhEVChargeRate")
                    if len(dWhEVChargeRate) > 1: DT.WhEVChargeRatePerTimeStep = int(dWhEVChargeRate)/3600.

                    dWhDroneRechargeRate = traci.poi.getParameter(poi, "WhDroneRechargeRate")
                    if len(dWhDroneRechargeRate) > 1: DT.WhDroneRechargePerTimeStep = int(dWhDroneRechargeRate)/3600.

                    try:
                        pos = traci.poi.getPosition(poi)
                        GG.cc.freeDrones.add(Drone(pos, poi, DT))
                        poiDroneCount += 1
                    except traci.TraCIException:
                        print("Drone ",poi," creation failed. :- ", traci.TraCIException)

            return poiDroneCount

        return 0

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
        if self.myCharge < self.myDt.droneChargeWh:
            self.myCharge += self.myDt.WhDroneRechargePerTimeStep
            self.myChargeMeCount += 1
        if self.myFlyingCharge < self.myDt.droneFlyingWh:
            self.myFlyingCharge += self.myDt.WhDroneRechargePerTimeStep
            self.myChargeMeFlyingCount += 1
        elif self.myCharge >= self.myDt.droneChargeWh:  # fully charged so move to null state, avoiding calls to chargeMe
            # self.myCharge = self.myDt.droneChargeWh
            # self.myFlyingCharge = self.myDt.droneFlyingWh
            self.dummyEVHide()
            self.myState = Drone.DroneState.NULLState
            self.setViableCharge()

    def dummyEVHide(self):
        """remove the dummy EVs   - we need to resume before remove to avoid the aborted stop warning However
            insertion may have collided and teleported/removed the EV so we need to check the list maintained by simulation before we try"""
        if GG.ss.useChargeHubs and self.myDummyEVInserted:
            dummyFB = self.myID + "-FB"
            try:
                stState = traci.vehicle.getStopState(dummyFB)
                if stState & 2 == 2:                          # only need to resume if it's actually stopped
                    traci.vehicle.resume(dummyFB)
            except Exception as e:
                pass
                #print("resume except ",e)  - was doing a resums when parking had not actually happened - may not need this now
            finally:
                traci.vehicle.remove(dummyFB)
                GG.cc.insertedDummies -= 1

            dummyCB = self.myID + "-CB"
            try:
                stState = traci.vehicle.getStopState(dummyCB)
                if stState & 2 == 2:
                    traci.vehicle.resume(dummyCB)
            except Exception as e:
                pass
                # print("resume except ",e)
            finally:
                traci.vehicle.remove(dummyCB)
                GG.cc.insertedDummies -= 1

            self.myDummyEVInserted = False

    def dummyEVInsert(self):
        """If we are generating charge station output add dummy EVs to the charge station for the drone batteries - whilst the drone is there"""
        if GG.ss.useChargeHubs:
            e,p = self.myParkEP
            dummyFB = self.myID + "-FB"
            traci.vehicle.add(dummyFB,e,"Drone",departLane=0,departPos=p)
            traci.vehicle.setParameter(dummyFB, "device.battery.maximumBatteryCapacity", self.myDt.droneFlyingWh)
            traci.vehicle.setParameter(dummyFB, "device.battery.actualBatteryCapacity", self.myFlyingCharge)
            traci.vehicle.setEmissionClass(dummyFB, "Energy/unknown")
            traci.vehicle.setStop(dummyFB, e, pos=p, duration=10000.0, flags=1)
            GG.cc.insertedDummies += 1

            dummyCB = self.myID + "-CB"
            traci.vehicle.add(dummyCB,e,"Drone",departLane=0,departPos=p+0.5)
            traci.vehicle.setParameter(dummyCB, "device.battery.maximumBatteryCapacity", self.myDt.droneChargeWh)
            traci.vehicle.setParameter(dummyCB, "device.battery.actualBatteryCapacity", self.myCharge)
            traci.vehicle.setEmissionClass(dummyCB, "Energy/unknown")
            traci.vehicle.setStop(dummyCB, e, pos=p + 0.5, duration=10000.0, flags=1)
            GG.cc.insertedDummies += 1

            self.myDummyEVInserted = True


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
        if  ddx == 0 or ddy == 0 :
            if ddx == 0:
                if ddy == 0:
                    x = 0.
                    y = 0.
                else:
                    x = 0.
                    y = abs(ddy)
            else: # ddy == 0
                    x = abs(ddx)
                    y = 0.0

        else:
            x = abs(math.sqrt(self.myDt.droneStepM2 / (1.0 + pow(ddy/ddx ,2))))
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

    def getIDCount(self):
        """ getter for idcount!"""
        return Drone.droneIDCount

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
              (GG.ss.timeStep, self.myID, evID, lane, lanePos, x, y, self.myEVChargingCount * self.myDt.WhEVChargeRatePerTimeStep , self.myCharge, self.myFlyingCharge, activity), file=GG.droneLog)

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
                    self.dummyEVHide()
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
            case Drone.DroneState.FLYINGTOPARK | Drone.DroneState.FLYINGTOCHARGE | Drone.DroneState.PARKED | Drone.DroneState.CHARGINGDRONE | Drone.DroneState.NULLState:
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
        if self.myCharge >= self.myDt.viableDroneCharge and self.myFlyingCharge >= self.myDt.viableDroneFlyingWh:
            if not self.myViableCharge:
                self.myViableCharge = True
                GG.cc.notifyDroneState(self)  # cc only interested when we become viable
                traci.poi.setColor(self.myID, self.myDt.droneColour)
        else:
            self.myViableCharge = False

    def update(self, pos):
        """primary update - invoked directly when EV is managing drone"""
        updateStatus = True
        updatePower = 0.0

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
                    updatePower = self.myDt.WhEVChargeRatePerTimeStep
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

            case Drone.DroneState.NULLState:  # am charged so do nothing until allocated
                pass

        return updateStatus, updatePower

    def usePower(self, mode):
        """Am flying or charging an EV so adjust my charge levels"""
        breakOff = False
        match mode:
            case "fly":
                self.myFlyingCharge -= self.myDt.droneFlyingWhperTimeStep
                self.myFlyingCount += 1
                if self.myFlyingCharge < self.myDt.minDroneFlyingWh:
                    breakOff = True

            case "chargeEV":
                self.myCharge -= self.myDt.WhEVChargeRatePerTimeStep
                self.myRequestedCharge -= self.myDt.WhEVChargeRatePerTimeStep
                self.myEVChargingCount += 1
                if self.myRequestedCharge <= 0:    #  we've charged the requested amount
                    self.myEV.stopCharging(0)   # don't clear, full charge
                    self.myFullCharges += 1
                    self.myRequestedCharge = 0
                    self.park()
                    return False      # ie we're not returning a charge
                if self.myCharge < self.myDt.minDroneCharge:
                    breakOff = True
            case _:
                if self.myState == Drone.DroneState.FLYINGTOCHARGE:       # we're flying back to charge, only time we get here for now
                    self.myFlyingCount += 1
                    self.myFlyingCharge -= self.myDt.droneFlyingWhperTimeStep

        # problem - one of my batteries below contingency
        if breakOff:
            if self.myEV is not None:
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
