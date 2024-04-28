"""Module managing allocation of Drones to EVs and control (parking, charging) of Drones when not assigned to EVs"""
import sys
import math
from datetime import datetime
import traci

from GlobalClasses import GlobalClasses as GG
from EV import EV
from Drone import Drone

class ControlCentre:
    """Main class receiving requests from EV's and notifications from Drones and EV's when charge completes or Drone is out of battery"""

    def __init__(self, wEnergy, wUrgency, proximityRadius, maxDrones, fullChargeTolerance=0, droneType="ehang184"):
        self.wEnergy = float(wEnergy)
        self.wUrgency = float(wUrgency)
        self.proximityRadius = proximityRadius
        self.maxDrones = maxDrones
        self.fullChargeTolerance = fullChargeTolerance

        self.requests = {}
        self.allocatedEV = {}
        self.startChargeEV = {}
        self.allocatedDrone = {}
        self.freeDrones = set()
        self.needChargeDrones = set()
        self.droneType = droneType

        self.spawnedDrones = 0
        self.insertedDummies = 0


    def allocate(self, drone, ev):
        """Set up the mapping between drone and ev"""
        self.allocatedEV[ev] = drone
        self.allocatedDrone[drone] = ev
        if GG.modelRendezvous:
            ev.allocate(drone, self.findRendezvousXY(ev, drone))
        else:
            ev.allocate(drone, None)
        requestedWh = self.requests[ev]
        drone.allocate(ev, requestedWh)
        del self.requests[ev]
        #print("allocation", GG.ss.timeStep, drone.getID(), ev.getID())

    def allocateDrones(self, urgencyList):
        """Allocate whatever drones we have free/viable in order of urgency,
            if more than one drone available assign the nearest"""
        ld = len(self.freeDrones)
        nd = self.maxDrones - self.spawnedDrones
        if ld == 1:
            for ev in dict(sorted(urgencyList.items(), key=lambda item: item[1])):
                if self.chargeCanComplete(ev):
                    self.allocate(self.freeDrones.pop(), ev)
                    break

                del self.requests[ev]

        elif ld < 1 and nd > 0:
            for ev in dict(sorted(urgencyList.items(), key=lambda item: item[1])):
                evPos = ev.getMyPosition()
                (x, y, e, p), hubDistance = GG.ch.nearestHubLocation(evPos)
                drone = Drone((x, y), "", None)
                self.spawnedDrones += 1
                self.allocate(drone, ev)
                nd -= 1
                if nd <= 0:
                    break

        elif ld > 1:  # need to find drone nearest the ev before we allocate
            for ev in dict(sorted(urgencyList.items(), key=lambda item: item[1])):
                if self.chargeCanComplete(ev):
                    evPos = ev.getMyPosition()
                    nearestDrone = None
                    evDistance = sys.float_info.max
                    droneID = ""
                    for drone in self.freeDrones:
                        dronePos = drone.getMyPosition()
                        distance = math.dist(evPos, dronePos)
                        if distance < evDistance or (distance == evDistance and drone.getID() > droneID):
                            nearestDrone = drone
                            evDistance = distance
                            droneID = drone.getID()
                    if nearestDrone is not None:
                        self.allocate(nearestDrone, ev)
                        self.freeDrones.remove(nearestDrone)
                    elif nd > 0:
                        (x, y, e, p), hubDistance = GG.ch.nearestHubLocation(evPos)
                        drone = Drone((x, y),"", None)
                        self.spawnedDrones += 1
                        self.allocate(drone, ev)
                        nd -= 1
                        if nd <= 0:
                            break
                else:
                    del self.requests[ev]


        else:   # ld == 0 nd >0
            for ev in dict(sorted(urgencyList.items(), key=lambda item: item[1])):
                evPos = ev.getMyPosition()
                (x, y, e, p), hubDistance = GG.ch.nearestHubLocation(evPos)
                drone = Drone((x, y), "", None)
                self.spawnedDrones += 1
                self.allocate(drone, ev)
                nd -= 1
                if nd <= 0:
                    break

    def calcUrgency(self):
        """urgency defined as distance to nearest hub/distance ev can travel on charge.
            creates a list of ev's that want charge, have not been allocated a drone
        """
        urgencyList = {}
        proximityList = {}
        urgencySum = 0.0
        proximitySum = 0.0

        if len(self.requests) == 1:    #  only one request so return that
          for ev in self.requests:
            urgencyList[ev] = 1.0

        else:
            firstCall = True
            for ev in self.requests:
                # note drivingDistance can be very large -  float max if there is no hub on the remaining route
                evID = ev.getID()
                if firstCall:
                    ev.setMyPosition()
                evPos = ev.getMyPosition()

                hub, hubDistance = GG.ch.findNearestHub(evPos[0], evPos[1])
                # hub, hubDistance = GG.ch.findNearestHubDriving(evID)

                evRange = 10000.0         # arbitrary default to make this visible outside if statement
                if self.wUrgency > 0.0:  # if we have an ugency weight then we need to calculate the range
                    distance = float(traci.vehicle.getDistance(evID))
                    if distance > 10000:  # can compute real range after we've been driving for a while - arbitrary 10km
                        mWh = distance / float(traci.vehicle.getParameter(evID, "device.battery.totalEnergyConsumed"))
                        evRange = float(traci.vehicle.getParameter(evID, "device.battery.actualBatteryCapacity")) * mWh / 1000.
                    else:  #  otherwise just a guesstimate
                        evRange = float(traci.vehicle.getParameter(evID, "device.battery.actualBatteryCapacity")) * ev.getMyKmPerWh()
                    urgency = hubDistance/evRange

                    urgencyList[ev] = urgency
                    urgencySum += urgency

                else:                      # for corner case where both weights are <= 0.0
                    urgencyList[ev] = 0.0

                if self.wEnergy > 0.0:            # We have a weight so need to calculate proximity
                    neighbours, meanDist = self.getNeighboursNeedingCharge(ev, firstCall)
                    firstCall = False              # getneighbours will have set position for all evs

                    # find distance for nearest drone to this eV - usually only one drone so will be the one allocated
                    droneDist = self.proximityRadius    # default - should never happen - otherwise fn wouldn't be called
                    if len(self.freeDrones) > 0:
                        droneDist = sys.float_info.max
                        for drone in self.freeDrones:
                            dist = math.dist(evPos, drone.getMyPosition())
                            if dist < droneDist:
                                droneDist = dist

                    # proximity factors - smallest value is most important = 'nearest
                    if len(neighbours) > 1:
                        meanDist /= len(neighbours)   # set distance  'smaller' the more neighbours there are
                    del neighbours

                    # add in drone distance - ie smallest proximity will have closest drone
                    if hubDistance == sys.float_info.max:
                       proximity = droneDist + meanDist  # + evRange
                    else:
                       proximity = droneDist + meanDist  # / drivingDistance

                    proximityList[ev] = proximity
                    proximitySum += proximity

            if self.wEnergy <= 0.0:
                return urgencyList
            if self.wUrgency <= 0.0:
                return proximityList

            #  non zero values for both so need to normalise
            evCount = len(urgencyList)
            proximityWt = self.wEnergy / (proximitySum/evCount)
            urgencyWt = self.wUrgency / (urgencySum/evCount)

            for ev in proximityList:
                CEC = (proximityList[ev] * proximityWt) + (urgencyList[ev] * urgencyWt)
                urgencyList[ev] = CEC

        return urgencyList

    def chargeCanComplete(self, ev):
        """ Estimate whether there will be time for the charge to complete """
        if self.fullChargeTolerance <= 0:    # don't care whether it will complete
            return True

        evID = ev.getID()
        evRoute = traci.vehicle.getRoute(evID)
        lastEdge = evRoute[len(evRoute)-1]
        dDist = traci.vehicle.getDrivingDistance(evID,lastEdge,0) # get distance to start of last edge in route (length of edge "ignored"as tolerance)
        evSpeed = 0.9 * traci.vehicle.getAllowedSpeed(evID)       # use same speed estimate as in find rendezvousxy
        availableChargeTime = (dDist/evSpeed) - self.fullChargeTolerance
        requestedWh = self.requests[ev]
        possibleCharge = Drone.d0Type.WhEVChargeRatePerTimeStep * availableChargeTime

        return  possibleCharge > requestedWh


    def findEdgePos(self, evID, deltaPos):
        """work out the edge and position of the EV, when it is deltaPos metres along the route from the current position
            to give us an approximation to the rendezvous position
        """
        jnDelta = 150          # no of travel metres 'lost' by vehicle crossing a junction, found by testing against random grid  (ie allowance for vehicle slowing/accelerating)
        # find route and position of vehicle along the route - (this will always give us an edge)
        evRoute = traci.vehicle.getRoute(evID)
        idx = traci.vehicle.getRouteIndex(evID)
        edge = evRoute[idx]
        lanePosition = traci.vehicle.getLanePosition(evID)
        if deltaPos < 0:
            print("oops invalid call to findEdgePos:", deltaPos)
            deltaPos = 0
        # whilst we have the edge from above the vehicle could actually be on a junction which messes up the edge to edge calculation
        # so we get current road ID which can be a junction and if it is then skip along the route to the next edge
        road = traci.vehicle.getRoadID(evID)
        if road != edge:       # ev is on a junction, set to start of next edge
          idx += 1
          edge = evRoute[idx]
          lanePosition = 0

        # 'travel' along edges on route until we've gone  deltaPos metres
        newEVPosition = lanePosition + deltaPos
        laneLength = traci.lane.getLength(edge + '_0')
        while newEVPosition > laneLength + jnDelta:
            newEVPosition -= (laneLength + jnDelta)    # subtract jnDelta as the penalty in distance travelled, in the total travel time, for each junction
            idx += 1
            if idx >= len(evRoute):                 # rv point is after end of route - so we can't charge
                return evRoute[idx-1], laneLength, False   # set to end of route
            edge = evRoute[idx]
            laneLength = traci.lane.getLength(edge + '_0')

        newEVPosition = min(newEVPosition, laneLength)

        return edge, newEVPosition, True

    def findRendezvousXY(self, ev, drone):
        """estimate a direct rendezvous point for the drone/vehicle - assumes constant vehicle speed
              apply a factor of 90% to allow for acceleration/deceleration/% of time not at allowed speed
           algorithm from https://www.codeproject.com/Articles/990452/Interception-of-Two-Moving-Objects-in-D-Space
        """
        evID = ev.getID()         # needed for Traci calls
        # assume speed on current edge is that for subsequent edges
        evSpeed = 0.9 * traci.vehicle.getAllowedSpeed(evID)

        # work out how long it takes drone to fly to ev
        posEV = ev.getMyPosition()
        posDrone = drone.getMyPosition()
        distanceToEV = math.dist(posDrone, posEV)
        crowFlies = distanceToEV/drone.myDt.droneMperSec
        # how far vehicle can travel in same time
        evCrowFlies = evSpeed * crowFlies
        # where on the road that distance is
        vEdge, vPos, valid = self.findEdgePos(evID, evCrowFlies)
        if valid:
            posRV = traci.simulation.convert2D(vEdge, vPos)        # get the x, y position after evCrowFlies metres
            # compute the velocity vector
            evV = (posRV[0] - posEV[0])/crowFlies, (posRV[1] - posEV[1])/crowFlies

            vectorFromEV = posDrone[0] - posEV[0], posDrone[1] - posEV[1]
            a = pow(drone.myDt.droneMperSec,2) - pow(evSpeed,2)
            b = 2 * (vectorFromEV[0]*evV[0] + vectorFromEV[1] * evV[1])
            c = -distanceToEV * distanceToEV
            bb4ac = (b * b) - (4 * a * c)
            t1 = (-b + math.sqrt(bb4ac))/(2*a)
            t2 = (-b - math.sqrt(bb4ac))/(2*a)

            if t1 < 0 and t2 < 0:   # no solution we can't meet the EV
                return posDrone
            if t1 > 0 and t2 > 0:  # both positive take the smallest
                interceptDistance = min(t1, t2) * evSpeed
            else:  # one is -ve so take the maximum
                interceptDistance = max(t1, t2) * evSpeed

            rendezvousEdge, newEVPosition, valid = self.findEdgePos(evID, interceptDistance)
            if valid:       # will normally only fail if drone cannot reach EV under straight line intercept assumptions
                posRV = traci.simulation.convert2D(rendezvousEdge, newEVPosition)      # get the x, y position after evCrowFlies metres
                # Algorithm debug lines - show rendezvous point
                # pid = ev + " " + str(timeStep)
                # traci.poi.add(pid, posRV[0], posRV[1], color=(255, 0, 255), layer=250, imgFile=".\\PNG132.bmp", width=5, height=5)
            return posRV   # this falls back to the original crow flies when straight line intercept fails

        # print(timeStep, ev, drone, evCrowFlies, "fail 2")  'fail' usually because vehicle has left simulation
        return posDrone   # revert to direct intercept

    def getNeighboursNeedingCharge(self, ev, firstCall):
        """find all the ev's that are requesting a charge and compute the mean distance to these
              note calling math.dist which will use sqrt is actually faster than comparing distances to the square
              we only update the ev positions on first call because we will repeat call to this fn for each ev in creating urgency list
        """
        neighbours = []
        meanDist = 0.0
        if firstCall:
            ev.setMyPosition()
        evPos = ev.getMyPosition()

        for nEV in self.requests:
            if nEV == ev:
                continue
            try:
                if firstCall:
                    nEV.setMyPosition()
                vPos = nEV.getMyPosition()

                xdist = math.dist(evPos, vPos)
                if xdist < self.proximityRadius:
                    neighbours.append(nEV)
                    meanDist += xdist
            except traci.TraCIException:
                print("neighbour exception:", traci.TraCIException)
                continue

        if meanDist > 0.0:                      # we have at least 1 ev so calculate the actual mean distance
            meanDist = meanDist / len(neighbours)
        return neighbours, meanDist

    def notifyDroneState(self, drone):
        """Notification from Drone when charging finished or Drone has broken off the charge/flight"""
        if drone in self.allocatedDrone:
            del self.allocatedEV[self.allocatedDrone[drone]]
            del self.allocatedDrone[drone]
        if drone.viable():
            self.freeDrones.add(drone)
            if drone in self.needChargeDrones:
                self.needChargeDrones.remove(drone)
        else:
            self.needChargeDrones.add(drone)

    def notifyEVState(self, ev, evState, drone, capacity):
        """Notification from EV - when EV has left simulation (or completed charge)"""
        charge = 0.0
        match evState:
            case EV.EVState.DRIVING:            # means charge complete  so calculate charge
                if ev.getID() in self.startChargeEV:
                    charge = (GG.ss.timeStep - self.startChargeEV[ev.getID()]) * drone.myDt.WhEVChargeRatePerTimeStep
                    del self.startChargeEV[ev.getID()]

            case EV.EVState.CHARGEREQUESTED:    # just log request
                pass

            case EV.EVState.CHARGEBROKENOFF:    # drone broke off charge so should have an entry in self.startChargeEV
                if ev.getID() in self.startChargeEV:
                    charge = (GG.ss.timeStep - self.startChargeEV[ev.getID()]) * drone.myDt.WhEVChargeRatePerTimeStep
                    del self.startChargeEV[ev.getID()]

            case EV.EVState.CHARGINGFROMDRONE:      # charge started
                self.startChargeEV[ev.getID()] = GG.ss.timeStep

            case EV.EVState.LEFTSIMULATION:
                if ev.getID() in self.startChargeEV:
                    if self.startChargeEV[ev.getID()] > 0:
                        if drone is not None:
                            charge = (GG.ss.timeStep - self.startChargeEV[ev.getID()]) * drone.myDt.WhEVChargeRatePerTimeStep
                        else:
                            charge = (GG.ss.timeStep - self.startChargeEV[ev.getID()]) * Drone.d0Type.WhEVChargeRatePerTimeStep
                if ev in self.requests:
                    del self.requests[ev]
                elif ev in self.allocatedEV:
                    del self.allocatedDrone[self.allocatedEV[ev]]
                    del self.allocatedEV[ev]

            case EV.EVState.WAITINGFORRENDEZVOUS:   # don't see this yet
                print(GG.ss.timeStep, "rv")

            case EV.EVState.WAITINGFORDRONE:        # don't see this
                print(GG.ss.timeStep, "wd")

        if GG.chargePrint:
            droneID = "-"
            if drone is not None:
                droneID = drone.getID()

            print("{}\t{}\t{!r}\t{}\t{:.1f}\t{:.1f}".format(GG.ss.timeStep, ev.getID(), evState, droneID, capacity, charge), file=GG.chargeLog)

    def printDroneStatistics(self, brief, version, runstring):
        """Print out Drone and EV statistics for the complete run"""
        # compute drone statistic totals
        tmyFlyingCount = 0           # used to compute distance travelled
        tmyFullCharges = 0           # count of complete charges
        tmyBrokenCharges = 0         # count of charges broken off - either by me out of charge
        tmyBrokenEVCharges = 0       # count of charges broken off - by EV leaving
        tmyFlyingKWh = 0.0           # wH i've used flying
        tmyChargingKWh = 0.0         # wH i've used charging EVs
        tmyChargeMeFlyingCount = 0   # wh i've charged my flying battery
        tmyChargeMeCount = 0         # wh i've used charging my EV charging battery
        tmyResidualChargeKWh = 0.0   # Wh left in charge battery at end
        tmyResidualFlyingKWh = 0.0   # Wh left in flying battery at end
        tmyChaseCount = 0            # no of successful chases  (ie successfully got from rendezvous to EV)
        tmyBrokenChaseCount = 0      # no of broken chases  (vehicle left after rendezvous but before drone got there)
        tmyChaseSteps = 0            # steps for succesful chases - used to compute average chase time

        tDroneDistance = 0.0
        tmyChargeMeFlyingKWh = 0.0
        tmyChargeMeKWh = 0.0

        for drone in self.freeDrones | self.needChargeDrones | set(self.allocatedDrone):
            tmyFlyingCount          += drone.myFlyingCount
            tmyFlyingKWh            += drone.myFlyingCount * drone.myDt.droneFlyingWhperTimeStep

            tDroneDistance          += drone.myFlyingCount * drone.myDt.droneStepMperTimeStep

            tmyFullCharges          += drone.myFullCharges
            tmyBrokenCharges        += drone.myBrokenCharges
            tmyBrokenEVCharges      += drone.myBrokenEVCharges

            tmyChargingKWh          += drone.myEVChargingCount * drone.myDt.WhEVChargeRatePerTimeStep
            tmyChargeMeFlyingCount  += drone.myChargeMeFlyingCount
            tmyChargeMeFlyingKWh    += drone.myChargeMeFlyingCount * drone.myDt.WhDroneRechargePerTimeStep
            tmyChargeMeCount        += drone.myChargeMeCount
            tmyChargeMeKWh          += drone.myChargeMeCount * drone.myDt.WhDroneRechargePerTimeStep

            tmyResidualFlyingKWh    += drone.myFlyingCharge
            tmyResidualChargeKWh    += drone.myCharge
            if GG.modelRendezvous:
                tmyChaseCount += drone.myChaseCount
                tmyBrokenChaseCount += drone.myBrokenChaseCount
                tmyChaseSteps += drone.myChaseSteps

        # alternative computation - avoiding errors from addition of large no of floating point
        # tmyFlyingKWh = tmyFlyingCount * Drone.droneFlyingWhperTimeStep / 1000.
        tDroneDistance        /= 1000.
        tmyFlyingKWh          /= 1000.
        tmyChargingKWh        /= 1000.
        tmyChargeMeFlyingKWh  /= 1000.
        tmyChargeMeKWh        /= 1000.
        tmyResidualFlyingKWh  /= 1000.
        tmyResidualChargeKWh  /= 1000.

        if GG.modelRendezvous:
            # note chases + broken chases usually less than charge sessions total because some will break off before they get to rendezvous
            # ie chases are between rendezvous point and beginning charging - indicator of efficiency of rendezvous algorithm
            if tmyChaseCount > 0:
                averageChase = (tmyChaseSteps * GG.ss.stepSecs)/tmyChaseCount
            else:
                averageChase = 0

        timeStamp = datetime.now().isoformat()
        # all done, dump the distance travelled by the drones and KW used
        print("\t")   #  tidy up after the ....
        if brief:
            sumoVersion = traci.getVersion()
            print("Date\tRv\tOnce\tOutput\twE\twU\tradius\tSteps\t# Drones"
                  "\tDistance\tFlyKWh\tchKWh\tFlyChgKWh\tChgKWh\trFlyKWh\trChKWh"
                  "\t# EVs\tEVChg\tEVgap\tFull\tbrDrone\tbrEV\tChases\tAvg Chase\tBrk Chase")
            print("{}\t{!r}\t{!r}\t{!r}\t{:.1f}\t{:.1f}\t{:.0f}\t{:.1f}".format
                  (timeStamp, GG.modelRendezvous, GG.onlyChargeOnce,
                   GG.dronePrint, self.wEnergy, self.wUrgency, self.proximityRadius, GG.ss.timeStep), end='')
            print("\t%i\t%.2f\t%.2f\t%.2f" % (self.spawnedDrones, tDroneDistance, tmyFlyingKWh, tmyChargingKWh), end="")
            print("\t{:.2f}\t{:.2f}\t{:.2f}\t{:.2f}".format
                  (tmyChargeMeFlyingKWh, tmyChargeMeKWh, tmyResidualFlyingKWh, tmyResidualChargeKWh), end="")
            print("\t%i\t%.2f\t%.2f" %
                  (EV.evCount, EV.evChargeSteps * Drone.d0Type.WhEVChargeRatePerTimeStep/1000., EV.evChargeGap/(1000. * EV.evCount)), end="")
            print("\t{:.0f}\t{:.0f}\t{:.0f}".format(tmyFullCharges, tmyBrokenCharges, tmyBrokenEVCharges), end="")

            if GG.modelRendezvous:
                print("\t{}\t{:.2f}\t{}\t{}\t{}\t{}".format(tmyChaseCount, averageChase, tmyBrokenChaseCount, runstring, version, sumoVersion))
            else:
                print("\t\t\t\t{}\t{}\t{}".format(runstring, version, sumoVersion))

        else:
            print("\nSummary Statistics:\t\t", timeStamp)
            print("\tModel flags:\tRendezvous: {!r}\tCharge Once: {!r}\tDrone print: {!r}\tOne Battery: {!r}\n\tEnergy Wt: "
                  "{:.1f}\tUrgency Wt: {:.1f}\t\tProximity radius(m): {:.0f}\tSteps: {:.1f}\tTolerance(s): {:.0f}".format
                  (GG.modelRendezvous, GG.onlyChargeOnce, GG.dronePrint, Drone.d0Type.useOneBattery, self.wEnergy,
                   self.wUrgency, self.proximityRadius, GG.ss.timeStep, self.fullChargeTolerance))
            print("\n\tDrone Totals:\t(%i drones)\n\t\tDistance Km:\t%.2f\n\t\tFlying KWh:\t%.2f\n\t\tCharging KWh:\t%.2f" %
                  (self.spawnedDrones, tDroneDistance, tmyFlyingKWh, tmyChargingKWh))
            print("\tDrone Charger usage:\n\t\tFlying KWh:\t{:.2f}\n\t\tCharge KWh:\t{:.2f}".format
                  (tmyChargeMeFlyingKWh, tmyChargeMeKWh))
            print("\tResiduals:\n\t\tFlying KWh:\t{:.1f}\n\t\tCharging KWh:\t{:.1f}".format
                  (tmyResidualFlyingKWh, tmyResidualChargeKWh))
            print("\n\tEV Totals:\t(%i EVs)\n\t\tCharge KWh:\t%.1f\n\t\tCharge Gap KWh:\t%.1f" %
                  (EV.evCount, EV.evChargeSteps * Drone.d0Type.WhEVChargeRatePerTimeStep/1000., EV.evChargeGap/(1000. * EV.evCount)))
            print("\t\tCharge Sessions:\n\t\t\tFull charges:\t{:.0f}\n\t\t\tPart (drone):\t{:.0f}\n\t\t\tPart (ev):\t{:.0f}".format
                  (tmyFullCharges, tmyBrokenCharges, tmyBrokenEVCharges))

            if GG.modelRendezvous:
                print("\n\tSuccessful chases: %i\tAverage chase time: %.1fs\tbroken Chases: %i" %
                      (tmyChaseCount, averageChase, tmyBrokenChaseCount))

            print("\nDiscrete Drone data:")
            for drone in sorted(self.freeDrones | self.needChargeDrones | set(self.allocatedDrone)):
                droneDistance = drone.myFlyingCount * drone.myDt.droneStepMperTimeStep / 1000.
                droneFlyingKWh = drone.myFlyingCount * drone.myDt.droneFlyingWhperTimeStep / 1000.
                droneChargeKWh = drone.myEVChargingCount * drone.myDt.WhEVChargeRatePerTimeStep / 1000.
                print("\tdrone: {}\tKm: {:.2f}\tCharge KW: {:.2f}\tFlyingKW: {:.2f}\tResidual (chargeWh: {:.0f} flyingWh: {:.0f})"
                      .format(drone.myID, droneDistance, droneChargeKWh, droneFlyingKWh, drone.myCharge, drone.myFlyingCharge))

    def requestCharge(self, ev, capacity, requestedWh=2000.):
        """request for charge from EV"""
        self.requests[ev] = requestedWh
        if GG.chargePrint:
            print("{}\t{}\t{!r}\t{}\t{:.1f}\t{:.1f}\t{:.1f}".format(GG.ss.timeStep, ev.getID(), EV.EVState.CHARGEREQUESTED, "", capacity, 0.0, requestedWh), file=GG.chargeLog)

    def setMaxDrones(self, pmaxDrones):
        """ update maxDrones - when --z option is used drones are limited to those in the add file"""
        self.maxDrones = pmaxDrones
        self.syncSpawnedDrones()

    def syncSpawnedDrones(self):
        """if we've generated drones from POI definitions in the add file we need to update our spawnedDrone count"""
        self.spawnedDrones = Drone.getIDCount(self)

    def tidyDrones(self):
        """remove any dummy vehicles left after all vehicles have left - ie simulation has finished"""
        if self.insertedDummies > 0:
            for drone in self.freeDrones | self.needChargeDrones:
                if drone.myDummyEVInserted:
                    drone.dummyEVHide()

    def update(self):
        """Management of 'control centre' executed by simulation on every step"""
        availableDrones = len(self.freeDrones) + self.maxDrones - self.spawnedDrones
        if availableDrones > 0 and len(self.requests) > 0:
            urgencyList = self.calcUrgency()
            self.allocateDrones(urgencyList)
            del urgencyList
        # Control centre manages parking/charging of drones
        # each EV 'manages' the drone allocated to them
        for drone in self.freeDrones | self.needChargeDrones:
            drone.parkingUpdate()
