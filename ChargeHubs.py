"""Methods mapping charge hubs - initialised on startup then read only"""
import sys
import traci


class ChargeHubs:
    """Static Class capturing the charge hub locations and functions to find the nearest hub"""
    chargeHubLocations = {}

    def __init__(self):
        self.locateChargeHubs()

    @staticmethod
    def findNearestHub(px, py):
        """find the nearest hub, as the crow flies, to a point (ev or drone location)"""
        minHub = ""
        minDistance = sys.float_info.max

        for hub, (hx, hy, e, p) in ChargeHubs.chargeHubLocations.items():
          distance = ((hx - px) * (hx - px)) + ((hy - py) * (hy - py))   # don't need to sqrt
          if distance < minDistance:
             minDistance = distance
             minHub = hub
        return minHub, minDistance

    @staticmethod
    def findNearestHubDriving(evID):
        """find the nearest hub reachable by the ev, (driving distance)"""
        minHub = ""
        minDistance = sys.float_info.max

        for hub, (hx, hy, edge, pos) in ChargeHubs.chargeHubLocations.items():
          dd = traci.vehicle.getDrivingDistance(evID, edge, pos)
          if dd >= 0.0:     # if vehicle cannot get to charge hub on its route this returns INVALID_DOUBLE_VALUE - large negative no
            if dd < minDistance:
                minDistance = dd
                minHub = hub
        return minHub, minDistance

    def locateChargeHubs(self):   # not static - this updates the class
        """save the positions of all the hubs"""
        chargeHubs = traci.chargingstation.getIDList()
        for hub in chargeHubs:
            lane = traci.chargingstation.getLaneID(hub)
            edge = lane[:lane.find("_")]
            pos = traci.chargingstation.getStartPos(hub) + 5      # move inside the CS
            x, y = traci.simulation.convert2D(edge, pos)
            ChargeHubs.chargeHubLocations[hub] = x, y, edge, pos
            traci.route.add(edge, [edge])                         # create a route comprising the edge where the hub resides - to allow add of dummyEVs for drone batteries

    @staticmethod
    def nearestHubLocation(pos):
        """Helper function"""
        hub, distance = ChargeHubs.findNearestHub(pos[0], pos[1])
        return ChargeHubs.chargeHubLocations[hub], distance
