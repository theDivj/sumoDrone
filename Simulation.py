"""Module implementing the SUMO simulation loop"""
import sys
import traci
from GlobalClasses import GlobalClasses as GG
from Drone import Drone
from EV import EV

class Simulation:
    """Class executing the simulation loop - tracks the timeStep"""
    # Basic simulation parameters
    maxEVs = sys.maxsize    # default to no limit on the number of EVs we will shadow - does not impact the actual no of EVs in the simulation
    stepSecs = 1.0          # Real length modelled by each simulation step - we can only know this when the simulation starts

    useChargeHubs = False   # whether we put drone charging output into charging station file - set true if the sumo options chargingstations-output is set

    timeStep = 0            # running count of simulation steps
    EVs = {}                # collection for the EVs we are managing

    def __init__(self, sumoCmd, maxEVs):
        traci.start(sumoCmd)  #  traceFile="./tracilog.txt")
        self.stepSecs = traci.simulation.getDeltaT()
        Drone.stepSecsAdjust(self.stepSecs)
        Simulation.maxEVs = maxEVs
        if traci.simulation.getOption("chargingstations-output"):
            Simulation.useChargeHubs = True

    def __del__(self):
        traci.close()
        Simulation.EVs.clear()

    @classmethod
    def step(cls):
        """Simulation step"""
        if traci.simulation.getMinExpectedNumber() > 0:
            traci.executeMove()                     #  move vehicles first so we can move drones to the same position
            Simulation.timeStep += 1

            loadedVehicles = traci.simulation.getLoadedIDList()     # add new EVs to our management list upto the maximum allowed
            for vehID in loadedVehicles:
                if traci.vehicle.getParameter(vehID, "has.battery.device") == "true":       # we are only interested in EVs
                    if len(Simulation.EVs) < Simulation.maxEVs:
                        Simulation.EVs[vehID] = EV(vehID,EV.kmPerWh)   # can set kmPerWh here to cater for different EVs - get from an EV parameter?

            if traci.simulation.getArrivedNumber() > 0:             # handle vehicles that have left the simulation
                arrivedVehicles = traci.simulation.getArrivedIDList()
                for aID in arrivedVehicles:
                   if aID in Simulation.EVs:
                        Simulation.EVs[aID].leftSimulation()        # notify EV shadow that the vehicle has left
                        Simulation.EVs[aID].update()                #  run the update as we will be removing this from the management loop
                        del Simulation.EVs[aID]

            for vehID,ev in Simulation.EVs.items():     # run the update (state machine) for each EV  we are managing
                ev.update()
            GG.cc.update()                      # trigger control centre management on this step
            traci.simulationStep()              # complete the SUMO step

            return True
        return False
