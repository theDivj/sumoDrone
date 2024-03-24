"""Module implementing the SUMO simulation loop"""
import sys
import traci
from GlobalClasses import GlobalClasses as GG
from EV import EV

class Simulation:
    """Class executing the simulation loop - tracks the timeStep"""
    # Basic simulation parameters
    maxEVs = sys.maxsize    # default to no limit on the number of EVs we will shadow - does not impact the actual no of EVs in the simulation
    stepSecs = 1.0          # Real length modelled by each simulation step - we can only know this when the simulation starts

    useChargeHubs = False   # whether we put drone charging output into charging station file - set true if the sumo options chargingstations-output is set

    timeStep = 0            # running count of simulation steps
    EVs = {}                # collection for the EVs we are managing
    poiDrones = 0
    
    usingSumoGui = False    # flag to let us breadcrumb

    def __init__(self, sumoCmd, maxEVs):   # cpp version passes maxdrones by ref
        try:
            traci.start(sumoCmd)  #   default port, retres traceFile="./tracilog.txt")
        except traci.TraCIException:
            print("Could not start: ",sumoCmd, " - ",traci.TraCIException)
            sys.exit(1)

        self.stepSecs = traci.simulation.getDeltaT()

        Simulation.maxEVs = maxEVs
        if traci.simulation.getOption("chargingstations-output"):
            Simulation.useChargeHubs = True
            
        for str in sumoCmd:                 # check whether we're using sumo-gui/sumo-gui.exe
            if str.find("sumo-gui") > 0:
                Simulation.usingSumoGui = True
            
    def __del__(self):
        traci.close()
        Simulation.EVs.clear()

    @classmethod
    def step(cls):
        """Simulation step"""
        if traci.simulation.getMinExpectedNumber() > 0:
            traci.executeMove()                     #  move vehicles first so we can move drones to the same position
            Simulation.timeStep += 1

            if not Simulation.usingSumoGui:
                op = int(Simulation.timeStep / 200) * 200
                if op == Simulation.timeStep:               #  let them know we're working
                    print(".", end="", flush=True, file=sys.stderr)
                    op = int(Simulation.timeStep / 16000) * 16000   # new line every 80 dots
                    if (op == Simulation.timeStep):
                        print("", file=sys.stderr)

            loadedVehicles = traci.simulation.getLoadedIDList()     # add new EVs to our management list upto the maximum allowed
            for vehID in loadedVehicles:
                if traci.vehicle.getParameter(vehID, "has.battery.device") == "true":       # we are only interested in EVs
                    if len(Simulation.EVs) < Simulation.maxEVs:
                        Simulation.EVs[vehID] = EV(vehID,EV.kmPerWh)   # can set kmPerWh here to cater for different EVs - get from an EV parameter?

            #tlist = traci.simulation.getStartingTeleportIDList();
            #if len(tlist) > 0:
            #    for  tport in tlist:
            #       if tport.endswith("-CB") or tport.endswith("-FB"):
            #           Simulation.tports.append(tport)


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

    def setMaxEvs(self, pmaxEVs):
        """set a limit to the number of EVs we handle - default is no limit"""
        Simulation.maxEVs = pmaxEVs
