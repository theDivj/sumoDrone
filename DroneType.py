"""DroneType class"""

class DroneType:
    """Drone parameters"""
    def __init__(self):
        self.droneKMperh = 60.0                          # drone cruising speed - will be overridden by global / runstring value
        self.droneChargeWh = 30000.0                       # capacity of battery used to charge ev's  - based on Ehang 184 load capacity
        self.droneFlyingWh = 14400.0                       # capacity of battery used to power drone
        self.droneFlyingWhperTimeStep = self.droneFlyingWh / (23 * 60.)   # power usage based on Ehang 184 which has battery capacity of 14.4 KW giving 23 mins flight time
        self.droneChargeContingencyp = .05           # minimum contingency level %
        self.droneChargeViablep = .3                      # minimum viable level %
        self.WhEVChargeRatePerTimeStep = 25000 / 3600.      # 25KW   rate of vehicle charge from drone(adjust for timeStep when simulation starts)
        self.WhDroneRechargePerTimeStep = 75000 / 3600.     # 75KW   rate of drone charge when parked(adjust for timeStep when simulation starts)

        self.droneImageFile = "drone.png"
        self.droneColour =  0, 0, 255, 255
        self.droneWidth = 10.0
        self.droneHeight = 10.0
        self.useOneBattery = False

        # derived variables
        self.droneMperSec =self.droneKMperh / 3.6
        self.droneStepMperTimeStep = self.droneMperSec                        # How far(metres) the drone will travel in one time step(adjust for timeStep when simulation starts)
        self.droneStepM2 = pow(self.droneStepMperTimeStep, 2)                 # precompute - used in distance calculations(adjust for timeStep when simulation starts)
        self.minDroneCharge = self.droneChargeContingencyp * self.droneChargeWh    # Thresholds to break off charging / flying
        self.minDroneFlyingWh = self.droneChargeContingencyp * self.droneFlyingWh
        self.viableDroneCharge = self.droneChargeViablep * self.droneChargeWh      # thresholds to allow allocation - ie enough charge to be useful
        self.viableDroneFlyingWh = self.droneChargeViablep * self.droneFlyingWh


    def setDerived(self, stepSecs = 1.0):
        """Adjust the derived variables for simulation step duration """
        #droneMperSec = (droneKMperh / 3.6)
        self.droneStepMperTimeStep = self.droneMperSec * stepSecs                        # How far(metres) the drone will travel in one time step(adjust for timeStep when simulation starts)
        self.droneStepM2 = pow(self.droneStepMperTimeStep, 2)                 # precompute - used in distance calculations(adjust for timeStep when simulation starts)

        self.droneFlyingWhperTimeStep *= stepSecs
        self.WhEVChargeRatePerTimeStep *= stepSecs
        self.WhDroneRechargePerTimeStep *= stepSecs

        self.minDroneCharge = self.droneChargeContingencyp * self.droneChargeWh    # Thresholds to break off charging / flying
        self.minDroneFlyingWh = self.droneChargeContingencyp * self.droneFlyingWh
        self.viableDroneCharge = self.droneChargeViablep * self.droneChargeWh      # thresholds to allow allocation - ie enough charge to be useful
        self.viableDroneFlyingWh = self.droneChargeViablep * self.droneFlyingWh
