"""DroneType class"""
import math

class DroneType:
    """ """
    def __init__(self):
        self.droneKMperh = 60.0                          # drone cruising speed - will be overridden by global / runstring value
        self.droneChargeWh = 30000.0                       # capacity of battery used to charge ev's  - based on Ehang 184 load capacity
        self.droneFlyingWh = 14400.0                       # capacity of battery used to power drone
        self.droneFlyingWhperTimeStep = droneFlyingWh / (23 * 60.)   # power usage based on Ehang 184 which has battery capacity of 14.4 KW giving 23 mins flight time
        self.droneChargeContingencyp = .05           # minimum contingency level %
        self.droneChargeViablep = .3                      # minimum viable level %
        self.WhEVChargeRatePerTimeStep = 25000 / 3600.      # 25KW   rate of vehicle charge from drone(adjust for timeStep when simulation starts)
        self.WhDroneRechargePerTimeStep = 75000 / 3600.     # 75KW   rate of drone charge when parked(adjust for timeStep when simulation starts)

        self.droneImageFile = "drone.png"
        self.droneColour =  0, 0, 255, 255 
        self.droneWidth = 10.0
        self.droneHeight = 10.0

        # derived variables
        self.droneMperSec = droneKMperh / 3.6
        self.droneStepMperTimeStep = droneMperSec                        # How far(metres) the drone will travel in one time step(adjust for timeStep when simulation starts)
        self.droneStepM2 = pow(droneStepMperTimeStep, 2)                 # precompute - used in distance calculations(adjust for timeStep when simulation starts)
        self.minDroneCharge = droneChargeContingencyp * droneChargeWh    # Thresholds to break off charging / flying
        self.minDroneFlyingWh = droneChargeContingencyp * droneFlyingWh
        self.viableDroneCharge = droneChargeViablep * droneChargeWh      # thresholds to allow allocation - ie enough charge to be useful
        self.viableDroneFlyingWh = droneChargeViablep * droneFlyingWh


    def setDerived(self, stepSecs = 1.0) 
        #droneMperSec = (droneKMperh / 3.6)
        self.droneStepMperTimeStep = droneMperSec * stepSecs                        # How far(metres) the drone will travel in one time step(adjust for timeStep when simulation starts)
        self.droneStepM2 = pow(droneStepMperTimeStep, 2)                 # precompute - used in distance calculations(adjust for timeStep when simulation starts)

        self.droneFlyingWhperTimeStep *= stepSecs
        self.WhEVChargeRatePerTimeStep *= stepSecs
        self.WhDroneRechargePerTimeStep *= stepSecs

        self.minDroneCharge = droneChargeContingencyp * droneChargeWh    # Thresholds to break off charging / flying
        self.minDroneFlyingWh = droneChargeContingencyp * droneFlyingWh
        self.viableDroneCharge = droneChargeViablep * droneChargeWh      # thresholds to allow allocation - ie enough charge to be useful
        self.viableDroneFlyingWh = droneChargeViablep * droneFlyingWh
