# Using sumo POIs to represent 'Drones' for charging EVs
>  (Traci scripts for any SUMO model using charging stations and EVs.)      cf <https://github.com/eclipse-sumo/sumo>                                    
---
  Basically vehicles drive until they need a charge - currently triggered by thresholds in the EV model
  they then get charged by the nearest drone, that is not busy, after which the drone will return to the nearest charge hub - or another vehicle if required  

  Charging stations(need to be created on the road network and are 'discovered' in the code. These are used to launch and 'charge' drones
  The SUMO default for these is that the charging power is 0 - so they won't charge vehicles even if the vehicles stop at the station.

  This code uses the nearest free drone, or if none free, then one will be spawned (upto a limit) from the nearest charging station.
  When charging is complete/the drone is out of charge then the drone goes to the nearest chargeing station to recharge.

  The parameters used for the Drone in this model correspond to those of an Ehang 184 which has top speed of 60km/h, and a 14.4 KW battery giving 23 mins flight time.
  The order of allocation of drones to vehicles is dependant on the 'urgency' the ratio between the distance to the nearest charge point and the remaining charge

  In sumo-gui, vehicles and drones turn red when they need charging and green when they are actually charging.
  
    (These scripts were created as a test of the feasibility of using POIs to model movements outside the sumo road network.)
