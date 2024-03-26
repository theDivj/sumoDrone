# Using sumo POIs to represent 'Drones' for charging EVs
>  Traci scripts overlaying Drones on any [ SUMO ](https://github.com/eclipse-sumo/sumo) model that makes use of charging stations and EVs.

>   a faster, C++ version of this code, using libsumo, is available as [ SumoDrone-Cpp ](https://github.com/theDivj/SumoDrone-Cpp)                                   
---
 This software layers on top of a SUMO model using libsumo/libtraci to model the use of Drones in recharging vehicles. Drones are dispatched from charging stations and return there to recharge. POIs are used to represent drones in sumo-gui. Both Drones and Vehicles turn red when their charge threshold is reached and green whilst recharging. The assumption is that the Drone travels coupled with the vehicle when recharging and only ‘flies’ between charging stations and vehicles/between vehicles.
 
When vehicles reach a charge threshold they request a charge from a control centre which dispatches the nearest drone when available. 

When charging is complete/the drone is out of charge, then the drone goes to the nearest charging station to recharge, or to the nearest vehicle
in range with the most urgent charge need.

Charging stations(need to be created on the road network and are 'discovered' in the code. These are used to launch and 'charge' drones.
Drone and Vehicle behaviour depend on defaults set in additional files. 

The default parameters used for the Drone in this model correspond to those of an Ehang 184 which has top speed of 60km/h, and a 14.4 KW battery, giving 23 mins flight time.

The order of allocation of drones to vehicles is dependant on the 'urgency', the ratio between the distance to the nearest charge point and the remaining charge.

There are Python (using libtraci) and C++ (using libsumo) versions of this code. The Python version defaults to using sumo-gui.
The C++ version can only use sumo at the moment. (libsumo doesn’t yet support sumo-gui.)
  
---
  > Note that the charge "requested" by the ev defaults to 2kW. This can be modified by use of a vehicle or vehicleType parameter: chargeRequestWh
        eg \<param key="chargeRequestWh" value="10000"/\>   would set a charge request size of 10kW.  In all cases use of the -r option varies the request by +/- 30%
---
