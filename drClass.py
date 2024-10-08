#!/usr/bin/env python3
"""Module initiating SUMO Traci code implementing Drone based charging of EVs in motion"""
import os
import sys

from GlobalClasses import GlobalClasses as GG
from ChargeHubs import ChargeHubs
from ControlCentre import ControlCentre
from Simulation import Simulation
from Drone import Drone

"""
    sample traci code - using a POI to represent a drone able to fly outside the network and track specific vehicles
                            network needs charging stations to launch and recharge drones
   run as:
        python drclass.py [-h] [-v] [-b] [-c filePath] [-d n] [-e n] [-f n] [-k n] [-l] [-m] [-o filePath] [-p metres] [-r n] [-s sumo.exe] 
            [-t ehang184] [-u] [-we n.n] [-wu n.n] [-z]
                  sumocfg

   This program is made available under the terms of the Eclipse Public License 2.0 which is available at https://www.eclipse.org/legal/epl-2.0/
    updated rendezvous point algorithm from: https://www.codeproject.com/Articles/990452/Interception-of-Two-Moving-Objects-in-D-Space
"""
__version__ = '3.5 at least'    # default - should be overwritten by getVersion()
#
# v3.0 is a complete rewrite as object code - replacing the quick and dirty original which was becoming spaghetti
#
# v3.1 Honours the charge request - v3 charged upto thresholds - which would be the charge request plus that used before the drone arrived and whilst charging
#        now the drone breaks off charging once it has delivered the requested charge.
#      Added option supporting randomisation of the charge request - by default request is 2kWh - passing a random seed varies this and the start threshold by +/- 30%
#      Added dummy EVs when Drone is at charging hub to represent the Drone batteries and record their charging period - only used when chargingstations-output is configured
#
# v3.2 Added override to default charge request  - parameter chargeRequestWh (for vehicle or vehicleType). Handled collisions when "hiding" dummy EV's
#
# v3.3 Added support for defining drones in additional file(s)  - ie add.xml
#
# v3.4 Corrected bug that ignored fractional weights for wurgency - updated calculation to equalise proximity vs urgency weight behaviour
#
# v3.5 Added runstring and add file option to use charge battery for flying. Added runstring option to try to avoid charges breaking when ev 'arrives'
#
class drClass:
    """Wrapper class to setup and execute simulation"""
    briefStatistics = False     # whether to output single line summary statistics
    maxEvs = sys.maxsize        # maximum no of EVs that can be shadowed

    sumoCmd = None      # The sumo runstring
    runstring = None    # the drClass.py runstring

    def __ini__(self):
        """Check whether we have access to sumo and parse runstring"""
        if 'SUMO_HOME' in os.environ:
            sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
        else:
            print(" SUMO_HOME not set in environment, cannot execute sumo/sumo-gui")
            sys.exit(1)

    def __del__(self):
        """Print statistics and close any files"""
        # output statistics
        version = self.getVersion()
        if GG.cc is not None:
            GG.cc.tidyDrones()
            GG.cc.printDroneStatistics(drClass.briefStatistics, version, self.runstring)

        # tidy up
        if GG.dronePrint:
            GG.droneLog.close()
        if GG.chargePrint:
            GG.chargeLog.close()

    def loop(self):
        """main simulation loop"""
        while GG.ss.step():
            pass

    def parseRunstring(self, parser=None, argparse=None):
        """use argparse to parse runstring and set our variables"""
        parser.add_argument('-v', '--version', action='version', version='%(prog)s ' + self.getVersion())

        # set up the expected runstring
        parser.add_argument('sumocfg', help='sumo configuration file')            # mandatory - sumo configuration

        parser.add_argument('-b', '--brief', help='output a single line statistics summary, default full summary', action='store_const', default='False')
        parser.add_argument('-c', '--chargeFile', help='file for output of detailed EV charge levels beginning/end of charge, default no output', metavar='filePath', type=argparse.FileType('a'))
        parser.add_argument('-d', '--maxDrones', help='maximum drones to spawn, default is 6', metavar='n', type=int, default=6)
        parser.add_argument('-e', '--maxEVs', help='maximum EVs that are allowed to charge by Drone, default is no limit', metavar='n', type=int, default=sys.maxsize)
        parser.add_argument('-f', '--fullChargeTolerance', help='tolerance (s) use > 0 ensure only full charges', metavar='n', type=int, default=0)
        parser.add_argument('-g', '--globalCharge', help='global override of all charge request values with this', metavar='wH', type=float, default=0.0)
        parser.add_argument('-k', '--droneKmPerHr', help='drone speed Km/h', metavar='n', type=float, default=60.0)
        parser.add_argument('-l', '--lineOfSight', help='route drone to EV by line of sight at each step, default is to compute a rendezvous point\n', action='store_const', default='True')
        parser.add_argument('-m', '--multipleCharge', help='Allow EVs to be charged more than once - default is only once', action='store_const', default='True')
        parser.add_argument('-o', '--outputFile', help='file for output of detailed drone charge levels for each step, default no output', metavar='filePath', type=argparse.FileType('a'))
        parser.add_argument('-p', '--proximityRadius', help='proximity radius to scan for vehicles needing charge, default 1000', metavar='metres', type=float, default=1000)
        parser.add_argument('-r', '--randomSeed', help='seed for random generator triggering requests and sizeof requests', metavar='n', type=int, default=0)
        parser.add_argument('-s', '--sumoBinary', help='sumo binary to execute against configuration, default is sumo-gui.exe', metavar='sumo.exe', default="sumo-gui.exe")
        parser.add_argument('-t', '--droneType', help='type of drone - currently ehang184 or ehang184x', metavar='ehang184', default="ehang184")
        parser.add_argument('-u', '--useOneBattery', help='use the charge battery for flying',action='store_const', default='False')
        parser.add_argument('-we', '--wEnergy','--we', help='weighting to apply to vehicles found in radius, default 1', metavar='n.n', type=float, default=1.0)
        parser.add_argument('-wu', '--wUrgency','--wu', help='weighting to apply to nearest vehicle urgency, default 0', metavar='n.n', type=float, default=0.0)
        parser.add_argument('-z', '--zeroDrone', '--z', help='Only use drones defined in the ...add.xml file', action='store_const', default='True')

        # and parse what we actually got
        args = parser.parse_args()
        if args.lineOfSight:                   # has the default value of True
            modelRendezvous = True
        else:                                  # unless the flag is in the runstring when it's 'None'
            modelRendezvous = False
        if args.multipleCharge:                # has the default value of True
            onlyChargeOnce = True
        else:                                  # unless the flag is in the runstring when it's 'None'
            onlyChargeOnce = False

        if args.outputFile:
            droneLog = args.outputFile
        else:
            droneLog = None
        if args.chargeFile:
            chargeLog = args.chargeFile
        else:
            chargeLog = None

        if args.brief:
            drClass.briefStatistics = False
        else:
            drClass.briefStatistics = True

        if args.zeroDrone:
            zeroDrone = False
        else:
            zeroDrone = True

        if args.useOneBattery:
            useOneBattery = False
        else:
            useOneBattery = True


        # maximum no of EVs that can be charged by Drones
        maxEVs = args.maxEVs
        randomSeed = args.randomSeed
        droneKmPerHr = args.droneKmPerHr

        # create sumo runstring
        sumoBinary = os.environ['SUMO_HOME'] + '/bin/' + args.sumoBinary
        drClass.sumoCmd = [sumoBinary, "-c", args.sumocfg]

        # create our management objects plus ChargeHubs - which is essentially static
        ss = Simulation(drClass.sumoCmd, maxEVs)
        ch = ChargeHubs()
        cc = ControlCentre(args.wEnergy, args.wUrgency, args.proximityRadius, args.maxDrones, args.fullChargeTolerance, args.globalCharge)

        # setup the global references to these objects
        gg = GG(cc, ss, ch)
        gg.setGlobals(droneKmPerHr, randomSeed, droneLog, chargeLog, onlyChargeOnce, modelRendezvous)

        Drone.setDroneType(useOneBattery, args.droneType)
        if args.droneType != "ehang184x":
            poiDrones = Drone.setDroneTypeFromPOI(useOneBattery, zeroDrone)
        if zeroDrone and (poiDrones > 0):
            cc.setMaxDrones(poiDrones)

        # any output file would have been opened in parse_args() - write out the title line if needed
        if gg.dronePrint:
            print("timeStep\tDrone\tEV\tLane\tPosition\tdrone x\tdrone y\tdroneWh\tchargeWh\tflyingWh\tactivity", file=droneLog)
        if gg.chargePrint:
            print("timeStep\tEV id\tEV State\tDrone\tCapacity\tCharge Wh(if any)\tRequested Charge Wh", file=chargeLog)

        return gg

    def getVersion(self):
        """fudge to extract version from a C++ header file"""
        try:
            vf = open("version.h","r")
            vfile = vf.readlines()
            vf.close()
            for vstr in vfile:
                vp = vstr.find("#define")
                if vp >= 0:
                    vp = vstr.find("__version__")
                    if vp > 1:
                        vstr = vstr[vp+12:]
                        vstr = vstr.replace("\"","")
                        vstr = vstr.strip()
                        return vstr
        except:
            pass
        return __version__

def main():
    """Instantiate!"""
    import argparse    # here because pdoc gets upset if its in the stamdard position at the top of the file
    # import tracemalloc
    # tracemalloc.start()

    runstring = ""                 # runstring to be used in print output
    for runArg in sys.argv:
        runstring += " " + runArg

    parser = argparse.ArgumentParser(description="sample traci code - using a POI to represent a drone charging EVs")
    session = drClass()
    session.runstring = runstring

    gg = session.parseRunstring(parser, argparse)
    del parser

    session.loop()

    # snapshot = tracemalloc.take_snapshot()
    # top_stats = snapshot.statistics('lineno')
    # for stat in top_stats[:10]:
    # print(stat)

    del session, gg


# wrapper to allow import without execution for pydoc/pdoc etc
if __name__ == '__main__':
    main()
