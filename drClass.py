#!/usr/bin/env python3
"""Module initiating SUMO Traci code implementing Drone based charging of EVs in motion"""
import os
import sys

from GlobalClasses import GlobalClasses as GG
from ChargeHubs import ChargeHubs
from ControlCentre import ControlCentre
from Simulation import Simulation
"""
    sample traci code - using a POI to represent a drone able to fly outside the network and track specific vehicles
                            network needs charging stations to launch and recharge drones
   run as:
        python drclass.py [-h] [-v] [-b] [-c filePath] [-d n] [-e n] [-k n] [-l] [-m] [-o filePath] [-p metres] [-r n]
                  [-s sumo.exe] [-t ehang184] [-we n.n] [-wu n.n]
                  sumocfg

   This program is made available under the terms of the Eclipse Public License 2.0 which is available at https://www.eclipse.org/legal/epl-2.0/
    updated rendezvous point algorithm from: https://www.codeproject.com/Articles/990452/Interception-of-Two-Moving-Objects-in-D-Space
"""
__version__ = '3.2 17th March 2024'
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

class drClass:
    """Wrapper class to setup and execute simulation"""
    briefStatistics = False     # whether to output single line summary statistics
    maxEvs = sys.maxsize        # maximum no of EVs that can be shadowed

    sumoCmd = None  # The sumo runstring

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
        if GG.cc is not None:
            GG.cc.printDroneStatistics(drClass.briefStatistics, __version__)

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
        parser.add_argument('-v', '--version', action='version', version='%(prog)s' + __version__)

        # set up the expected runstring
        parser.add_argument('sumocfg', help='sumo configuration file')            # mandatory - sumo configuration

        parser.add_argument('-b', '--brief', help='output a single line statistics summary, default full summary', action='store_const', default='False')
        parser.add_argument('-c', '--chargeFile', help='file for output of detailed EV charge levels beginning/end of charge, default no output', metavar='filePath', type=argparse.FileType('a'))
        parser.add_argument('-d', '--maxDrones', help='maximum drones to spawn, default is 6', metavar='n', type=int, default=6)
        parser.add_argument('-e', '--maxEVs', help='maximum EVs that are allowed to charge by Drone, default is no limit', metavar='n', type=int, default=sys.maxsize)
        parser.add_argument('-k', '--droneKmPerHr', help='drone speed Km/h', metavar='n', type=float, default=60.0)
        parser.add_argument('-l', '--lineOfSight', help='route drone to EV by line of sight at each step, default is to compute a rendezvous point\n', action='store_const', default='True')
        parser.add_argument('-m', '--multipleCharge', help='Allow EVs to be charged more than once - default is only once', action='store_const', default='True')
        parser.add_argument('-o', '--outputFile', help='file for output of detailed drone charge levels for each step, default no output', metavar='filePath', type=argparse.FileType('a'))
        parser.add_argument('-p', '--proximityRadius', help='proximity radius to scan for vehicles needing charge, default 1000', metavar='metres', type=float, default=1000)
        parser.add_argument('-r', '--randomSeed', help='seed for random generator triggering requests and sizeof requests', metavar='n', type=int, default=0)
        parser.add_argument('-s', '--sumoBinary', help='sumo binary to execute against configuration, default is sumo-gui.exe', metavar='sumo.exe', default="sumo-gui.exe")
        parser.add_argument('-t', '--droneType', help='type of drone - currently ehang184 or ehang184x', metavar='ehang184', default="ehang184")
        parser.add_argument('-we', '--wEnergy', help='weighting to apply to vehicles found in radius, default 1', metavar='n.n', type=float, default=1.0)
        parser.add_argument('-wu', '--wUrgency', help='weighting to apply to nearest vehicle urgency, default 0', metavar='n.n', type=float, default=0.0)

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

        # maximum no of EVs that can be charged by Drones
        maxEVs = args.maxEVs
        #
        randomSeed = args.randomSeed
        droneKmPerHr = args.droneKmPerHr

        # create sumo runstring
        sumoBinary = os.environ['SUMO_HOME'] + '/bin/' + args.sumoBinary
        drClass.sumoCmd = [sumoBinary, "-c", args.sumocfg]

        # create our management objects plus ChargeHubs - which is essentially static
        ss = Simulation(drClass.sumoCmd, maxEVs)
        ch = ChargeHubs()
        cc = ControlCentre(args.wEnergy, args.wUrgency, args.proximityRadius, args.maxDrones,args.droneType)

        # setup the global references to these objects
        gg = GG(cc, ss, ch)
        gg.setGlobals(droneKmPerHr, randomSeed, droneLog, chargeLog, onlyChargeOnce, modelRendezvous)

        # any output file would have been opened in parse_args() - write out the title line if needed
        if gg.dronePrint:
            print("Time Step\tDrone\tEV\tLane\tPosition\tdrone x\tdrone y\tdroneWh\tchargeWh\tflyingWh\tactivity", file=droneLog)
        if gg.chargePrint:
            print("timeStep\tEV id\tEV State\tDrone\tCapacity\tCharge Wh(if any)\tRequested Charge Wh", file=chargeLog)

        return gg


def main():
    """Instantiate!"""
    import argparse    # here because pdoc gets upset if its in the stamdard position at the top of the file
    # import tracemalloc
    # tracemalloc.start()

    parser = argparse.ArgumentParser(description="sample traci code - using a POI to represent a drone charging EVs")
    session = drClass()
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
