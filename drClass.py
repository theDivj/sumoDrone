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
        python drclass.py [-h] [-v] [-m] [-l] [-d n] [-b] [-e n] [-s sumo.exe] [-p metres] [-wu n.n] [-we n.n] [-o filePath] [-c filePath] xxx.sumocfg
                    where:    xxx.sumocfg      is the sumo configuration file


   This program is made available under the terms of the Eclipse Public License 2.0 which is available at https://www.eclipse.org/legal/epl-2.0/
    updated rendezvous point algorithm from: https://www.codeproject.com/Articles/990452/Interception-of-Two-Moving-Objects-in-D-Space
"""
__version__ = ' 3.0 25th February 2024'
#
#  v3.0 is a complete rewrite as object code - replacing the quick and dirty original which was becoming spaghetti
#
class drClass:
    """Wrapper class to setup and execute simulation"""
    modelRendezvous = True      # whether we comnpute a rendezvous point
    onlyChargeOnce = True       # whether each EV will only be charged once
    dronePrint = False          # whether to output a file detailing drone charge/charging
    droneLog = None             #  filepath for dronePrint
    chargePrint = False         # whether to output a file detailing charge request/charging for each EV
    chargeLog = None            #   filepath for chargeLog
    briefStatistics = False     # whether to output single line summary statistics
    maxEvs  = sys.maxsize       # maximum no of EVs that can be shadowed

    # Environment classes
    ss = None       # Simulation class
    ch = None       # ChargeHubs class
    cc = None       # Control Centre class
    sumoCmd = None  #  The sumo runstring

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
        if drClass.cc is not None:
            drClass.cc.printDroneStatistics(drClass.briefStatistics,drClass.sumoCmd,__file__,__version__)

        # tidy up
        if drClass.dronePrint:
            drClass.droneLog.close()
        if self.chargePrint:
            drClass.chargeLog.close()
        del drClass.cc, drClass.ch, drClass.ss

    def loop(self):
        """main simulation loop"""
        while drClass.ss.step():
            pass

    def parseRunstring(self,parser=None,argparse=None):
        """use argparse to parse runstring and set our variables"""
        #parser = argparse.ArgumentParser(description="sample traci code - using a POI to represent a drone charging EVs")
        parser.add_argument('-v', '--version', action='version', version='%(prog)s' + __version__)

        # set up the expected runstring
        parser.add_argument('sumocfg', help='sumo configuration file')            # mandatory - sumo configuration

        parser.add_argument('-m','--multipleCharge', help='Allow EVs to be charged more than once - default is only once',action='store_const',default='True')
        parser.add_argument('-l','--lineOfSight', help='route drone to EV by line of sight at each step, default is to compute a rendezvous point\n',action='store_const',default='True')
        parser.add_argument('-d','--maxDrones', help='maximum drones to spawn, default is 6',metavar='n',type=int, default=6)
        parser.add_argument('-b','--brief', help='output a single line statistics summary, default full summary',action='store_const',default='False')
        parser.add_argument('-e','--maxEVs', help='maximum EVs that are allowed to charge by Drone, default is no limit', metavar='n', type=int, default=sys.maxsize)
        parser.add_argument('-s','--sumoBinary', help='sumo binary to execute against configuration, default is sumo-gui.exe', metavar='sumo.exe', default="sumo-gui.exe")
        parser.add_argument('-p','--proximityRadius', help='proximity radius to scan for vehicles needing charge, default 1000',metavar='metres', type=float, default=1000)
        parser.add_argument('-wu','--wUrgency', help='weighting to apply to nearest vehicle urgency, default 0', metavar='n.n', type=float, default=0.0)
        parser.add_argument('-we','--wEnergy', help='weighting to apply to vehicles found in radius, default 1',metavar='n.n', type=float, default=1.0)
        parser.add_argument('-o','--outputFile', help='file for output of detailed drone charge levels for each step, default no output',metavar='filePath', type=argparse.FileType('x'))
        parser.add_argument('-c','--chargeFile', help='file for output of detailed EV charge levels beginning/end of charge, default no output',metavar='filePath', type=argparse.FileType('x'))

        # and parse what we actually got
        args = parser.parse_args()
        if args.lineOfSight:                   # has the default value of True
            drClass.modelRendezvous = True
        else:                                  # unless the flag is in the runstring when it's 'None'
            drClass.modelRendezvous = False
        if args.multipleCharge:                # has the default value of True
            drClass.onlyChargeOnce = True
        else:                                  # unless the flag is in the runstring when it's 'None'
            drClass.onlyChargeOnce = False
        if args.outputFile:
            drClass.dronePrint = True
            drClass.droneLog = args.outputFile
        else:
            drClass.dronePrint = False
            drClass.droneLog = ""
        if args.chargeFile:
            drClass.chargePrint = True
            drClass.chargeLog = args.chargeFile
        else:
            drClass.chargePrint = False
            drClass.chargeLog = ""

        if args.brief:
            drClass.briefStatistics = False
        else:
            drClass.briefStatistics = True

        # maximum no of EVs that can be charged by Drones
        drClass.maxEVs = args.maxEVs

        # create sumo runstring
        sumoBinary = os.environ['SUMO_HOME'] + '/bin/'+ args.sumoBinary
        drClass.sumoCmd = [sumoBinary, "-c", args.sumocfg]

        # create our management objects plus ChargeHubs - which is essentially static
        drClass.ss = Simulation(drClass.sumoCmd,drClass.modelRendezvous,drClass.onlyChargeOnce,drClass.maxEVs,drClass.dronePrint,drClass.droneLog)
        drClass.ch = ChargeHubs()
        drClass.cc = ControlCentre(args.wEnergy, args.wUrgency, args.proximityRadius,args.maxDrones,\
                                                drClass.dronePrint,drClass.droneLog,drClass.chargePrint,drClass.chargeLog)

        # setup the global references to these objects
        g = GG(drClass.cc,drClass.ss,drClass.ch)

        # any output file would have been opened in parse_args() - write out the title line if needed
        if self.dronePrint:
            print("Time Step\tDrone\tEV\tLane\tPosition\tdrone x\tdrone y\tdroneWh\tchargeWh\tflyingWh\tactivity", file=self.droneLog)
        if self.chargePrint:
            print("timeStep\tEV id\tEV State\tDrone\tCapacity\tCharge (if any)",file=self.chargeLog)

def main():
    """Instantiate!"""
    import argparse    # here because pdoc gets upset if its in the stamdard position at the top of the file
    # import tracemalloc
    #If we are using tracemalloc to check for memory leaks:
    #tracemalloc.start()

    parser = argparse.ArgumentParser(description="sample traci code - using a POI to represent a drone charging EVs")
    session = drClass()
    session.parseRunstring(parser, argparse)
    session.loop()

    #snapshot = tracemalloc.take_snapshot()
    #top_stats = snapshot.statistics('lineno')
    #for stat in top_stats[:10]:
    #   print(stat)

    del session

# wrapper to allow import without execution for pydoc/pdoc etc
if __name__ == '__main__':
    main()
