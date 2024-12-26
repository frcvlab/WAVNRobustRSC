#
# Collection of functions to run experiments
# using the WAVN RobustRSC class
# dml Dec 2024
#
import wavnRobustRSC as wv
import time as tm
import numpy as np
import math
import random
#

#-------------------------------------------------------------

#this alt main just gives examples of RCS Phase 1 and 2 paths
# for a fixed size world

def altMain1():
    wv.dr.Render=True
    gBFSvsRSC=False # RSC
    numRobots,numLandmarks=50,80
    rlRange =50
    wavn=wv.WAVNSim()
    wavn.gSwarmEnabled=False
    wavn.gRandomBlock=False
    wavn.blockProb=0 # robust or not
    wavn.gRobustPathEnabled=False
    wavn.rlRange=rlRange
    wavn.ppDebug=True
    robots, landmarks, walls = wavn.makeWorld(numRobots, numLandmarks, 0)
    #wavn.initTargets() # init swarm targets
    common,cansee = wavn.findCommon()
    wavn.makeGlobalSuccessors()
    ledger = wavn.makeLedger()
    wavn.drawWorld(False)
    print("Will show a bunch of examples.\n")
    print("RSC Phase 1 First, then press Enter Phase2, then again for BFS")
    for i in range(100): # show a bunch (100) of examples
        rs, le,ri=wavn.choosetargets()
        wavn.drawWorld(False,rs,le)
        rsppath = wavn.findPath(rs,ri,  le) # calculate the RSP path
        if rsppath is None:
            continue
        cpath=wv.Ledgerpath2path(rsppath,wavn.robots,wavn.landmarks)
        wv.dr.drawPath(cpath,wavn.landmarks,wavn.map,mark=wv.dr.pathMark1)
        wv.dr.showWorld(wavn.map)
        ch=input()
        
        path = wavn.refinePathVH(rsppath,options=wavn.gRobustPathEnabled)
        if path is None:
            continue
        cpath=wv.Ledgerpath2path(path,wavn.robots,wavn.landmarks)
        wv.dr.drawPath(cpath,wavn.landmarks,wavn.map,mark=wv.dr.pathMark2,offset=(2,2))
        wv.dr.showWorld(wavn.map)
        ch=input()
        
        bfspath,bfslength,findsucceeded = wavn.bfsFindPath(rs,le)
        cpath=wv.BFSpath2path(bfspath,wavn.robots,wavn.landmarks)
        wv.dr.drawPath(cpath,wavn.landmarks,wavn.map,mark=wv.dr.pathMark3,offset=(-2,-2))
        wv.dr.showWorld(wavn.map)
        ch=input()
    return

#
# Do same as altmain1 but for robust RSC
#

def altMain1R():
    wv.dr.Render=True
    gBFSvsRSC=False # RSC
    numRobots,numLandmarks=50,80
    rlRange =50
    wavn=wv.WAVNSim()
    wavn.gSwarmEnabled=False
    wavn.gRandomBlock=False
    wavn.blockProb=0 # robust or not
    wavn.gRobustPathEnabled=True
    wavn.rlRange=rlRange
    wavn.ppDebug=True
    robots, landmarks, walls = wavn.makeWorld(numRobots, numLandmarks, 0)
    #wavn.initTargets() # init swarm targets
    common,cansee = wavn.findCommon()
    wavn.makeGlobalSuccessors()
    ledger = wavn.makeRLedger()
    wavn.gBlocked=None # reverse any blockages
    wavn.drawWorld(False)
    print("Will show a bunch of examples.\n")
    print("RSC Phase 1 First, then press Enter Phase2, then again for BFS")

    for i in range(100):
        rs, le,ri=wavn.choosetargets()
        wavn.drawWorld(False,rs,le)
        rsppath = wavn.findPath(rs,ri,  le) # calculate the RSP path, auto robust
        if rsppath is None:
            continue
        cpath=wv.Ledgerpath2path(rsppath,wavn.robots,wavn.landmarks)
        wv.dr.drawPath(cpath,wavn.landmarks,wavn.map,mark=wv.dr.pathMark1)
        wv.dr.showWorld(wavn.map)
        wv.ch=input()
        path = wavn.refinePathVH(rsppath,options=wavn.gRobustPathEnabled,NS=200) # auto robust
        if path is None:
            continue
        cpath=wv.Ledgerpath2path(path,wavn.robots,wavn.landmarks)
        wv.dr.drawPath(cpath,wavn.landmarks,wavn.map,mark=wv.dr.pathMark2,offset=(2,2))
        wv.dr.showWorld(wavn.map)
        ch=input()
        bfspath,bfslength,findsucceeded = wavn.bfsFindPath(rs,le)
        cpath,avbr=wv.RBFSpath2path(bfspath,wavn.robots,wavn.landmarks,options=True)
        wv.dr.drawPath(cpath,wavn.landmarks,wavn.map,mark=wv.dr.pathMark3,offset=(-2,-2))
        wv.dr.showWorld(wavn.map)
        ch=input()
    return

#
# This alt main runs a series of experiments to compare BFS and RCS phase 1 and 2
# in terms of speed and path length

def altMain2(sigmaVal=10,sigma2Val=10,rlRangeVal=50,Ns=25,Nr=10):
    wv.dr.Render=False
    rsptime1,rsptime2,bfstime1=[],[],[] # initialize lists for metrics
    rsplen1,rsplen2,bfslen1=[],[],[]
    numScales=10
    numReps=100
    numK=5
    sigma=sigmaVal
    sigma2=sigma2Val
    rlRange =rlRangeVal
    for scale in range(1,numScales):
        print("Scale=",scale)
        rspt1,rspt2,bfst1=0.0,0.0,0.0 # initialize times
        rspl1,rspl2,bfsl1=0,0,0
        numPaths=0.0 # how many paths at this scale
        numRobots,numLandmarks=sigma*scale,sigma2*sigma*scale
        wavn=wv.WAVNSim()
        wavn.ppDebug=False
        wavn.expDebug=False
        wavn.gSwarmEnabled=False
        wavn.gRandomBlock=False
        wavn.blockProb=0 # robust or not
        wavn.gRobustPathEnabled=False
        wavn.rlRange=rlRange
        for k in range(0,numK):
            while True:
                robots, landmarks, walls = wavn.makeWorld(numRobots, numLandmarks, 0)
                #wavn.initTargets() # init swarm targets
                common,cansee = wavn.findCommon()
                wavn.makeGlobalSuccessors()
                ledger = wavn.makeLedger()
                if not ledger is None:
                    break
            i=0
            while i<numReps:
                # Do one experiment
                if wavn.expDebug and i % 50 ==0:
                    print("k=",k,"  i=",i)
                rs, le,ri=wavn.choosetargets()
                if rs is None: # no start robot => error in finding target
                    continue
                #wavn.drawWorld(False,rs,re)
                stime1=tm.perf_counter()
                rsppath1 = wavn.findPath(rs,ri, le) # RSC path(startR,endR, endLmk)
                etime1 = tm.perf_counter()-stime1
                if rsppath1 is None:
                    continue
                #'''
                stime2=tm.perf_counter()
                rsppath2 = wavn.refinePathVH(rsppath1,options=wavn.gRobustPathEnabled,NS=25,NR=10)
                etime2 = tm.perf_counter()-stime2
                if rsppath2 is None:
                    continue
                stime3=tm.perf_counter()
                bfspath,bfslength,findsucceeded = wavn.bfsFindPath(rs,le)
                #cpath2=wv.BFSpath2path(bfspath,wavn.robots,wavn.landmarks)
                etime3 = tm.perf_counter()-stime3
                if not findsucceeded:
                    continue
                # have all paths at this point
                numPaths+= 1
                i += 1
                rspt1 += etime1
                rspt2 += etime1+etime2 # total time
                bfst1 += etime3
                rspl1 += len(rsppath1) # wv.pathLength(rsppath1,landmarks)
                rspl2 +=  len(rsppath2) #wv.pathLength(rsppath2,landmarks) #plen[mili]#
                bfsl1 +=  len(bfspath) #wv.pathLength(bfspath,landmarks) #cpathLength(cpath2,wavn.robots,wavn.landmarks)#
        #end of this scale experiment k*100 runs
        print("End of scale=",scale," #P=",numPaths," collecting stats.")
        rsptime1.append( float(rspt1)/float(numPaths) )
        rsptime2.append( float(rspt2)/float(numPaths) )
        bfstime1.append( float(bfst1)/float(numPaths) )
        rsplen1.append( float(rspl1)/float(numPaths) )
        rsplen2.append( float(rspl2)/float(numPaths) )
        bfslen1.append( float(bfsl1)/float(numPaths) )
    # all experiments done, write results
    casename="RSC-BFS-Comparisons-12-24" # the name of the LOGFILE to use
    logfile = open("logfile"+casename+".csv","w") # APPEND!
    head="rlrange={},sigma2={}\n".format(rlRange,sigma2)
    logfile.write(head)
    print(head)
    head="Scale,nR,nL,RCST1,RCST2,BFST1,RCSL1,RSPL2,BFSL\n"
    logfile.write(head)
    print(head)
    print(len(rsptime1),len(rsptime2),len(bfstime1))
    for ss in range(1,numScales):
        s = ss-1
        line="{},{},{},{},{},{},{},{},{}\n".format(ss,sigma*ss,sigma*sigma2*ss,rsptime1[s],rsptime2[s],
                                                   bfstime1[s],rsplen1[s],rsplen2[s],bfslen1[s])
        logfile.write(line)
        print(line)
    return logfile,rsptime1,rsptime2,bfstime1,rsplen1,rsplen2,bfslen1


#
# This alt main runs a series of experiments to compare RBFS and RRCS phase 1 and 2
# in terms of speed and path length

def altMain2R(sigmaVal=10,sigma2Val=10,rlRangeVal=50,Ns=25,Nr=10):
    wv.dr.Render=False
    rsptime1,rsptime2,bfstime1=[],[],[] # initialize lists for metrics
    rsplen1,rsplen2,bfslen1=[],[],[]
    numScales=10
    numReps=100
    numK=5
    sigma=sigmaVal
    sigma2=sigma2Val
    rlRange =rlRangeVal
    for scale in range(1,numScales):
        print("Scale=",scale)
        rspt1,rspt2,bfst1=0.0,0.0,0.0 # initialize times
        rspl1,rspl2,bfsl1=0,0,0
        numPaths=0.0 # how many paths at this scale
        numRobots,numLandmarks=sigma*scale,sigma2*sigma*scale
        wavn=wv.WAVNSim()
        wavn.ppDebug=False
        wavn.expDebug=False
        wavn.gSwarmEnabled=False
        wavn.gRandomBlock=False
        wavn.blockProb=0 # robust or not
        wavn.gRobustPathEnabled=True
        wavn.rlRange=rlRange
        for k in range(0,numK):
            while True:
                robots, landmarks, walls = wavn.makeWorld(numRobots, numLandmarks, 0)
                #wavn.initTargets() # init swarm targets
                common,cansee = wavn.findCommon()
                wavn.makeGlobalSuccessors()
                ledger = wavn.makeRLedger()
                if not ledger is None:
                    break
            #for i in range(numReps):
            i=0
            while i<numReps:
                # Do one experiment
                if wavn.expDebug and i % 50 ==0:
                    print("k=",k,"  i=",i)
                rs, le,ri=wavn.choosetargets()
                if rs is None: # no start robot => error in finding target
                    continue
                #wavn.drawWorld(False,rs,re)
                stime1=tm.perf_counter()
                rsppath1 = wavn.findPath(rs,ri, le) # RSC path(startR,endR, endLmk), auto robust
                etime1 = tm.perf_counter()-stime1
                if rsppath1 is None:
                    continue
                stime2=tm.perf_counter()
                rsppath2 = wavn.refinePathVH(rsppath1,options=wavn.gRobustPathEnabled,NS=Ns,NR=Nr)
                etime2 = tm.perf_counter()-stime2
                if rsppath2 is None:
                    continue
                stime3=tm.perf_counter()
                bfspath,bfslength,findsucceeded = wavn.bfsFindPath(rs,le)
                etime3 = tm.perf_counter()-stime3
                if not findsucceeded:
                    continue
                # have all paths at this point
                numPaths+= 1
                i += 1
                rspt1 += etime1
                rspt2 += etime1+etime2 # total time
                bfst1 += etime3
                rspl1 += len(rsppath1) # wv.pathLength(rsppath1,landmarks)
                rspl2 += len(rsppath2) #wv.pathLength(rsppath2,landmarks)
                bfsl1 += len(bfspath) #wv.pathLength(bfspath,landmarks) 
        #end of this scale experiment k*100 runs
        print("End of scale=",scale," #P=",numPaths," collecting stats.")
        rsptime1.append( rspt1/numPaths )
        rsptime2.append( rspt2/numPaths )
        bfstime1.append( bfst1/numPaths )
        rsplen1.append( rspl1/numPaths )
        rsplen2.append( rspl2/numPaths )
        bfslen1.append( bfsl1/numPaths )
    # all experiments done, write results
    casename="RRSC-RBFS-Comparisons-12-18"
    logfile = open("logfile"+casename+".csv","w") # APPEND!
    head="rlrange={},sigma2={}\n".format(rlRange,sigma2)
    logfile.write(head)
    print(head)
    head="Scale,nR,nL,RRCST1,RRCST2,RBFST1,RRCSL1,RRSPL2,RBFSL\n"
    logfile.write(head)
    print(head)
    print(len(rsptime1),len(rsptime2),len(bfstime1))
    for ss in range(1,numScales):
        s = ss-1
        line="{},{},{},{},{},{},{},{},{}\n".format(ss,sigma*ss,sigma*sigma2*ss,rsptime1[s],rsptime2[s],
                                                   bfstime1[s],rsplen1[s],rsplen2[s],bfslen1[s])
        logfile.write(line)
        print(line)
    return logfile,rsptime1,rsptime2,bfstime1,rsplen1,rsplen2,bfslen1

#
# run a series of experiments with different ranges of rlRange and sigma2
#
def altMain3():
    rsptime1A,rsptime2A,bfstime1A,rsplen1A,rsplen2A,bfslen1A=[],[],[],[],[],[]
    s1=5 # sigma
    R1 = [25,100,500,1000] # sigma2
    R2 = [5,10,50] # rlRange
    for N1 in R1:
        for N2 in R2:
            print(f'N1={N1}, N2={N2}..')
            logfile,rsptime1,rsptime2,bfstime1,rsplen1,rsplen2,bfslen1=altMain2(Ns=N1,Nr=N2)
            rsptime1A.append( sum(rsptime1)/len(rsptime1) )
            rsptime2A.append( sum(rsptime2)/len(rsptime2) )
            bfstime1A.append( sum(bfstime1)/len(bfstime1) )
            rsplen1A.append( sum(rsplen1)/len(rsplen1) )
            rsplen2A.append( sum(rsplen1)/len(rsplen1) )
            bfslen1A.append( sum(bfslen1)/len(bfslen1) )
    casename="Multi-RSC-BFS-Comparisons-12-17"
    logfile = open("logfile"+casename+".csv","w") # APPEND!
    head="S2,RL,RCST1A,RCST2A,BFST1A,RCSL1A,RSPL2A,BFSLA\n"
    logfile.write(head)
    print(head)
    i = 0
    for s2 in R1: # sigma2 
        for rl in R2: # rlRange
            line="{},{},{},{},{},{},{},{},\n".format(s2,rl,rsptime1A[i],rsptime2A[i],bfstime1A[i],
                                                     rsplen1A[i],rsplen2A[i],bfslen1A[i])
            logfile.write(line)
            print(line)
            i += 1
    return
            


    
#run the RRCS/RBFS experiments

def altMain3R(sigmaVal=10,sigma2Val=10,rlRangeVal=50,Ns=25,Nr=10,appendFlag=False):
    #
    #GLOBAL settings for the simulation
    wv.dr.gRender = False # yes/no pictures
    gBFSvsRSC=True # true is BFS
    numRobots,numLandmarks=None,None
    sigma,sigma2=sigmaVal,sigma2Val
    #
    # make the list of experiments to perform
    expParamList = []
    kmax= 5 # 5 for repeat simulations
    rflag=True # robust only
    for tflag in [False,True]: # True is path type BFS and False is RSC
        for scale in range(1,10,1):
            numRobots=sigma*scale
            numLandmarks=sigma*sigma2*scale
            rlRange =50
            for pB in range(0,100,5): # probability of blockage
                for k in range(kmax): # k 'new' random worlds
                    exp=[numRobots,numLandmarks,k,rlRange,tflag,pB,rflag]
                    expParamList.append(exp)

    #
    casename="122624-AllP0-100-S1-9BIG"
    filemode='w'
    if appendFlag:
        filemode='a'
    logfile = open("logfile"+casename+".csv",filemode)
    head="nR,nL,K,rlR,BFSRSC,prob,Rob," + "%FP,%MP,avRed,%unblocked,avL,avS,avFPt\n"
    logfile.write(head)
    print(head)

    for params in expParamList:
        #MAIN experiment loop
        numRobots,numLandmarks,k,rlRange,gBFSvsRSC,blockProb,rflag=params
        s1="{},{},{},{},{},{},{},".format(numRobots,numLandmarks,
                                    k,rlRange,gBFSvsRSC,blockProb,rflag)
        print(s1)
        wavn=wv.WAVNSim()
        wavn.gSwarmEnabled=False
        wavn.gRandomBlock=(blockProb>0)
        wavn.blockProb=blockProb
        wavn.gRobustPathEnabled=rflag # robust or not
    
        while True:
            robots, landmarks, walls = wavn.makeWorld(numRobots, numLandmarks, 0)
            #wavn.initTargets() # init swarm targets
            common,cansee = wavn.findCommon()
            wavn.makeGlobalSuccessors()
            ledger = wavn.makeRLedger()
            if not ledger is None:
                break

        timeSteps=0
        findpathsucceeded=False
        movesucceeded=False
        path=None
        if k==0:
            # Initialize per experment - METRICS on performance
            numFPPaths=0 # successful planned paths
            numMPaths=0 # successful moves
            numFPFails=0 # number of find path fails
            numMFails=0 # number of move fails
            numRedundancy=0
            #
            numTO=0
            numPlen=0 # sum of path lengths
            numPsteps=0 # sum of steps in each path
            sumFPtime=0 # find path time
            #
            numBlocked=0 # paths that randomly were blocked
        #
        numGoals=0 # num unique goals
        maxTimeSteps=50
        maxGoals=5
        #
  
        while numGoals<maxGoals: #timeSteps<maxTimeSteps: #numFPPaths < 50: #  attempt to get to goal
            #
            timeSteps += 1
            #wavn.resetTargets() # anybody need a new goal
            #wavn.moveSwarm(-1)# move everybody else, not rs, increments time
            #
            rs,le,ri=wavn.choosetargets()
            if rs is None: # pick a another world 
                while True:
                    robots, landmarks, walls = wavn.makeWorld(numRobots, numLandmarks, 0)
                    wavn.initTargets() # init swarm targets
                    common,cansee = wavn.findCommon()
                    wavn.makeGlobalSuccessors()
                    ledger = wavn.makeRLedger()
                    if not ledger is None:
                        break
                continue
            #
            wavn.gBlocked=None # reverse any blockages from last exp
            #
            # Step 3: FIND A PATH
            numGoals+=1
            stime1=tm.perf_counter()  # start the time 
            #
            if not gBFSvsRSC:
                #
                #RSC findpath
                rsppath = wavn.findPath(rs,ri,le) # calculate the RSP path
                path = wavn.refinePathVH(rsppath,options=wavn.gRobustPathEnabled,NS=Ns,NR=Nr)
                findsucceeded = not rsppath is None
            else:
                #
                # BFS findpath
                bfspath,length,findsucceeded = wavn.bfsFindPath(rs,le) # automatic checks robust or not
                path,avbr=wv.RBFSpath2path(bfspath,wavn.robots,wavn.landmarks,options=True,coordFlag=False)
            #
            stime2=tm.perf_counter()-stime1 #end the time
            if findsucceeded:
                # increment various find path statistics
                numFPPaths += 1
                sumFPtime += stime2
                numRedundancy += wv.cpathRedundancy(path)
                bfssteps=len(path)
                movesucceeded,howfar=wavn.moveRobot(rs,path,le) # blocks handled within
                if movesucceeded :#or (not wavn.gRobustPathEnabled): #for speed
                    #
                    numMPaths += 1
                    numPsteps += (bfssteps-1) # single steps in the plan
                    numPlen += howfar # the distance moved for the plan
                else:
                    numMFails += 1
                wavn.gBlocked=None # reverse any blockages
            else:
                numFPFails += 1 # so just findpathpenalty
            #dr.showWorld(wavn.map)
        #end While
        numBlocked += wavn.numBlocked
        if k==kmax-1: # the final sim with the same parameters   
            #
            numGoals = maxGoals*kmax
            qFPPaths = round(numFPPaths/numGoals,2)  #% successful find paths
            qMPaths = round(numMPaths/numFPPaths,2)  #% succesfule xecuted paths
            qMUnBlocked = round(1.0-numBlocked/numFPPaths,2) #%paths randomly blocked
            #
            print("----")
            print(numFPPaths,numFPFails,numMPaths ,numMFails, numBlocked)
            print(numGoals, maxGoals,kmax)
            if numMPaths==0:
                numMPaths=1
            print("----")
            s2="{},{}, {},{}, ".format(qFPPaths,qMPaths,
                               float(numRedundancy)/float(numFPPaths),
                               qMUnBlocked)
            s3="{},{}, {}\n".format(round(numPlen/(numMPaths),2),
                           round(numPsteps/(numMPaths),2),
                           round(sumFPtime/numGoals,10))
            logfile.write(s1+s2+s3)
            print(head)
            print(s1,s2,s3)
            logfile.close()
            logfile = open("logfile"+casename+".csv","a")
    #endFOR
    
    logfile.close()
    print("Simulation DONE.")
    return

#
#--------------MAIN-----------
#

print("Select: \n 1. Show examples of RSC/BFS\n 2. Compare RSC/BFS\n")
print(          " 3. Run multi NrNs.\n 4. Show examples of RRCS/RBFS\n")
print(          " 5. Compare RRSC/RBFS.\n 6. Run RRSC/RBFS runtime comparison.")
ch = input("Enter your choice 1,2,3,4,5,6: ")

if ch=="1":
    altMain1() # show examples of RSC/BFS
elif ch=="2":
    altMain2() # run the RCS/BFS comparisons
elif ch=="3":
    altMain3() # run the multirange RCS/BFS comparisons
elif ch=="4":
    altMain1R() # show examples of RRSC/RBFS
elif ch=="5":
    altMain2R() # run the RCS/BFS comparisons
elif ch=="6":
    altMain3R() # Run RRSC/RBFS runtime comparison
else:
    print("Choices are 1,2,3,4,5,6 only.")
#-----END-MAIN-----------------