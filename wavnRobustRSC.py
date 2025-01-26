#
# WAVN world simulation for large teams
# Implement RSC and RRSC 
# Evaluating Robust BFS and RSC against non robust forms
#
# dml Feb 2024
# + adding walls and doors to do
# more dynamic testing
# +This version also has moving robots/swarm
#
#

import cv2
import numpy as np
import random
import matplotlib.pyplot as plt
import multiprocessing as mp
import time as tm
import math
import pandas as pd

import wavnbfs as s
import wavnrbfs as robust_s

import json

import wavnGeom as geo # to detect wall occlisions
import wavnDraw as dr # to draw the world

random.seed() #(150) # for reproducability for testing




class WAVNSim:
    def __init__(self):

        # variables for common landmarks
        self.common=None
        self.cansee=None
        # variables for things in the world
        self.robots, self.landmarks, self.walls =None,None,None
        # target list for all moving robots (not the robot doing a path)
        self.targets={} # indexed by robot will have target x,y and unit velocity x,y
        # Is the current segment of the robot doing a path blocked
        self.gBlocked=None
        # some flags to change conditions
        self.gSwarmEnabled=False # if true, all robots are moving to targets
        self.gRobustPathEnabled=False # if true, all robots are moving to targets
        self.gRandomBlock=False # if true, the robot moving on a path may be blocked
        self.gContinuousMotion=False # if true, additionally simulation motion to the landmark
        #
        self.rlRange=50 # how far away a robot can see a landmark
        self.worldX=200 # world X dimension
        self.worldY=200 # worldY dimension
        self.worldWall=15 # how many walls in the world
        self.numRobots=3 # how many robots
        self.numLandmarks=3 # how many landmarks
        self.numWalls=0 # how many walls
        self.blockProb=30 # probability of a segment of a path being blocked
        self.numBlocked=0 # number blocked so far
        # general debug flag
        self.debug=False
        #
        # variables for BFS callbacks
        self.gSuccDict = {} #successor dictionary for BFS
        self.gGoalMark,self.gGoalBot=None,None
        #
        self.ledger=[] # the ledger [ [robotindex,clindex-with-prev-robot],.....]
        self.ledgerR=[]
        self.ledgerLM=[]
        #
        self.ppDebug=False # debug statements in path planning
        self.expDebug=False # tracing in the experimental run sequence
        self.map = np.zeros((self.worldX,self.worldY,3), np.uint8)
        dr.initDraw(self.worldX,self.worldY)
        #
        return
        


    #
    # FUNCTIONS to interface to BFS
    #       
    #
    # make the successor dictionary as a global
    def makeGlobalSuccessors(self):
        common,robots,landmarks=self.common,self.robots,self.landmarks
        self.gSuccDict={}
        for c in common:
            r1 = robots.index( c[0] ) # mapp position to robot index
            r2 = robots.index( c[1] )
            lmlist = c[2]
            for lm in lmlist: # get all landmarks
                lmi = landmarks.index(lm)
                #print(lm,lmi)
                if not r1 in self.gSuccDict:
                    self.gSuccDict[ r1 ] = [ (r2,-lmi ) ]
                elif not r2 in self.gSuccDict[ r1 ]:
                    self.gSuccDict[ r1 ].append( (r2,-lmi ))
        #for r in robots:
        #    ri = robots.index(r)
        #    if ri in self.gSuccDict:
        #        print(ri,len(self.gSuccDict[ri]))
        return

    # useful function to print out common list for bot
    def checkCommon(self,bot):
        robots=self.robots
        for c in self.common:
            if robots.index(c[0])==bot or robots.index(c[1])==bot:
                print("Check common(",bot,")=",c)
        return

    # goal function for BFS where goal is a landmark gGoalMark
    # call using lambda
    def goalFunc(self,bot):
        return self.landmarks[self.gGoalMark] in self.cansee[bot],self.gGoalMark


    # successor function for BFS, uses global gSuccDict
    # call using lambda
    def successorFunc(self,bot,searched):
        if not bot in self.gSuccDict:
            return []
        else:
            return self.gSuccDict[bot]
        return []

    # envelop function for BFS search
    # automatically checks robust or not
    def bfsFindPath(self,rs,re): # rs is start robot and re is goal landmark
        self.gGoalMark=re
        initstate = [rs,None]
        initnode=[ initstate,0,[["S",rs]] ]
        fringe = [ initnode ]
        if self.gRobustPathEnabled:
            path,length,succeeded = robust_s.search(fringe, self.goalFunc, self.successorFunc)
        else:
            path,length,succeeded = s.search(fringe, self.goalFunc, self.successorFunc)
        return path,length,succeeded
        
    # MAKE,READ,WRITE world functions
    #
    # Generate a random world with robots,landmarks and walls
    # arguments are the numbers of each
    # and the return is the listof the locations of each (x,y)
    # Walls are only added last and if they don't intersect anything
    #
    def makeWorld(self, nRobots,nLandmarks,nWalls=0):
        xrange=[0,self.worldX]
        yrange=[0,self.worldY]
        robots = []
        common = []
        landmarks = []
        walls=[]
        #
        # 
        for r in range(nRobots):
            x = random.randrange(xrange[0]+5,xrange[1]-5)
            y = random.randrange(yrange[0]+5,yrange[1]-5)
            robots.append( (x,y) )
        for r in range(nLandmarks):
            x = random.randrange(xrange[0]+5,xrange[1]-5)
            y = random.randrange(yrange[0]+5,yrange[1]-5)
            if not geo.anyclose((x,y),robots):
                landmarks.append( (x,y) )
        for r in range(nWalls):
            xy=random.randrange(0,2)
            if xy==1:
                x1=random.randrange(xrange[0],xrange[1]-1)
                x2=random.randrange(x1,min(x1+worldWall,xrange[1]-1))+1
                y1=random.randrange(yrange[0],yrange[1])
                y2=y1+1
            else:
                y1=random.randrange(yrange[0],yrange[1]-1)
                y2=random.randrange(y1,min(y1+worldWall,yrange[1]-1))+1
                x1=random.randrange(xrange[0],xrange[1])
                x2=x1+1
            m=(y2-y1)/(x2-x1)
            c = y2-m*x2
            w=(x1,y1,x2,y2,m,c)
            if (not geo.anyonline(w,robots)) and (not geo.anyonline(w,landmarks)):
                walls.append(w)

        self.robots=robots
        self.landmarks=landmarks
        self.walls=walls
        return robots, landmarks, walls

    # serialize the elements of the world and
    # write to a named file
    def writeWorld(self,filename):
        robots=self.robots
        landmarks=self.landmarks
        walls=self.walls
        d = { 'walls':walls, "landmarks":landmarks, "robots":robots }
        f=open(filename,"w")
        #pickle.dump(d,f)
        json.dump(d,f,indent=2)
        f.close()
        return

    # read the elements of the world from
    # a named file
    def readWorld(self,filename):
        f=open(filename,"r")
        d=json.load(f)
        walls=d["walls"]
        robots=d["robots"]
        landmarks=d["landmarks"]
        self.robots=robots
        self.landmarks=landmarks
        self.walls=walls
        print("Read world ",filename)
        print("#robots=",len(robots)," #landmarks=",len(landmarks)," #walls=",len(walls))
        f.close()
        return

    # determine which landmarks can be seen by each robot
    # - that is the dictionary can see
    # and which landmarks can be seen in common by robots
    # - that is the list common
    # format of common landmark list
    # [ [r1,r2,[l1,l2,..]], [r1,r3,[l1,l2,...]], ... ]
    def findCommon(self,extRange=-1):
        #
        common=[]
        cansee={} # one robot list
        robots,landmarks,walls=self.robots,self.landmarks,self.walls
        # each robot
        sum=0
        visibility = self.rlRange
        if extRange>0:
            visibility=extRange
        for ri in range(len(robots)):
            r=robots[ri]
            cl = []
            for l in landmarks:
                d = np.hypot(r[0]-l[0], r[1]-l[1])
                e=0
                if r[0]==l[0]:
                    e=0.001
                m1=(r[1]-l[1])/(r[0]-l[0]+e)
                c1=l[1]-m1*l[0]
                w1=(r[0],r[1],l[0],l[1],m1,c1)
                if d<=visibility:# and not geo.anyintersect(walls,w1):
                    cl.append(l)
            cansee[ri]=cl
            sum += len(cl)
        #
        #
        for r1 in range(len(robots)):
            for r2 in range(len(robots)):
                if (r1!=r2):
                    cl=[]
                    for l1 in cansee[r1]:
                        if l1 in cansee[r2]:
                            cl.append(l1)
                    if len(cl)>0:
                        entry=(robots[r1],robots[r2],cl)
                        common.append(entry)
        #
        #
        '''
        print("===========")
        for c in common:
            print("C:",robots.index(c[0]),robots.index(c[1]),end=":")
            for l in c[2]:
                print(landmarks.index(l),end=" ")
            print()
        print("===========")
        '''
        self.common,self.cansee=common,cansee
        return common,cansee

    #
    # who can see this landmark index re
    # returns robot index ri
    def lookfor(self,re):
        target=self.landmarks[re]
        done=[]
        numRob=len(self.robots)
        ri=random.randint(0,numRob-1)
        while len(done)<numRob:
            while ri in done:
                ri = random.randint(0,numRob-1)
            if target in self.cansee[ri]:
                return ri
            done.append(ri)
        return None
        

    #
    # MOVING ROBOT FUNCTIONS
    #
    #
    #
    # Function to allow an asynchonrous change in the map based on
    # robot proximity, just a stub
    def checkForInterrupt(self,boti):
        robots,walls=self.robots,self.walls
        rp = robots[boti]
        return 

    # Move one robot along its path, allow for loss of landmark visibility
    # move discretely on landmark segments or 'continuously' along the segments
    # letter allows for change during motion, a more complicated situation
    #
    def moveRobot(self,boti,path,re):
        #
        robots,landmarks,walls=self.robots,self.landmarks,self.walls
        blockprob=self.blockProb
        cx,cy = robots[boti]
        segi=0
        wasBlocked=False
        distanceMoved=0
        for seg in path:
            ri,landmark=seg
            if isinstance(ri,str) or ri>=len(robots):
                print(f"FatalError: moveRobot; bad robot index in path {ri}")
                exit()
            robotPos=robots[ri] # look up coordinates for robot that reported this common landmark
            robustPath=isinstance(landmark,list)
            if robustPath: # there are choices for this segment of the path
                segi=0 # remember we are trying option segi now
                p = landmark[segi] # index of this landmark 
            else:
                p=landmark # no options, so we just have this one landmark index
            if p is None:
                print(f"FatalError: moveRobot; bad landmark index in path {seg}")
                exit()
            p = landmarks[abs(p)] # coordinates of landmark
        
            if self.gSwarmEnabled:
                common,cansee = self.findCommon()#Only needed if the swarm is moving around too
                
            d = np.hypot(cx-p[0], cy-p[1]) # distance on the map from robot boti to landmark
            isVisible = d<self.rlRange # is landmark visible # or check if its in cansee

            #***Random loss of visibility of the landmark Pf
            flip=random.randint(0,100)
            blocked = (flip < self.blockProb) and self.gRandomBlock
            #***Random loss of visibility of the landmark Pf
        
            if blocked: # and not wasBlocked :# and just one blockage
                if not wasBlocked:
                    self.numBlocked+=1 # record we blocked one here
                wasBlocked=blocked
            if blocked:
                self.gBlocked=p # but p has newly become blocked

            # if this is a blocked landmark, then try to find another
            #
            while (self.gBlocked==p) :# while still blocked, find unblocked path
                #
                if robustPath and segi<len(landmark)-2 and self.gRobustPathEnabled: # there are more choices
                    segi += 1
                    p = landmark[segi]
                else:# there are no other options, so fail
                    self.gBlocked=None
                    return False,distanceMoved # MOTION FAILED #
                
            # got an UNBLOCKED landmark now
            # prepare velocity to ROBOT (not to landmark) 
            nx,ny=robotPos
            d = max(0.1,np.hypot(nx-cx,ny-cy)) # in case of a zero path, for div error
            segLength=d # remember how long this segment of the path is
            if self.gContinuousMotion: # simulation motion along the segment
                dx,dy=(nx-cx)/d,(ny-cy)/d
            visualHomingAccuracy = 1.0 # visual homing accuracy, fixed radius 1m
            while d>visualHomingAccuracy and self.gContinuousMotion:
                #
                d = np.hypot(nx-cx,ny-cy)
                k=1 # step size
                cx,cy=cx+k*dx,cy+k*dy
                cx,cy=max(5,min(self.worldX-5,cx)),max(5,min(self.worldY-5,cy))
                robots[boti]=(cx,cy)
                if self.gSwarmEnabled:
                    common,cansee = self.findCommon()
                    self.makeGlobalSuccessors()
                    ledger = self.makeRLedger()#self.makeLedger()
                    self.resetTargets() # anybody need a new goal
                    self.moveSwarm(rs=boti)# move everybody else, not rs
                    #dr.drawWorld(robots,landmarks,walls,common,cansee,self.map,clflag=False,rs=boti,re=re)
                    #dr.showWorld(self.map)
                    #checkForInterrupt(boti)
            #END while d
            distanceMoved+=segLength
        self.gBlocked=None
        return True,distanceMoved

    # check the list of targets and if any is None then
    # make a target and unit velocity [x,y,dx,dy]
    def resetTargets(self):
        targets,robots=self.targets,self.robots
        xrange=[0,self.worldX]
        yrange=[0,self.worldY]
        for r in range(len(robots)): #each robot
            if targets[r] is None:    
                x = random.randrange(xrange[0]+5,xrange[1]-5)
                y = random.randrange(yrange[0]+5,yrange[1]-5)
                rx,ry=robots[r]
                dx,dy=(x-rx),(y-ry)
                d = np.hypot(dx,dy)
                if d>0:
                    dx,dy=dx/d,dy/d
                targets[r]=[x,y,dx,dy]
        return

    # set all the targets top None
    def initTargets(self):
        for r in range(len(self.robots)): #each robot
            self.targets[r]=None
        return

    # move all robots along their tracks and reset if reached
    # target or blocked
    def moveSwarm(self,rs):
        if not self.gSwarmEnabled:
            return
        for r in range(len(self.robots)): #each robot
            if r==rs:
                continue # done move the findpath robot
            x,y,dx,dy=self.targets[r]
            rx,ry=self.robots[r]
            d=np.hypot(x-rx,y-ry)
            if d<1: # at target
                self.targets[r]=None # signal for a new targetnext time
                #
                continue
            nx,ny=rx+dx,ry+dy # new position
            if (nx<=5 or nx>=self.worldX-5) or (ny<=5 or ny>=self.worldY-5):
                self.targets[r]=None # resetting due to blockage
            nx,ny=max(5,min(self.worldX-5,(nx))),max(5,min(self.worldY-5,(ny)))
            if geo.pointonanyline(self.walls,(nx,ny)):
                self.targets[r]=None # resetting due to blockage
                print(f"moveSwarm; Robot {r} blocked.")
                nx,ny=max(5,min(self.worldX-5,(rx-dx))),max(5,min(self.worldY-5,(ry-dy)))
            self.robots[r]=((nx),(ny)) # new position of robot    
        return   
        
    #
    # RCS PATH FINDING functions
    #
    # make a ledger by random start and chain through common landmarks
    #format of ledger [ [robotindex,clindex-with-prev-robot],.....]
    #
    def makeLedger(self,ctr=0):
        rlist,lmark,comm=self.robots,self.landmarks,self.common
        ledger,ledgerR,ledgerLM=[],[],[]
        done=[]
        start=random.randrange(0,len(rlist)) # robot index in rlist
        startList=self.cansee[start]
        cli=self.landmarks.index( startList[0] )
        ledger.append( [start,None]) # -cli] )
        ledgerR.append(start)
        r = start
        Found=True
        #print(f'Start is {start}')
        while Found: # try to chain all the robots through common
            Found=False
            for c in comm:
                r1 = rlist.index( c[0] )
                r2 = rlist.index( c[1] )
                if r in [r1,r2]:
                    rn=r2 if r1==start else r1
                    if rn not in done:
                        #print(f'r {r} ; r1 {r1} r2 {r2} ')
                        cl = random.choice( c[2] ) # list of common landmarks
                        cli = lmark.index(cl)
                        #print(f' common {cli}')
                        ledger.append( [rn,-cli] )
                        ledgerR.append(rn)
                        ledgerLM.append(cli)
                        r = rn
                        done.append(r)
                        Found=True
                        break
        #
        '''
        print("----LEDGER-----")
        for r in ledger:
            print(r)
        print("---------------")
        '''
        if self.ppDebug:
            print("Ledger: ",len(ledger)," robots out of ",len(self.robots))
        self.ledger,self.ledgerR,self.ledgerLM=ledger,ledgerR,ledgerLM
        if float(len(ledger))/float(len(self.robots))<0.8:
            if ctr<5:
                return self.makeLedger(ctr+1)
            else:
                return None
        if self.ppDebug:
            print(self.ledgerR)
        return ledger

    #
    # make a robust ledger by random start and chain through common landmarks
    #format of ledger [ [robotindex,[all cl-index-with-prev-robot] ],.....]
    #
    def makeRLedger(self,ctr=0):
        rlist,lmark,comm=self.robots,self.landmarks,self.common
        ledger,ledgerR,ledgerLM=[],[],[]
        done=[]
        start=random.randrange(0,len(rlist)) # robot index in rlist
        startlm = [self.landmarks.index(i) for i in self.cansee[start]]
        ledger.append( [start,startlm] )
        ledgerR.append(start)
        r = start
        Found=True

        #print(f'Start is {start}')
        while Found: # try to chain all the robots through common
            Found=False
            for c in comm:
                r1 = rlist.index( c[0] )
                r2 = rlist.index( c[1] )
                if r in [r1,r2]:
                    rn=r2 if r1==start else r1
                    if rn not in done:
                        # All the common landmarks between the two robots
                        #print(f'r {r} ; r1 {r1} r2 {r2} ')
                        clL = [lmark.index(cl)for cl in c[2]]
                        #print(f' common {clL}')
                        ledger.append( [rn,clL] )
                        ledgerR.append(rn)
                        ledgerLM.append(clL)
                        r = rn
                        done.append(r)
                        Found=True
                        break
        '''
        print("----RLEDGER----")
        for r in ledger:
            print(r)
        print("---------------")
        '''
        if self.ppDebug:
            print("Ledger: ",len(ledger)," robots out of ",len(self.robots))
        self.ledger,self.ledgerR,self.ledgerLM=ledger,ledgerR,ledgerLM
        if float(len(ledger))/float(len(self.robots))<0.8:
            if ctr<5:
                return self.makeLedger(ctr+1)
            else:
                return None
        if self.ppDebug:
            print(self.ledgerR)
        return ledger

            
    # RSC Phase 1: find path from ledger
    # ledger entry l[0]=robot
    #
    
    def findPath(self,rs,ri, lm):
        ls,le=None,None
        ledger=self.ledger
        sFlag,eFlag=False,False
        for l in ledger:
            if l[0]==rs:
                ls = ledger.index(l)
                sFlag=True
            if l[0]==ri:
                le=ledger.index(l)
                eFlag=True
            if sFlag and eFlag:
                break
        if ls is None:
            print(f"Error: findPath; Start robot not in ledger: {rs}")
            return None
        if le is None:
            print("Error: findPath;  End robot that sees goal not in ledger",ri,lm,self.ledgerR)
            return None
        inc = 1 if ls<le else -1
        li=ls
        path=[]
        ll=len(ledger)-1
        while li!=le:
            lp = li if inc>1 else min(ll,li+1)
            llp=ledger[lp][1]
            path.append( [ledger[li][0],llp] )
            li = li+inc
        lp = li if inc>1 else min(ll,li+1)
        lli=ledger[li][0]
        if lli!=ri:
            print("Error: findPath; did not end on end robot: ",ledger[li][0],ri)
            return None
        path.append( [lli,lm])  # last element
        return path


    # RSC Phase 2: refine path    
    #
    def refinePathVH(self,path,options=False,NS=25,NR=5):
        #
        if path is None:
            return None
        #
        rlist,lmark=self.robots,self.landmarks
        Ns=NS
        Nr=NR
        cpath = path
        rewires=0
        for i in range(0,Ns): # attempt a refine
            #
            lcp=len(cpath)
            if lcp<=1: # can't do better than this
                break
            pi = random.randrange(0,lcp-1) 
            lmi = cpath[pi][1]
            if options and isinstance(lmi,list):
                pj = random.randrange(0,len(lmi)) # random option path
                lmi = lmi[pj] #cpath[pi][1][pj]
            else:
                lmi = abs(lmi)

            lm = lmark[lmi] # coordinates of landmark
            #
            # can anyone from pi-1 down to here-Nr see this?
            lower=max(0,(pi-1)-Nr)
            #
            for npi in range (lower,pi-1):
                ri = cpath[npi][0]
                #
                if lm in self.cansee[ri]: # yes some robot  can
                    newpath=cpath[0:npi+1]+cpath[pi+1:lcp]
                    #
                    if options and isinstance(cpath[pi][1],list):
                        a = self.cansee[ri]
                        b = [lmark[x] for x in cpath[pi][1]] 
                        newCL = [lmark.index(x) for x in a if x in b ]
                        newpath[npi+1]=[ri,newCL] #lmi
                        if len(newCL)==0:
                            print(f"Robot {ri}  can also see landmark {lmi} at {lm}")
                            print("Cansee? ",lm in self.cansee[ri],self.cansee[ri])
                            print("Other cansee? ",self.cansee[ cpath[pi][0] ])
                            print(f'FatalError: 0 newCL! a={a} b={b}')
                            exit(0)
                    else:
                        newpath[npi]=[ri,lmi]
                    rewires += 1
                    cpath=newpath
                    break #return cpath
        if self.ppDebug:
            print(f"Debug: RSC P2: (rewires={rewires})")
            for p in cpath:
                print(p)
        return cpath

    # useful function to choose a target that exists
    # only useful if you are not also counting path finding fails
    # return: start robot, end landmark, end robot
    #
    def choosetargets(self,timeout=50):
        while True: # find a robot in the ledger
            rs=random.randrange(0,len(self.robots))
            if rs in self.ledgerR: # is the robot in the current ledger, ignore if not
                break
        count=0
        while True and count< timeout:
            # find a target and a robot that can see which is not rs
            count +=1 # avoid infinite loop
            le=random.randrange(0,len(self.landmarks))
            lePos = self.landmarks[le]
            ri=self.lookfor(le)
            if not ri is None and  ri in self.ledgerR and lePos in self.cansee[ri] and not lePos in self.cansee[rs]:
                if self.ppDebug:
                    print(f"Debug: start {rs}, end robot {ri} sees landmark {le}={lePos}")
                return rs,le,ri
        print("Error: Choose targets; Unable to find a target!")
        return None,None,None

    # version of draw that uses the WAVNSim member variables
    #
    def drawWorld(self,clflag=True,rs=-1,re=-1):
        robots,landmarks,walls,=self.robots,self.landmarks,self.walls
        common,cansee=self.common,self.cansee
        return dr.drawWorld(robots,landmarks,walls,common,cansee,self.map,clflag,rs,re)
    #END OF SIMULATION CLASS
    #---------------------------------------------------------------------
#
# Some useful path manipulation functions
#
        
# how long in units is the path, where path=seq landmarks
def pathLength(path,lmarks):
    sum=0
    for pi in range(0,len(path)-1):
        lm1=abs(path[pi][1])
        lm2=abs(path[pi+1][1])
        l1=lmarks[lm1]
        l2=lmarks[lm2]
        sum += np.hypot(l1[0]-l2[0], l1[1]-l2[1])
    return sum

# how long in units is the path, where path = seq (x,y)
# for an optioned path, the length is just between first segements
def cpathLength(path,rlist,lmark):
    sum=0
    #print(path)
    for pi in range(0,len(path)-1):
        l1,l2=path[pi],path[pi+1] 
        if isinstance(l1,list):
            l1 = l1[0]
        if isinstance(l2,list):
            l2 = l2[0]
            l1,l2=rlist[l1],lmark[abs(l2)]
        sum += np.hypot(l1[0]-l2[0], l1[1]-l2[1])
    return sum

# how much reduandcy is in a path
# one measure is the sum of all path options over the lenegth
def cpathRedundancy(path):
    options=0.0
    for p in path:
        if isinstance(p[1],list):
            options += float(len(p[1]))
        else:
            options += 1.0
    #
    return float(options)/float(len(path))


      
# translate BFS path to a sequence of coordinates
# format of a no-chocie path =[p1,p2,...,pn], p=(lmark,robot) 
def BFSpath2path(bp,rlist,lmark):
    path=[]
    for p in bp:
        if p[0] in ['S','G']: # special landmarks, just use the robot position
            path.append( rlist[ p[1] ]  )
        else:
            path.append( lmark[ abs(p[0]) ] ) # landmark may/maynot be negative
            # the landmark is negative just to make it easier to see in a list
    return path

# translate BFS robust path to a non robust sequence of coordinates
# format of a robust path =[p1,p2,...,pn], p=(lmarklist,robot) 
def RBFSpath2path(bp,rlist,lmark,options=False,coordFlag=True):
    path=[]
    sum=0
    for p in bp:
        if p[0] in ['S','G']: # special landmarks, just use the robot position
            if coordFlag:
                path.append( (rlist[ p[1] ][0],rlist[ p[1] ][1])  )
            else:
                path.append( [p[1],p[1]]  )
        else:
            ll=p[0] # landmark list, just pick the first
            sum += len(p[0])
            if not options:
                if coordFlag:
                    path.append( [rlist[p[1]],lmark[ abs(ll[0]) ]] ) # landmark may/maynot be negative
                else:
                    path.append( [p[1],abs(ll[0])]  )
                # the landmark is negative just to make it easier to see in a list
            else:
                if coordFlag:
                    seg = [rlist[p[1]],[ lmark[ abs(i) ] for i in ll ]]
                else:
                    seg = [p[1],[ abs(i)  for i in ll ]]
                path.append(seg)
    avbr=sum/(len(bp)-1)
    return path,avbr



# translate Ledger path to a sequence of coordinates
def Ledgerpath2path(bp,rlist,lmark,coordFlag=True):
    if bp is None:
        return None
    path=[]
    path.append( rlist[ bp[0][0] ] )
    for p in bp:
        if isinstance(p[1],list): # robust path option
            if coordFlag:
                newlist=[lmark[ abs(a)] for a in p[1]]
            else:
                newlist=[abs(a) for a in p[1]]
            path.append(newlist)
        else:
            if coordFlag:
                path.append( rlist[p[0]]) #lmark[ abs(p[1]) ] )
            else:
                path.append( p[0] )
    if isinstance(bp[-1][1],list): # robust path option
        if coordFlag:
            path.append( [lmark[abs(a)] for a in bp[-1][1]])
        else:
            path.append( [abs(a) for a in bp[-1][1]] )
    else:
        if coordFlag:
            path.append( lmark[ abs(bp[-1][1]) ])
        else:
            path.append( abs(bp[-1][1]) )
    return path

#-------------------------------------------------------------




