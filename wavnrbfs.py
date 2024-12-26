#
# BFS implementation
# for WAVN
#
# c 2016 dml
#
# modified for minimum dist landmark selection
# 
# c 2022 dml
#
# # modified for robust returm
#
# c 2024 dml
#
import math
import numpy as np
#
#

#
# BFS treesearch given
#               fringe = [ searchnode ]
#               searchnode = [state,cost,[]] 
#               goalFunc(state)
#               succesorFunc(state)
#
# state is cm = ( bot, imageList )
#
# will return a path that contains all common landmarks
# between adjacent robot pairs, so a substitue can be picked
#

ctr = 0

def search(fringe, goalFunc, successorFunc):
    """carry out a tree search for goal from node in fringe"""
    global ctr
    #searched=[]
    rootbot = fringe[0][0][0]
    searched=[rootbot]
    while len(fringe)>0:
        rootnode = fringe[0] # bfs FIFO
        #print("rootnode is ",rootnode[0][0])
        fringe.remove(rootnode)
        ctr = ctr+1

        state = rootnode[0] # unpack state
        #print("State=",state)
        bot, image = state
        d = rootnode[1]
        planSoFar=rootnode[2]
        robotsSoFar = [ d[1] for d in planSoFar ]
        #print("wavnbfs: State is [",state[0].modelName,state[1]," image ] robotlist=",robotsSoFar)
        
        
        #print("\n wavnbfs: BFS Expands ",bot,image," and d is ",d,"\n")
        
        done,dest=goalFunc(bot)
        if done:
            #print("wavnbfs: Robot ",bot," sees homing target")
            lastPlan = rootnode[2]
            pathLength=rootnode[1]+1
            lastPlan.append(([dest],bot))
            #print("RBFS: returning path= ",lastPlan)
            return lastPlan,pathLength,True # final action and distance
        
        succList = successorFunc(bot,searched) # a list of states
        #print("wavnbfs: Len Sucessors ",len(succList))
        added2Fringe=[]
        robDict={}
        
        #print(succList)
        for entry in succList: # collect all for same robot
            bot,image=entry
            if bot in robDict:
                robDict[bot].append(image)
            else:
                robDict[bot]=[image]
        #for bot in robDict: # entry in succList: # (bot,panodet,image)
        for entry in succList: # (bot,panodet,image)
            #print("Entry =",entry)
            bot,image=entry
            #print("Entry =",entry)
            imageL=robDict[bot]
            #print("wavnrbfs: succ bot ",bot," list size ",len(imageL))
            planSoFar=list(rootnode[2])
            newAction = (imageL,bot)
            planSoFar.append(newAction)
            distSoFar=rootnode[1]+1
            #print("rootnode ",rootnode)
            newnode = (entry,distSoFar,planSoFar)
            if not bot in searched:
                fringe.append(newnode)# put them at the end
                searched.append(bot)
                added2Fringe.append(bot)
        #print("ADDED to fringe: ",added2Fringe)
    #print("No route found")
    return [],0,False


