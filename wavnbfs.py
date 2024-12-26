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
# state is cm = ( bot, image )
#

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
            lastPlan.append((dest,bot))
            return lastPlan,pathLength,True # final action and distance
        
        succList = successorFunc(bot,searched) # a list of states
        #print("wavnbfs: Len Sucessors ",len(succList))
        added2Fringe=[]
        for entry in succList: # (bot,panodet,image)
            #print("Entry =",entry)
            bot,image=entry
            planSoFar=list(rootnode[2])
            newAction = (image,bot)
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


