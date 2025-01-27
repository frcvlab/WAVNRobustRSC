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
        fringe.remove(rootnode)
        ctr = ctr+1

        state = rootnode[0] # unpack state
        bot, image = state
        d = rootnode[1]
        planSoFar=rootnode[2]
        robotsSoFar = [ d[1] for d in planSoFar ]
        done,dest=goalFunc(bot)
        if done:
            lastPlan = rootnode[2]
            pathLength=rootnode[1]+1
            lastPlan.append((-dest,bot))
            return lastPlan,pathLength,True # final action and distance
        
        succList = successorFunc(bot,searched) # a list of states)
        added2Fringe=[]
        for entry in succList: # (bot,panodet,image)
            bot,image=entry
            planSoFar=list(rootnode[2])
            newAction = (image,bot)
            planSoFar.append(newAction)
            distSoFar=rootnode[1]+1
            newnode = (entry,distSoFar,planSoFar)
            if not bot in searched:
                fringe.append(newnode)# put them at the end
                searched.append(bot)
                added2Fringe.append(bot)
    return [],0,False


