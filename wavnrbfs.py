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
        fringe.remove(rootnode)
        ctr = ctr+1

        state = rootnode[0] # unpack state
        bot, image = state
        d = rootnode[1]
        planSoFar=rootnode[2]
        robotsSoFar = [ d[1] for d in planSoFar ]

        done,dest=goalFunc(bot)
        if done:
            #
            lastPlan = rootnode[2]
            pathLength=rootnode[1]+1
            lastPlan.append(([dest],bot))
            #
            return lastPlan,pathLength,True # final action and distance
        
        succList = successorFunc(bot,searched) # a list of states
        #
        added2Fringe=[]
        robDict={}
        
        for entry in succList: # collect all for same robot
            bot,image=entry
            if bot in robDict:
                robDict[bot].append(image)
            else:
                robDict[bot]=[image]
        
        for entry in succList: # (bot,panodet,image)
            #
            bot,image=entry
            imageL=robDict[bot]
            planSoFar=list(rootnode[2])
            newAction = (imageL,bot)
            planSoFar.append(newAction)
            distSoFar=rootnode[1]+1
            newnode = (entry,distSoFar,planSoFar)
            if not bot in searched:
                fringe.append(newnode)# put them at the end
                searched.append(bot)
                added2Fringe.append(bot)
    return [],0,False


