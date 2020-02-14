from queue import PriorityQueue
import math
class AStar(object):
    @staticmethod
    def run(graph, start, end, outfile):
        resultRoad = []
        #fo = open(outfile, "w")
        def e_dis(xxx_todo_changeme, xxx_todo_changeme1):
            (cx,cy) = xxx_todo_changeme
            (endx, endy) = xxx_todo_changeme1
            return math.sqrt((endx-cx)*(endx-cx) + (endy-cy)*(endy-cy))

        def recursive_print(currentS):
            if currentS[1] == 0 :
                return
            else:
                recursive_print(currentS[3])
                resultRoad.append(currentS[2])
                #fo.write(currentS[2]) 
                
        pq = PriorityQueue()
        startS = (e_dis(start[1],end[1]), 0.0, start, None)
        print(startS)
        pq.put(startS);
        popNumber = 0 
        X = [False]*(len(graph)+1)
        while not pq.empty() : 
            currentS = pq.get()
            popNumber = popNumber+1
            (index, (cx,cy)) = currentS[2]
            X[index] = True
            cost = currentS[1]
            if (cx,cy) == end[1]: 
                #fo.write(str(popNumber))
                #fo.write("\n")
                recursive_print(currentS)
                #fo.close()
                return resultRoad 
            else: 
                for neighbor in graph[index]:  
                    if X[neighbor[0]]:
                        pass
                    else:    
                        h = e_dis(neighbor[1], end[1])
                        newcost = cost+e_dis(neighbor[1], (cx,cy))
                        f = h+newcost
                        pq.put((f, newcost, neighbor,currentS))
  
        #this shouldn't happen    
        return -1    

