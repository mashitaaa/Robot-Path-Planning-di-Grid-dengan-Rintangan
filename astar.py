import matplotlib.pyplot as plt
import numpy as np
import heapq
import time
from IPython.display import clear_output

grid = np.array([
    [0, 0, 0, 1, 0],
    [0, 1, 0, 1, 0],
    [0, 1, 0, 0, 0],
    [0, 0, 0, 1, 0],
    [1, 1, 0, 0, 0]
])

start = (0,0)
goal = (4,4)

def plot_step(grid, path=[], current=None):
    plt.figure(figsize=(5,5))
    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            if grid[i,j]==1:
                plt.fill_between([j,j+1],[i,i],[i+1,i+1],color='black')
            else:
                plt.fill_between([j,j+1],[i,i],[i+1,i+1],color='white', edgecolor='black')
    for (i,j) in path:
        plt.fill_between([j,j+1],[i,i],[i+1,i+1],color='lightgreen')
    if current:
        plt.fill_between([current[1],current[1]+1],[current[0],current[0]],[current[0]+1,current[0]+1],color='blue')
    plt.text(start[1]+0.5,start[0]+0.5,'S',ha='center',va='center',fontsize=12,color='yellow')
    plt.text(goal[1]+0.5,goal[0]+0.5,'G',ha='center',va='center',fontsize=12,color='red')
    plt.xlim(0,grid.shape[1])
    plt.ylim(0,grid.shape[0])
    plt.gca().invert_yaxis()
    plt.grid(True)
    plt.show()
    time.sleep(0.2)
    clear_output(wait=True)

def astar_visual(grid,start,goal):
    def heuristic(a,b):
        return abs(a[0]-b[0])+abs(a[1]-b[1])
    
    open_list=[]
    heapq.heappush(open_list,(0+heuristic(start,goal),0,start,[start]))
    visited=set()
    iterations=0

    start_time=time.time()
    
    while open_list:
        f,g,current,path=heapq.heappop(open_list)
        iterations+=1
        plot_step(grid,path,current)
        
        if current==goal:
            end_time=time.time()
            return path,iterations,end_time-start_time
        
        if current in visited:
            continue
        visited.add(current)
        
        for dx,dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            nx,ny=current[0]+dx,current[1]+dy
            if 0<=nx<grid.shape[0] and 0<=ny<grid.shape[1] and grid[nx,ny]==0 and (nx,ny) not in visited:
                heapq.heappush(open_list,(g+1+heuristic((nx,ny),goal),g+1,(nx,ny),path+[(nx,ny)]))
    return None,iterations,time.time()-start_time

print("Menjalankan A*...")
astar_path,astar_iter,astar_time=astar_visual(grid,start,goal)
print("A* selesai. Path:",astar_path)
print("Iterations:",astar_iter,"Time:",astar_time)








