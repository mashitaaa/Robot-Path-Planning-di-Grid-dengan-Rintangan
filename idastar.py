import matplotlib.pyplot as plt
import numpy as np
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


def ida_star_visual(grid,start,goal):
   def heuristic(a,b):
       return abs(a[0]-b[0])+abs(a[1]-b[1])
  
   iterations=0
   start_time=time.time()
  
   def dfs(path,g,fbound):
       nonlocal iterations
       current=path[-1]
       f=g+heuristic(current,goal)
       iterations+=1
       plot_step(grid,path,current)
      
       if f>fbound:
           return f,None
       if current==goal:
           return f,path
       min_f=float('inf')
       for dx,dy in [(-1,0),(1,0),(0,-1),(0,1)]:
           nx,ny=current[0]+dx,current[1]+dy
           if 0<=nx<grid.shape[0] and 0<=ny<grid.shape[1] and (nx,ny) not in path and grid[nx,ny]==0:
               temp,result=dfs(path+[(nx,ny)],g+1,fbound)
               if result is not None:
                   return temp,result
               if temp<min_f:
                   min_f=temp
       return min_f,None
  
   fbound = heuristic(start,goal)
   while True:
       temp,result=dfs([start],0,fbound)
       if result is not None:
           end_time=time.time()
           return result,iterations,end_time-start_time
       if temp==float('inf'):
           return None,iterations,time.time()-start_time
       fbound=temp


print("Menjalankan IDA*...")
ida_path,ida_iter,ida_time=ida_star_visual(grid,start,goal)
print("IDA* selesai. Path:",ida_path)
print("Iterations:",ida_iter,"Time:",ida_time)
