"""Bounded 2D A* with circular footprint inflation; simulation only."""
import heapq
import math


class Grid:
    def __init__(self, bounds, obstacles, radius=.65, resolution=.2):
        self.bounds = bounds
        self.obstacles = obstacles
        self.radius = radius
        self.resolution = resolution

    def free(self, point):
        x,y = point[:2]
        x0,y0,x1,y1 = self.bounds
        r = self.radius + 1e-6  # Keep rounded grid paths off exact obstacle boundaries.
        if not (x0+r <= x <= x1-r and y0+r <= y <= y1-r):
            return False
        # Rectangle inflation is conservative at corners.
        return not any(a-r <= x <= c+r and b-r <= y <= d+r for a,b,c,d in self.obstacles)

    def path(self, start, goal):
        if not self.free(start) or not self.free(goal):
            return []
        s = self.resolution
        key = lambda p: (round(p[0]/s),round(p[1]/s))
        point = lambda k: (k[0]*s,k[1]*s)
        origin, target = key(start), key(goal)
        if not self.free(point(origin)) or not self.free(point(target)):
            return []
        queue = [(0,origin)]
        costs = {origin:0}
        parent = {}
        while queue:
            _,current = heapq.heappop(queue)
            if current == target:
                route = [current]
                while current in parent:
                    current = parent[current]
                    route.append(current)
                return [tuple(start[:2])] + [point(k) for k in reversed(route)] + [tuple(goal[:2])]
            for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
                nxt = current[0]+dx,current[1]+dy
                if not self.free(point(nxt)):
                    continue
                score = costs[current]+1
                if score < costs.get(nxt,math.inf):
                    parent[nxt] = current
                    costs[nxt] = score
                    h = abs(nxt[0]-target[0])+abs(nxt[1]-target[1])
                    heapq.heappush(queue,(score+h,nxt))
        return []
