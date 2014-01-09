f = open("newpoints.xyz","r")
g = open("lessnewpoints.xyz","w")

points = f.readlines()

import random

size = len(points)
nsize = size/10

subset = [points[i] for i in sorted(random.sample(xrange(size), nsize)) ]

for point in subset:
  g.write(point)
  
