import timeit
from Robot import Robot

r = Robot(200, 200, 1080, 720, [1, 1], 55)
n1 = Robot(225, 200, 1080, 720, [1, 1], 55)
n2 = Robot(220, 220, 1080, 720, [1, 1], 55)
n3 = Robot(220, 180, 1080, 720, [1, 1], 55)

r.neighbors.append(n1)
r.neighbors.append(n2)
r.neighbors.append(n3)
start = timeit.timeit()
r.find_direction()
end = timeit.timeit()
print("time of execution:", end - start)
