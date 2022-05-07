from Robot import Robot
import SpotNeighbor as spot

r = Robot(200, 200, 1080, 720, [1, 1], 55)
n1 = Robot(225, 200, 1080, 720, [1, 1], 55)
n2 = Robot(170, 170, 1080, 720, [1, 1], 55)
n3 = Robot(220, 180, 1080, 720, [1, 1], 55)

r.neighbors.append(n1)
r.neighbors.append(n2)
r.neighbors.append(n3)
r.AS = 1
n1.AS = 1
n2.AS = 1
n3.AS = 3
for i in range(18):
    print(spot.check_line(r, i, 19, 55))
print(r.find_direction())
#r.neighbors.append(n3)
#r.dir_x, r.dir_y = r.find_direction()
'''
n1.neighbors.append(r)
n1.neighbors.append(n2)

print(r.dir_x, r.dir_y)
n1.dir_x = r.dir_x
n1.dir_y = r.dir_y
a, b, d = spot.direction_line_equation(n1)
print("My line equation:", a, b, d)
print(spot.neighbor_check(n1, r, a, b, d))
print("My direction", spot.follower(n1))
'''
#dbg -> funkcja follower()
#objawy -> dir_x, dir_y = None if dirp
'''
    if d:
        if neighbor.y > (neighbor.x * a) + b:
            return True
    else:
        if neighbor.y < (neighbor.x * a) + b:  #this is breaking
            return True
    return False

wtedy dziala gora,
------------------------
    if d:
        if neighbor.y > (neighbor.x * a) + b:
            return True
    else:
        if neighbor.y < (neighbor.x * a) + b:  #this is breaking
            return True
    return False

a wtedy dziala dol xd
------------------------

    if d:
        if neighbor.y > (neighbor.x * a) + b:
            return True
    else:
        if neighbor.y < (neighbor.x * a) + b:  #this is breaking
            return True
        else:
            return True
    return False
a teraz mamy agregację -> zblizamy sie do siebie jak teletubisie
'''
