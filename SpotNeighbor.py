import math
import numpy as np
import mpmath as mp
'''
Changing the approach:
-the weight is the distance to the closest robot in given direction
'''


def calc_y(i, r, k=15):
    '''
    Calculates the value of the y - in order to find the neighbors.
    k - is the number of equal partions that will always spot the neighbor that is on the border of the radius
    k for the range radius R = 55 is equal 13
    i - is the current line that is being calculated (i = {1, 2,..., k-1}
#    x - is the x on the axis; x = [0; R] for i <= [k/2] or x=[-R;0] otherwise
    r - is the radius of the given circle
    '''
    x = calc_x(i, r, k)
    if i >= (k / 4) and i <= (3 * k / 4):
        return int(round(math.sqrt(r**2 - x**2))) * -1
    return int(round(math.sqrt(r**2 - x**2)))


def calc_x(i, r, k=15):
    '''
    Calculates the value of the x direction - in order to find the neighbors.
    k - is the number of equal partions that will always spot the neighbor that is on the border of the radius
    k for the range radius R = 55 is equal 13
    i - is the current line that is being calculated (i = {1, 2,..., k-1}
    x - is the x on the axis; x = [0; R] for i <= [k/2] or x=[-R;0] otherwise

    from the circle equation x^2 + y^2 = 1 and we know that y = tan(i(degree)) * x
    thus |x| = x/sqrt(1 + tan(i(degree))); where 
    '''
    radians = mp.radians(i * (360) / k)
    cot = abs(mp.cot(radians))  #1 / math.tan(radians)
    if i > (k / 2):
        cot *= -1
    if cot < 0:
        return int(round(-r / math.sqrt(1 + abs(cot))))
    return int(round(r / math.sqrt(1 + cot)))


'''
Below there is the original way to calculate the weight. It assumes that the range goes after the robot.
'''


def add_weight(Robot, x, y, distance, W=150):

    #    For given x,y there is added the weight W (depending how far away this point P(x,y) is from the center of weight
    #    W(xi', yi') - the neighbor i coordinates

    weight = 0
    for n in Robot.neighbors:
        if (abs(n.x - x) <= (W / 2)) and (abs(n.y - y) <= (W / 2)):
            weight = weight + W - (abs(n.x - x) + abs(n.y - y))
    return weight


def check_line(Robot, i, k=15, R=75):
    line_weight = 0
    radius = 10
    for x in range(Robot.x + 10, Robot.x + R, 1):  #the robot radius is 10!
        y = calc_y(i, radius, k) + Robot.y
        line_weight = add_weight(Robot, x, y, radius)
        if line_weight > 0:
            return line_weight
        radius += 1

    return line_weight


def check_x0_line(Robot, R=75):
    line_weight = 0
    for y in range(10, R, 1):  #the robot radius is 10!
        line_weight += add_weight(Robot, Robot.x, Robot.y + y, y)
    return line_weight


def direction_line_equation(Robot):
    x_a, y_a = Robot.x, Robot.y
    x_b, y_b = (Robot.dir_x * 100) + Robot.x, (Robot.dir_y * 100) + Robot.y
    if x_a == x_b:
        if y_a > y_b:
            d = False
        else:
            d = True
        return 1, y_a, d
    elif y_a == y_b:
        if x_b < x_a:
            return 0, x_a, False
        return 0, x_a, True
    else:
        #        a = (y_b - y_a) / (x_b - x_a)
        #        a = -1 / a
        a = -Robot.dir_x / Robot.dir_y
        b = Robot.y - (Robot.x * a)
        if (Robot.x * a) + b < Robot.dir_y:
            d = True  #indicates if the direction is over or under the prependicular to the direction line
        else:
            d = False
        return a, b, d


def relative_distance(x0, y0, x1, y1):
    return math.sqrt(((x0 - x1)**2 + (y0 - y1)**2))


def is_follower(Robot):
    '''
    if there are any neighbors in our direction -> I am follower; Otherwise I am the leader.
    Returns direction and information if it is a follower
    '''
    a, b, d = direction_line_equation(Robot)

    x, y = 0, 0

    for n in Robot.neighbors:
        if n.AS != Robot.AS:
            continue  #we don't care now
        if d:
            if n.y > (n.x * a) + b:
                return True
        else:
            if n.y < (n.x * a) + b:
                return True
    return False


def point_to_direction_rd(Robot, neighbor):
    '''
    Calculating the distance from neighbor to direction line
    rd = (Ax0 + By0 + C) / sqrt(A^2 + B^2)
    '''
    A = Robot.dir_x
    B = Robot.dir_y
    C = Robot.y - (Robot.x * A)
    return abs((neighbor.x * A) +
               (neighbor.y * B) + C) / math.sqrt(A**2 + B**2)


def direction_to_neighbor(Robot, neighbor, rd):
    '''
    Change the Robot direction to approach the given neighbor
    '''
    Robot.dir_x = (neighbor.x - Robot.x)
    Robot.dir_y = (neighbor.y - Robot.x)
    while Robot.dir_x > 1 or Robot.dir_y > 1:
        Robot.dir_x /= 2
        Robot.dir_y /= 2


def follower(Robot):
    best_rd = 100000  #rd - relative distance
    best_neighbor = None
    for n in Robot.neighbors:
        if n.AS != Robot.AS:
            continue
        rd = point_to_direction_rd(Robot, n)
        if rd < best_rd:
            best_neighbor = n
            best_rd = rd
            '''            
    print("best rd", best_rd, Robot.x, Robot.y, "n:", best_neighbor.x,
          best_neighbor.y)
    print("Direction:", Robot.dir_x, Robot.dir_y)
    print("Prosta prostopadla:", direction_line_equation(Robot))
            '''
    direction_to_neighbor(Robot, best_neighbor, best_rd)


def keep_distances(Robot):
    for n in Robot.neighbors:
        rd = relative_distance(Robot.x, Robot.y, n.x, n.y)
        if rd < 25:
            if rd == 0:
                Robot.dir_x -= 0.75
                Robot.dir_y -= 0.75
                return
            negative_impact = (Robot.x - n.x, Robot.y - n.y)
            cx = (negative_impact[0] * 25 / (rd**2)) * 0.25
            cy = (negative_impact[1] * 25 / (rd**2)) * 0.25
            Robot.dir_x -= cx
            Robot.dir_y -= cy
