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


def add_weight(Robot, x, y, distance, W=150):
    '''
    Calculates the weight of the path. If neighbor position is close to the line which 
    is being checked, then some weight will be added. The closest is the robot to the 
    streight line, the higher weight will be added.
    '''

    #    For given x,y there is added the weight W (depending how far away this point P(x,y) is from the center of weight
    #    W(xi', yi') - the neighbor i coordinates

    weight = 0
    for n in Robot.neighbors:
        if (abs(n.x - x) <= (W / 2)) and (abs(n.y - y) <= (W / 2)):
            weight = weight + W - (abs(n.x - x) + abs(n.y - y))
    return weight


def check_line(Robot, i, k=15, R=75):
    '''
    Checks the given line (k - the given fraction of the circle).
    '''
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
    '''
    Special case of check line - checks the line for the x = 0.
    '''
    line_weight = 0
    for y in range(10, R, 1):  #the robot radius is 10!
        line_weight += add_weight(Robot, Robot.x, Robot.y + y, y)
    return line_weight


def direction_line_equation(Robot):
    '''
    Returns a - the coefficient of the line prependicular to the direction (y = ax + b),
    b - similarly
    d - the half of the space in which we are searching for other robots
    '''
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
        a = -Robot.dir_x / Robot.dir_y
        b = Robot.y - (Robot.x * a)
        if (Robot.x * a) + b < Robot.dir_y:
            d = True  #indicates if the direction is over or under the prependicular to the direction line
        else:
            d = False
        return a, b, d


def relative_distance(x0, y0, x1, y1):
    '''
    Calculates the relative distance between two points
    '''
    return math.sqrt(((x0 - x1)**2 + (y0 - y1)**2))


def neighbor_check(Robot, neighbor, a, b, d):
    '''
    Checks if given robot can possibly be followed by this robot.
    returns true if the robot can be followed. Otherwise returns false.
    '''
    if d:
        if neighbor.y > (neighbor.x * a) + b:
            return True
    else:
        if neighbor.y < (neighbor.x * a) + b:
            return True
    return False


def is_follower(Robot):
    '''
    if there are any neighbors in our direction -> I am follower; Otherwise I am the leader.
    Returns direction and information if it is a follower
    '''
    a, b, d = direction_line_equation(Robot)

    for n in Robot.neighbors:
        if n.AS != Robot.AS:
            continue  #we don't care now
        if neighbor_check(Robot, n, a, b, d):
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
    Change the Robot direction to approach the given neighbor.
    '''
    Robot.dir_x = (neighbor.x - Robot.x)
    Robot.dir_y = (neighbor.y - Robot.y)
    #proportionally dividing the dir value by 2
    while abs(Robot.dir_x) > 1 or abs(Robot.dir_y) > 1:
        Robot.dir_x /= 2
        Robot.dir_y /= 2
    '''        
    rd = relative_distance(Robot.x, Robot.y, neighbor.x, neighbor.y)

    if rd > 50:
        Robot.dir_x *= 2
        Robot.dir_y *= 2        
    '''


def follower(Robot):
    '''
    Finds the neighbor which is the most close to the broadcasted direction and follows it.
    '''
    best_rd = 100000  #rd - relative distance
    best_neighbor = None
    a, b, d = direction_line_equation(Robot)
    isCollision = False
    for n in Robot.neighbors:
        if n.AS != Robot.AS:
            continue

        if not neighbor_check(Robot, n, a, b, d):
            continue

        rd = point_to_direction_rd(Robot, n)

        isCollision = is_collision_distance(Robot)
        '''
        Only robots that are on our collision distance are being considered now.
        '''

        if rd < best_rd:
            best_neighbor = n
            best_rd = rd
    if not best_neighbor:  #although it should never appear, it was decided to keep it
        return  #just in case
    if not isCollision:
        direction_to_neighbor(Robot, best_neighbor, best_rd)
    else:
        Robot.dir_x = 0
        Robot.dir_y = 0


def is_collision_distance(Robot):
    '''
    If there exists a robot that is in collision distance returns True, otherwise returns false.
    '''
    if len(Robot.neighbors) == Robot.in_range_robots:
        return False
    return True


def evaluate_velocity(Robot, const_velocity):
    '''
    Calculates the velocity of the robot on the basis of the direction. Idea is that the velocity's magnitued is constant and equal to "const_velocity" parameter.
    '''
    direction_value = math.sqrt(Robot.dir_x**2 + Robot.dir_y**2)
