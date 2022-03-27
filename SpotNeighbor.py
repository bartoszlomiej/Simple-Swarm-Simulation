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
    thus |x| = r/sqrt(1 + tan(i(degree))); where 

    from the circle equation x^2 + y^2 = R^2 and we know that y = tan(i(degree)) * x
    thus |x| = r/sqrt(1 + tan(i(degree))); where 
    '''
    radians = mp.radians(i * (360) / k)
    cot = abs(mp.cot(radians))  #1 / math.tan(radians)
    if i > (k / 2):
        cot *= -1
    if cot < 0:
        return int(round(-r / math.sqrt(1 + abs(cot))))
    return int(round(r / math.sqrt(1 + cot)))


def is_neighbor_spotted(Robot, x, y, radius):
    '''
    Checks whether neighbor is being spotted on the given coordinates.
    Returns True if neighbor is spotted, otherwise returns False
    '''
    for n in Robot.neighbors:
        if ((n.x - x)**2 + (n.y - y)**2 <= radius**2):
            return True
    return False


def check_line(Robot, i, k=15, R=75):
    '''
    Checks the given line (k - the given fraction of the circle).

    Returns True, if other robot is being spotted, otherwise returns False.
    '''
    radius = 10  #radius of the robot
    step_size = 2  #the step of line that is being checked, it doesn't need to be 1, but too big can crash
    for r in range(radius + 1, R + 1, step_size):
        x = calc_x(i, r, k) + Robot.x
        y = calc_y(i, r, k) + Robot.y
        if is_neighbor_spotted(Robot, x, y, radius):
            return True
    return False


def check_x0_line(Robot, R=75):
    '''
    Special case of check line - checks the line for the x = 0.
    '''
    radius = 10  #radius of the robot
    step_size = 2
    for y in range(radius + 1, R + 1, step_size):
        if is_neighbor_spotted(Robot, Robot.x, Robot.y + y, radius):
            return True
    return False


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
        if x_b * a + b < y_b:
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
    if not A and not B:
        return 1000
    return abs((neighbor.x * A) +
               (neighbor.y * B) + C) / math.sqrt(A**2 + B**2)


def direction_to_neighbor(Robot, neighbor):
    '''
    Change the Robot direction to approach the given neighbor.

    The dir_x, dir_y values assumes that the direction is always given on the unit circle.
    '''
    delta_x = (neighbor.x - Robot.x)
    delta_y = (neighbor.y - Robot.y)
    suma = math.sqrt(delta_x**2 + delta_y**2)
    Robot.dir_x = delta_x / suma
    Robot.dir_y = delta_y / suma

    rd = relative_distance(Robot.x, Robot.y, neighbor.x, neighbor.y)
    if rd > 30:
        Robot.dir_x *= 1.5
        Robot.dir_y *= 1.5


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

        if rd < best_rd:
            best_neighbor = n
            best_rd = rd
    if not best_neighbor:  #although it should never appear, it was decided to keep it
        return  #just in case
    '''
    Only robots that are on our collision distance are being considered now.
    '''
    if not is_collision_distance(Robot):
        direction_to_neighbor(Robot, best_neighbor)
    else:
        direction_to_neighbor(Robot, best_neighbor)
        if not is_any_collision(Robot):
            return
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


def is_any_collision(Robot):
    '''
    Checks whether there is going to be a collision in the direciton we are approaching
    '''
    a, b, d = direction_line_equation(Robot)
    for n in Robot.neighbors:
        if neighbor_check(Robot, n, a, b, d):
            if (n.x - Robot.x)**2 + (n.y -
                                     Robot.y)**2 < 0.15 * Robot.s_range**2:
                return True
    return False


def is_collision(Robot):
    '''
    If leader spots the obstacle (other robot) in front of it then the new path needs to be recalculated
    '''
    a, b, d = direction_line_equation(Robot)
    for n in Robot.neighbors:
        if n.AS != Robot.AS:
            if neighbor_check(Robot, n, a, b, not d):
                return True
    return False


def evaluate_velocity(Robot, const_velocity):
    '''
    Calculates the velocity of the robot on the basis of the direction. Idea is that the velocity's magnitued is constant and equal to "const_velocity" parameter.
    '''
    direction_value = math.sqrt(Robot.dir_x**2 + Robot.dir_y**2)
