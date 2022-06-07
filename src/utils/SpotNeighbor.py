import math
import mpmath as mp
from simulation.robot.Direction import Direction
from copy import deepcopy
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


def calculate_slope(x0, y0, x1, y1):
    '''
    Returns the slope of the given line (a coefficient in y = ax + b)
    '''
    return (y0 - y1) / (x0 - x1)


def is_neighbor_spotted(robot, x, y, radius):
    '''
    Checks whether neighbor is being spotted on the given coordinates.
    Returns True if neighbor is spotted, otherwise returns False
    '''
    for n in robot.neighbors:
        if ((n.position.x - x)**2 + (n.position.y - y)**2 <= radius**2):
            return True
    return False


def isNeighborOnSensor(robot, a, b, d, radius):
    '''
    Checks whether neighbor is being spotted on the given coordinates.
    Returns True if neighbor is spotted, otherwise returns False
    '''
    for n in robot.neighbors:
        if abs((a * n.position.x - n.position.y + b) /
               math.sqrt(a**2 + 1)) <= radius:
            if (n.position.x > robot.position.x) and d:
                return True
            elif (n.position.x <= robot.position.x) and not d:
                return True
    return False


def check_line(robot, i, k=15, R=75):
    '''
    Checks the given line (k - the given fraction of the circle).

    Returns True, if other robot is being spotted, otherwise returns False.
    '''
    radius = robot.radius  #radius of the robot
    a = robot.sensors[i]
    b = robot.sensorFunction(i)
    return isNeighborOnSensor(robot, a, b, True if i < k / 2 else False,
                              radius)


def check_x0_line(robot, R=75):
    '''
    Special case of check line - checks the line for the x = 0.
    '''
    radius = robot.radius  #radius of the robot
    step_size = 2
    for y in range(radius + 1, R + 1, step_size):
        if is_neighbor_spotted(robot, robot.position.x, robot.position.y + y,
                               radius):
            return True
    return False


def direction_line_equation(robot):
    '''
    Returns a - the coefficient of the line prependicular to the direction (y = ax + b),
    b - similarly
    d - the half of the space in which we are searching for other robots
    '''
    x_a, y_a = robot.position.x, robot.position.y
    x_b, y_b = (robot.direction.x * 100) + robot.position.x, (
        robot.direction.y * 100) + robot.position.y
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
        a = -robot.direction.x / robot.direction.y
        b = robot.position.y - (robot.position.x * a)
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


def neighbor_check(robot, neighbor, a, b, d):
    '''
    Checks if given robot can possibly be followed by this robot.
    returns true if the robot can be followed. Otherwise returns false.
    '''
    if d:
        if neighbor.position.y > (neighbor.position.x * a) + b:
            return True
    else:
        if neighbor.position.y < (neighbor.position.x * a) + b:
            return True
    return False


def extended_neighbor_check(robot, neighbor, a, b, d):
    '''
    Checks if there is a neighbor in about direction the robot would like to follow
    '''
    if d:
        if neighbor.position.y > (neighbor.position.x * a) + b:
            return True
    else:
        if neighbor.position.y < (neighbor.position.x * a) + b:
            return True
    return False


def is_follower(robot):
    '''
    if there are any neighbors in our direction -> I am follower; Otherwise I am the leader.
    Returns direction and information if it is a follower
    '''
    a, b, d = direction_line_equation(robot)

    for n in robot.neighbors:
        if n.cluster_id != robot.cluster_id:
            continue  #we don't care now
        if neighbor_check(robot, n, a, b, d):
            return True
    return False


def point_to_direction_rd(robot, neighbor):
    '''
    Calculating the distance from neighbor to direction line
    rd = (Ax0 + By0 + C) / sqrt(A^2 + B^2)
    '''
    A = robot.direction.x
    B = robot.direction.y
    C = robot.position.y - (robot.position.x * A)
    if not A and not B:
        return 1000
    return abs((neighbor.position.x * A) +
               (neighbor.position.y * B) + C) / math.sqrt(A**2 + B**2)


def direction_to_neighbor(robot, neighbor):
    '''
    Change the robot direction to approach the given neighbor.

    The direction.x, direction.y values assumes that the direction is always given on the unit circle.
    '''
    delta_x = (neighbor.position.x - robot.position.x
               )  #it must be greater than 0 - robots cannot overlap
    delta_y = (neighbor.position.y - robot.position.y)
    suma = math.sqrt(delta_x**2 + delta_y**2)
    robot.direction.x = delta_x / suma
    robot.direction.y = delta_y / suma

    rd = relative_distance(robot.position.x, robot.position.y,
                           neighbor.position.x, neighbor.position.y)
    if rd > 30:
        robot.direction.x *= 1.5
        robot.direction.y *= 1.5


def find_best_neighbor(robot, closestNeighbor=False, allowedAS=None):
    '''
    Returns the best neighbor as well as it's relative distance.
    If closestNeighbor flag is set to True, than instead of best neighbor in terms of phase 2 algorithm
    the closest neighbor is being returned
    '''
    best_rd = 100000  #rd - relative distance
    best_neighbor = None
    a, b, d = direction_line_equation(robot)
    for n in robot.neighbors:
        if not allowedAS:
            if n.cluster_id != robot.cluster_id:
                continue
        else:
            if not n.cluster_id in allowedAS:
                continue

        if not neighbor_check(robot, n, a, b, d):
            continue
        if not closestNeighbor:
            rd = point_to_direction_rd(robot, n)
        else:
            rd = relative_distance(robot.position.x, robot.position.y,
                                   n.position.x, n.position.y)

        if rd < best_rd:
            best_neighbor = n
            best_rd = rd
    return best_neighbor, best_rd


def follower(robot):
    '''
    Finds the neighbor which is the most close to the broadcasted direction and follows it.
    '''
    best_neighbor, best_rd = find_best_neighbor(robot)
    if not best_neighbor:  #although it should never appear, it was decided to keep it
        return  #just in case

    direction_to_neighbor(robot, best_neighbor)


def is_collision_distance(robot):
    '''
    If there exists a robot that is in collision distance returns True, otherwise returns false.
    '''
    if len(robot.neighbors) == robot.detected_robots_number:
        return False
    return True

def isBorderReturn(robot, next_move=False):
    if next_move:
        future_position = deepcopy(robot.position)
        future_position.moveBy(robot.velocity)
    else:
        future_position = robot.position
    x_is_out = False
    y_is_out = False
    if future_position.x <= 2 or future_position.x > robot.board_resolution.width - 2 * robot.radius - 2:
        x_is_out = True
    if future_position.y <= 2 or future_position.y > robot.board_resolution.height - 2 * robot.radius - 2:
        y_is_out = True
    return x_is_out, y_is_out

def border_return(robot, next_move=False):
    is_x, is_y = isBorderReturn(robot, next_move)
    if is_x:
        robot.direction.x = -robot.direction.x
        robot.velocity.x = -robot.velocity.x
    if is_y:
        robot.direction.y = -robot.direction.y
        robot.velocity.y = -robot.velocity.y
        
def is_any_collision(robot, min_d = 0.3):
    '''
    Checks whether there is going to be a collision in the direciton we are approaching
    '''
    a, b, d = direction_line_equation(robot)
    minimal_distance =  min_d * (robot.sensor_range - robot.radius)**2 + robot.radius**2
    for n in robot.neighbors:
        if neighbor_check(robot, n, a, b, d):
            if (n.position.x - robot.position.x)**2 + (n.position.y -
                                     robot.position.y)**2 < minimal_distance:
                return True
    return False


def is_collision(robot):
    '''
    If leader spots the obstacle (other robot) in front of it then the new path needs to be recalculated
    '''
    a, b, d = direction_line_equation(robot)
    for n in robot.neighbors:
        if n.cluster_id != robot.cluster_id:
            if neighbor_check(robot, n, a, b, not d):
                return True
    return False


def evaluate_velocity(robot, const_velocity):
    '''
    Calculates the velocity of the robot on the basis of the direction. Idea is that the velocity's magnitued is constant and equal to "const_velocity" parameter.
    '''
    direction_value = math.sqrt(robot.direction.x**2 + robot.direction.y**2)
