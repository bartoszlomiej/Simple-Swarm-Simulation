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
'''
def add_weight(Robot, x, y, W=30):

#    For given x,y there is added the weight W (depending how far away this point P(x,y) is from the center of weight
#    W(xi', yi') - the neighbor i coordinates

    weight = 0
    for n in Robot.neighbors:
        if (abs(n.x - x) <= (W / 2)) and (abs(n.y - y) <= (W / 2)):
            weight = weight + W - (abs(n.x - x) + abs(n.y - y))
    return weight
    '''


def add_weight(Robot, x, y, distance, R=20):
    '''
    For given x,y there is added the weight W (depending how far away this point P(x,y) is from the center of weight
    distance - is equal to the current radius of the line that is being checked
    R - radius of a robot
    '''
    for n in Robot.neighbors:
        if (abs(n.x - x) <= (R / 2)) and (abs(n.y - y) <= (R / 2)):
            return distance
    return 0


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
    x_b, y_b = Robot.dir_x, Robot.dir_y
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
        a = (y_a - y_b) / (x_a - x_b)
        a = -1 / a
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
    follower = False
    a, b, d = direction_line_equation(Robot)
    rd = 0  #relative distance to closest neighbor
    best_neighbor = None
    best_direction = 0

    x, y = 0, 0

    for n in Robot.neighbors:
        if n.AS != Robot.AS:
            return False
        if d:
            if n.y > (n.x * a) + b:
                follower = True
        else:
            if n.y < (n.x * a) + b:
                follower = True

        if follower and rd == 0:
            rd = relative_distance(Robot.x, Robot.y, n.x, n.y)
            if rd == 0:
                Robot.dir_x, Robot.dir_y = 1, 1
                return True
            x = ((n.x - Robot.x) / rd)**2
            y = ((n.y - Robot.y) / rd)**2
            best_direction = math.sqrt((x - Robot.dir_x)**2 +
                                       (y - Robot.dir_y)**2)
            closest_neighbor = n

        elif follower:
            if rd == 0:
                dir_x, dir_y = 1, 1
                return True
            buffer_rd = relative_distance(Robot.x, Robot.y, n.x, n.y)
            x = ((n.x - Robot.x) / rd)**2
            y = ((n.y - Robot.y) / rd)**2
            buffer_best_direction = math.sqrt((x - Robot.dir_x)**2 +
                                              (y - Robot.dir_y)**2)
            if buffer_best_direction < best_direction:
                best_direction = buffer_best_direction
                rd = buffer_rd
                closest_neighbor = n
    if follower:
        Robot.dir_x = ((n.x - Robot.x) / rd)**2
        Robot.dir_y = ((n.y - Robot.y) / rd)**2
        while Robot.dir_x > 1:
            Robot.dir_x /= 2

        while Robot.dir_y > 1:
            Robot.dir_y /= 2

        if rd < 25:  #robots are relatively close one to another
            Robot.dir_x *= 0.5
            Robot.dir_y *= 0.5
        elif rd > 55:
            Robot.dir_x *= 1.5
            Robot.dir_y *= 1.5
        if rd < 15:  #robots are almost in collision
            Robot.dir_x *= -1
            Robot.dir_y *= -1


#follow the neighbor
    return follower
'''
Problems:
is_follower or find_direction is extremely time & resources consuming. Moreover, it is hard to test both.

-is_follower -> not really follows
-find_direction - doesn't avoid collisions (especially with other AS's ???) (probably extremely time consuming)

-when wall is encounter they don't know how to behave
-collision avoidance doesn't work
(it used to work in previous commits; highly probable that it was the is_follow function XD) and it was efficient
-distances are not kept
'''
