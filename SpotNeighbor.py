import math
import numpy as np


def degree_tangent(i, k=13):
    """
    Calculates the angle proper for the tangent.
    """
    degree = i * (360 / k)
    if degree > 0 and degree < 90:
        return degree
    elif degree > 90 and degree < 180:
        return 180 - degree
    elif degree > 180 and degree < 270:
        return degree - 180
    else:
        return 360 - degree


def calc_y(i, r, k=13):
    '''
    Calculates the value of the y - in order to find the neighbors.
    k - is the number of equal partions that will always spot the neighbor that is on the border of the radius
    k for the range radius R = 55 is equal 13
    i - is the current line that is being calculated (i = {1, 2,..., k-1}
#    x - is the x on the axis; x = [0; R] for i <= [k/2] or x=[-R;0] otherwise
    r - is the radius of the given circle
    '''
    x = calc_x(i, r, k)
    return int(round(math.sqrt(r**2 - x**2))) * np.sign(r)


def calc_x(i, r, k=13):
    '''
    Calculates the value of the x direction - in order to find the neighbors.
    k - is the number of equal partions that will always spot the neighbor that is on the border of the radius
    k for the range radius R = 55 is equal 13
    i - is the current line that is being calculated (i = {1, 2,..., k-1}
    x - is the x on the axis; x = [0; R] for i <= [k/2] or x=[-R;0] otherwise

    from the circle equation x^2 + y^2 = 1 and we know that y = tan(i(degree)) * x
    thus |x| = x/sqrt(1 + tan(i(degree))); where 
    '''
    radians = degree_tangent(i, k) * (math.pi / 180)
    cot = 1 / math.tan(radians)
    if cot < 0:
        if r < 0:
            r = r * -1  #special case!
        return int(round(-r / math.sqrt(1 + abs(cot))))
    return int(round(r / math.sqrt(1 + cot)))


#        return (-r / math.sqrt(1 + abs(cot)))
#    return (r / math.sqrt(1 + cot))


def add_weight(Robot, x, y, W=30):
    '''
    For given x,y there is added the weight W (depending how far away this point P(x,y) is from the center of weight
    W(xi', yi') - the neighbor i coordinates
    '''
    weight = 0
    for n in Robot.neighbors:
        if abs(n.x - x) <= W / 2 and abs(n.y - y) <= W / 2:
            weight = weight + W - (abs(n.x - x) + abs(n.y - y))
    return weight


def check_line(Robot, i, k=13, R=55):
    line_weight = 0
    if i <= math.ceil(k / 4) or i >= math.floor(3 * k / 4):
        for x in range(10, R, 1):  #the robot radius is 10!w
            y = calc_y(i, x, k)
            line_weight += add_weight(Robot, x, y)
    else:
        for x in range(-R, -10, 1):  #the robot radius is 10!w
            y = calc_y(i, x, k)
            line_weight += add_weight(Robot, x, y)
    return line_weight


def check_x0_line(Robot, R=55):
    line_weight = 0
    for y in range(10, R, 1):  #the robot radius is 10!
        line_weight += add_weight(Robot, 0, y)
    return line_weight
