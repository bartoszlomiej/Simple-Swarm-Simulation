import math


def calc_y(i, x, k=13):
    '''
    Calculates the value of the y - in order to find the neighbors.
    k - is the number of equal partions that will always spot the neighbor that is on the border of the radius
    k for the range radius R = 55 is equal 13
    i - is the current line that is being calculated (i = {1, 2,..., k-1}
    x - is the x on the axis; x = [0; R] for i <= [k/2] or x=[-R;0] otherwise
    '''
    radians = i * (360 / k) * (math.pi / 180)
    return int(round(math.tan(radians) * x))


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
    if i <= math.ceil(k / 2):
        for x in range(R):
            y = calc_y(i, x, k)
            line_weight += add_weight(Robot, x, y)
    else:
        for x in range(-R, 0, 1):
            y = calc_y(i, x, k)
            line_weight += add_weight(Robot, x, y)
    return line_weight


def check_x0_line(Robot, R=55):
    line_weight = 0
    for y in range(R):
        line_weight += add_weight(Robot, 0, y)
    return line_weight
