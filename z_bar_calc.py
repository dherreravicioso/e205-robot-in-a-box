import math

theta = math.radians(30) # heading
x = 0 # x position
y = 0 # y position




def z_bar_calc(theta, x, y):

    x_tof = 0 # predicted x tof
    y_tof = 0 # predicted y tof

    L = 0.8 # box side length
    x_offset = 0.05 # sensor x offset from center
    y_offset = 0.07 # sensor y offset from center

    # x calculation

    if theta >= 0 and theta <= math.pi/2: # 0 to 90 degrees

        if y + math.tan(theta+3*math.pi/2)*(L/2-x) > -L/2: # projection above bottom wall - RIGHT WALL - case is good!
            x_tof = (L/2-x)/math.cos(theta-math.pi/2)-x_offset
        else: # projection below bottom wall - BOTTOM WALL - case is good!
            x_tof = (L/2+y)/math.cos(-theta)-x_offset

    elif theta >= math.pi/2 and theta <= math.pi:  # 90 to 180 degrees

        if y + math.tan(theta-math.pi/2)*(L/2-x) < L/2: # projection below top wall - RIGHT WALL - case is good!
            x_tof = (L/2-x)/math.cos(theta-math.pi/2)-x_offset
        else: # projection above top wall - TOP WALL - case is good!
            x_tof = (L/2-y)/math.cos(math.pi-theta)-x_offset

    elif theta >= math.pi and theta <= 3*math.pi/2:  # 180 to 270 degrees

        if y + math.tan(3*math.pi/2-theta)*(L/2+x) < L/2: # projection below top wall - LEFT WALL - case is good!
            x_tof = (L/2+x)/math.cos(3*math.pi/2-theta)-x_offset
        else: # projection above top wall - TOP WALL - case is good!
            x_tof = (L/2-y)/math.cos(math.pi-theta)-x_offset

    else: # 270 to 360 degrees

        if y + math.tan(3*math.pi/2-theta)*(L/2+x) > -L/2: # projection above bottom wall - LEFT WALL - case is good!
            x_tof = (L/2+x)/math.cos(3*math.pi/2-theta)-x_offset
        else: # projection below bottom wall - BOTTOM WALL - case is good!
            x_tof = (L/2+y)/math.cos(-theta)-x_offset

    theta += math.pi/2

    if theta >= 2*math.pi:
        theta -= 2*math.pi

    # y calculation

    if theta >= 0 and theta <= math.pi/2: # 0 to 90 degrees

        if y + math.tan(theta+3*math.pi/2)*(L/2-x) > -L/2: # projection above bottom wall - RIGHT WALL - case is good!
            y_tof = (L/2-x)/math.cos(theta-math.pi/2)-y_offset
        else: # projection below bottom wall - BOTTOM WALL - case is good!
            y_tof = (L/2+y)/math.cos(-theta)-y_offset

    elif theta >= math.pi/2 and theta <= math.pi:  # 90 to 180 degrees

        if y + math.tan(theta-math.pi/2)*(L/2-x) < L/2: # projection below top wall - RIGHT WALL - case is good!
            y_tof = (L/2-x)/math.cos(theta-math.pi/2)-y_offset
        else: # projection above top wall - TOP WALL - case is good!
            y_tof = (L/2-y)/math.cos(math.pi-theta)-y_offset

    elif theta >= math.pi and theta <= 3*math.pi/2:  # 180 to 270 degrees

        if y + math.tan(3*math.pi/2-theta)*(L/2+x) < L/2: # projection below top wall - LEFT WALL - case is good!
            y_tof = (L/2+x)/math.cos(3*math.pi/2-theta)-y_offset
        else: # projection above top wall - TOP WALL - case is good!
            y_tof = (L/2-y)/math.cos(math.pi-theta)-y_offset

    else: # 270 to 360 degrees

        if y + math.tan(3*math.pi/2-theta)*(L/2+x) > -L/2: # projection above bottom wall - LEFT WALL - case is good!
            y_tof = (L/2+x)/math.cos(3*math.pi/2-theta)-y_offset
        else: # projection below bottom wall - BOTTOM WALL - case is good!
            y_tof = (L/2+y)/math.cos(-theta)-y_offset

    return [x_tof, y_tof]

print(z_bar_calc(theta,x,y))