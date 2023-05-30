# -*- Mode: Python; indent-tabs-mode: nil; py-indent-offset: 4; tab-width: 4 -*-
# vim: set expandtab tabstop=4 shiftwidth=4 softtabstop=4:

import math

def dist_to_coord_linear(target, pos, dist):
    """returns coordinate with distance from target linear to pos1
    useful for position robot away from target """
    x1, y1, q1 = target
    x2, y2, q2 = pos
    th = math.atan2(y1-y2,x1-x2)
    x3 = x1 - (dist*math.cos(th))
    y3 = y1 - (dist*math.sin(th))
    return [int(x3),int(y3),0]

def normalize_rad(ang):
    while ang > math.pi:
        ang = ang - 2*math.pi
    while ang < -math.pi:
        ang = ang + 2*math.pi
    return ang

def distance(pos):
    x, y, q = pos
    return math.sqrt(x**2+y**2)

def distance2points(pos1, pos2):
    """ Returns distance (Euclid norm) between two points.
    Usage: distance = Distance([x1, y1, 0],[x2, y2, 0])
    note: third parameter is necessary, but ommited in calculation
    written by kaminaga"""
    x1, y1, q1 = pos1
    x2, y2, q2 = pos2
    return math.sqrt((x1-x2)**2+(y1-y2)**2)

def distance_within(selfpos, destpos, within_dist):
    return within_dist > distance2points(selfpos, destpos) > 0

def direction_rad(pos):
    """ Returns argument angle of the specified point from origin in rad.
    Usage: angle = DirectionRad([x, y, 0])
    note: third parameter is necessary, but ommited in calculation
    written by kaminaga"""
    x, y, q = pos
    if ((x == 0) and (y == 0)):
        return 0
    else:
        return math.atan2(y, x)

def direction_deg(pos):
    """ Returns argument angle of the specified point from origin in deg.
    Usage: angle = DirectionDeg([x, y, 0])
    note: third parameter is necessary, but ommited in calculation
    written by kaminaga"""
    return math.degrees(direction_rad(pos))

def direction_diff(d1, d2):
    """ Returns dirrence of 2 angle in [-PI:PI]
    Author:irie
    """
    diff = d1-d2
    while diff > math.pi:
        diff = diff - 2*math.pi
    while diff < -math.pi:
        diff = diff + 2*math.pi
    return diff

def direction_within(self_th, dest_angle, within_angle):
    return math.fabs(normalize_rad(dest_angle - self_th)) < within_angle

def rotation2d(pos, dq):
    """ Returns rotated 3 element list.
    Usage: x2, y2, q2 = Rotation2D([x, y, q], dq)
    note: dq[rad]
    written by ayusawa"""

    x, y, q = pos
    
    x2 = math.cos(dq) * x - math.sin(dq) * y
    y2 = math.sin(dq) * x + math.cos(dq) * y
    q2 = normalize_rad(q + dq)

    return (x2, y2, q2)

def coord_trans_local_to_global(global_pos_A, local_pos_B):
    """ Calculate forward kinematics and returns global 3 element list
    Usage: g_xb, g_yb, g_qb = CoordTransLocalToGlobal([g_xa, g_ya, g_qa], [l_xb, l_yb, l_qb])
    note: g_qa[rad], g_qb[rad], l_qb[rad]
    written by ayusawa"""
    
    g_xa, g_ya, g_qa = global_pos_A
    tmp_xb, tmp_yb, g_qb = rotation2d(local_pos_B, g_qa)
    g_xb = g_xa + tmp_xb
    g_yb = g_ya + tmp_yb
    
    return (g_xb, g_yb, normalize_rad(g_qb))

def coord_trans_global_to_local(global_pos_A, global_pos_B):
    """ Calculate forward kinematics and returns local 3 element list
    Usage: l_x_ab, l_yab, l_qab = CoordTransGlobalToLocal([g_xa, g_ya, g_qa], [g_xb, g_yb, g_qb])
    note: g_qa[rad], g_qb[rad], l_qb[rad]
    written by ayusawa"""
    
    g_xa, g_ya, g_qa = global_pos_A
    g_xb, g_yb, g_qb = global_pos_B
    global_pos_AB = (g_xb-g_xa, g_yb-g_ya, normalize_rad(g_qb - g_qa))
    l_xb, l_yb, tmp_qb = rotation2d(global_pos_AB, -g_qa)

    return [l_xb, l_yb, normalize_rad(global_pos_AB[2])]

def intervene_in_between_two_points(global_pos_A,global_pos_B,dist=None):

    g_xa, g_ya, g_qa = global_pos_A
    g_xb, g_yb, g_qb = global_pos_B

    r = math.sqrt((g_xa-g_xb)**2 + (g_ya-g_yb)**2)

    if r <= dist or dist == None:
        mid_gx = (g_xa + g_xb)/2
        mid_gy = (g_ya + g_yb)/2
    else: 
        mid_gx = (g_xa*dist + g_xb*(r-dist))/r
        mid_gy = (g_ya*dist + g_yb*(r-dist))/r

    return[mid_gx,mid_gy,0]
