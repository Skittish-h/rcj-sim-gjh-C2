#basic coord recalculation
def coor_recalc(x,y, team=True):
  x = (x+0.75)/1.5
  y = 1-(y+0.65)/1.3
  return {"x":x,"y":y}

#recalculates robot positions into better 
def robot_pos_recalc(robot_pos: dict, Team=True):
  temp_orientation = robot_pos["orientation"]
  
  new = coor_recalc(robot_pos['x'], robot_pos['y'], team=Team)
  new['orientation'] = temp_orientation
  return new
