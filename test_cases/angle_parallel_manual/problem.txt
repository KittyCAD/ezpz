# constraints
point p0
point p1
point p2
point p3
p0 = (0, 0)
p2 = (0, 0)
lines_at_angle(p0, p1, p2, p3, 720deg)
distance(p0, p1, sqrt(32))
distance(p2, p3, sqrt(32))
p1.x = 4

# guesses
p0 roughly (0,0)
p1 roughly (3,3)
p2 roughly (0,0)
p3 roughly (3,3)
