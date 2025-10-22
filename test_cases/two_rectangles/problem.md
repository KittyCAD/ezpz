# constraints
point p0
point p1
point p2
point p3
line(p0, p1)
line(p1, p2)
line(p2, p3)
line(p3, p0)
p0 = (1,1)
horizontal(p0, p1)
horizontal(p2, p3)
vertical(p1, p2)
vertical(p3, p0)
distance(p0, p1, 4)
distance(p0, p3, 3)
point p4
point p5
point p6
point p7
p4 = (2, 2)
horizontal(p4, p5)
horizontal(p6, p7)
vertical(p5, p6)
vertical(p7, p4)
distance(p4, p5, 4)
distance(p4, p7, 4)

# guesses
p0 roughly (1,1)
p1 roughly (4.5,1.5)
p2 roughly (4.0,3.5)
p3 roughly (1.5,3.0)
p4 roughly (2,2)
p5 roughly (5.5,3.5)
p6 roughly (5,4.5)
p7 roughly (2.5,4)
