# constraints
point p
arc a
a.center.x = 0
a.center.y = 0
a.a.x = 0
arc_radius(a, 5)

# guesses
p roughly (4, 3)
a.center roughly (0.1, 0.2)
a.a roughly (0, 4)
a.b roughly (4, 0)
