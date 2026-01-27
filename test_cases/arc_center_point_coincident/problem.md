# constraints
point line3start
point line3end
point line4start
point line4end
point arc1center
point arc1a
point arc1b
arc arc1
line(line3start, line3end)
line(line4start, line4end)
is_arc(arc1)
coincident(arc1center, line3end)
point_line_distance(arc1b, line3start, line3end, 0)
point_arc_coincident(line4start, arc1)

# guesses
line3start roughly (4.41, 3.68)
line3end roughly (0.55, -3.31)
line4start roughly (-1.16, -2.63)
line4end roughly (-6.71, -2.8)
arc1center roughly (0.55, -3.31)
arc1a roughly (2.25, -3.99)
arc1b roughly (1.43, -1.71)
arc1.center roughly (0.55, -3.31)
arc1.a roughly (2.25, -3.99)
arc1.b roughly (1.43, -1.71)
