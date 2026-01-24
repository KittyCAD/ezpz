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

# guesses
line3start roughly (4.32, 3.72)
line3end roughly (1.06, -3.26)
line4start roughly (-2.32, -2.96)
line4end roughly (-7.01, -2.77)
arc1center roughly (1.06, -3.26)
arc1a roughly (-1.44, -0.99)
arc1b roughly (2.49, -0.2)
arc1.center roughly (1.06, -3.26)
arc1.a roughly (-1.44, -0.99)
arc1.b roughly (2.49, -0.2)
