# constraints
point a
point b
point c
point d
point e
line(a, b)
line(b, c)
line(c, d)
line(d, e)
line(e, a)
vertical(e, a)
e = (0, 0)
distance(c, d, 30)
distance(d, e, 40)
horizontal(d, e)
horizontal(a, b)
vertical(c, d)
lines_equal_length(d, e, e, a)
lines_equal_length(a, b, c, d)

# guesses
a roughly (0, 10)
b roughly (10, 10)
c roughly (10, 5)
d roughly (10, 0)
e roughly (0.1, 0.1)
