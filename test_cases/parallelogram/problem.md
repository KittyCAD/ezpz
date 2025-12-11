# constraints
point a
point b
point c
point d
lines_equal_length(a, b, c, d)
lines_equal_length(b, d, a, c)
parallel(a, b, c, d)
vertical(a, b)
vertical(c, d)
a = (0, 0)
b.y = 4

# guesses
a roughly (3, 4)
b roughly (5, 6)
c roughly (3, 5)
d roughly (5, 8)
