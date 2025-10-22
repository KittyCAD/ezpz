# constraints
point a
point b
point c
point d
lines_equal_length(a, b, c, d)
lines_equal_length(b, c, a, d)
horizontal(a, b)
vertical(b, c)
parallel(a, b, c, d)
parallel(b, c, d, a)
a = (0, 0)
c = (4, 4)

# guesses
a roughly (3, 4)
b roughly (5, 6)
c roughly (3, 5)
d roughly (5, 8)
