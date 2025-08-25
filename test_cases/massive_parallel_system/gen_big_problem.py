# Generates a big EZPZ problem.
EXTRA_LINES = 250

print("# constraints\npoint p0\npoint p1\npoint p2")
for line in range(3, EXTRA_LINES + 3):
    print(f"point p{line}")
print("p0 = (0, 0)\nparallel(p0, p1, p1, p2)")
for line in range(1, EXTRA_LINES + 1):
    print(f"parallel(p{line}, p{line+1}, p{line+1}, p{line+2})")
print("distance(p0, p1, sqrt(32))")
print("distance(p1, p2, sqrt(32))")
for line in range(2, EXTRA_LINES + 2):
    print(f"distance(p{line}, p{line+1}, sqrt(32))")
print("p1.x = 4\n\n# guesses")

print("p0 roughly (0,0)")
print("p1 roughly (3,3)")
print("p2 roughly (6,6)", end="")
for line in range(3, EXTRA_LINES + 3):
    print(f"\np{line} roughly ({6+line},{6+line})", end="")
