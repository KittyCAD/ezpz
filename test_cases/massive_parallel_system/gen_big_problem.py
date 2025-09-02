# Generates a big EZPZ problem.
import sys


def get_overconstrain():
    try:
        return sys.argv[2] == "true"
    except IndexError:
        return False


total_lines = int(sys.argv[1])
overconstrain = get_overconstrain()

print("# constraints")
for line in range(total_lines):
    a = line * 2
    b = line * 2 + 1
    print(f"point p{a}")
    print(f"point p{b}")
    print(f"vertical(p{a}, p{b})")
    print(f"p{a}.x={line}")
    print(f"p{a}.y=0")
    print(f"p{b}.y=4")
    if overconstrain:
        print(f"distance(p{a}, p{b}, 4)")

print()
print("# guesses")
for line in range(total_lines):
    start = line * 2
    print(f"p{start} roughly ({start},{start})")
    print(f"p{start+1} roughly ({start+1},{start+1})")
