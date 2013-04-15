from scipy.optimize import fsolve

def f(r):
    x = float(r[0])
    y = float(r[1])
    z = float(r[2])
    return [
        (x+0.00966)**2 + (y-0.267892)**2 + (z-1.197412)**2 - 0.09**2,
        (x+0.08347)**2 + (y-0.406319)**2 + (z-1.04607)**2 - 0.10855**2,
        x + 0.00966
    ]

result = fsolve(f, [1,1, 1])

print result
print f(result)
