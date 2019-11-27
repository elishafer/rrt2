# Path Planning Algos



## RRT*
Used the path planningalgos from Karaman and Frizzoli 2011. 

γRRG* = 2 ( 1 + 1/d)^(1/d) ( μ( Xfree) /
ζ_d)^(1/d)

Where:

- d - dimension of the space.
- μ( Xfree) - Volume of free space
- ζ_d - Volume of unit ball for d dimensional euclidean space.

for 2 dimensions:
- d = 2
- μ( Xfree) = approx size of map
- ζ_d = 2 pi

Thus:

γRRG* = sqrt(6) * μ( Xfree)^(1/d)/sqrt(2pi)

γRRG* = sqrt(3/pi) * sqrt(μ( Xfree))

γRRG* = 0.9772 * sqrt(μ( Xfree))

γRRG has to be greater than γRRG*. Hence we'll multiply our result by some parameter.

