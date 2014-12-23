Fast cubic Bézier curve lenght calculation library and benchmark.

We test 3 methods:
1. The easiest one - line interpolation: We split the curve into n points, and return the sum of the distances between points.
2. The fastest one - midpoint quadratic interpolation: We interpolate the cubic curve with quadratic one and calculate it's length from integral.
3. The most precise - adaptive quadratic interpolation: We interpolate segments of the cubic curve with multiple quadratic curves
   and return the sum of their lengths calculated with integral.

Test sample:

Short railway track segment, length: ca 33m.
For the line interpolation the precision was set to 0.05 (5cm).
For integral interpolations the presision is 0.001 (1mm).

License and legal stuff:

Free / Open Source, all the hard work done by Adrian Colomitchi and Dave Eberly, I only made a C# library and benchmark.
For more details and theory visit:
http://caffeineowl.com/graphics/2d/vectorial/
http://www.gamedev.net/topic/551455-length-of-a-generalized-quadratic-bezier-curve-in-3d/