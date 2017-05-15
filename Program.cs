using System;
using System.Collections.Generic;
using System.Linq;

namespace FastBezier {

    /// <summary>
    /// Represents 3D vector structure.
    /// </summary>
    struct V3D {
        
        /// <summary>
        /// X coordinate.
        /// </summary>
        public double X;
        
        /// <summary>
        /// Y coordinate.
        /// </summary>
        public double Y;
        
        /// <summary>
        /// Z coordinate;
        /// </summary>
        public double Z;
        
        /// <summary>
        /// Test if both vectors are equal.
        /// </summary>
        /// <param name="obj">The object the equality is tested with.</param>
        /// <returns></returns>
        public override bool Equals(object obj) => obj is V3D v && v.X == X && v.Y == Y && v.Z == Z;
        
        /// <summary>
        /// Returns the hash code calculated for this instance.
        /// </summary>
        /// <returns>Integer most likely to be unique for different vectors.</returns>
        public override int GetHashCode() => 17 * (17 * X.GetHashCode() + Y.GetHashCode()) + Z.GetHashCode();
        
        /// <summary>
        /// Gets the length of the vector.
        /// </summary>
        public double Length => Math.Sqrt(X * X + Y * Y + Z * Z);
        
        /// <summary>
        /// Gets the value indicating whether this vector is a zero vector (a vector whose all coordinates equal zero).
        /// </summary>
        public bool Zero => X == 0 && Y == 0 && Z == 0;
        
        /// <summary>
        /// Creates a new vector.
        /// </summary>
        /// <param name="x">X coordinate.</param>
        /// <param name="y">Y coordinate.</param>
        /// <param name="z">Z coordinate.</param>
        public V3D(double x, double y, double z) { X = x; Y = y; Z = z; }

        /// <summary>
        /// Binary equality test.
        /// </summary>
        /// <param name="a">Left side.</param>
        /// <param name="b">Right side.</param>
        /// <returns>True if both vectors are not null and equal.</returns>
        public static bool operator ==(V3D a, V3D b) => (object)a != null && (object)b != null && a.X == b.X && a.Y == b.Y && a.Z == b.Z;

        /// <summary>
        /// Binary inequality test.
        /// </summary>
        /// <param name="a">Left side.</param>
        /// <param name="b">Right side.</param>
        /// <returns>True if one of the vectors is null or the vectors are not equals.</returns>
        public static bool operator !=(V3D a, V3D b) => (object)a == null || (object)b == null || a.X != b.X || a.Y != b.Y || a.Z != b.Z;

        /// <summary>
        /// Vector addition.
        /// </summary>
        /// <param name="a">Left side.</param>
        /// <param name="b">Right side.</param>
        /// <returns>The sum of vectors.</returns>
        public static V3D operator +(V3D a, V3D b) => new V3D(a.X + b.X, a.Y + b.Y, a.Z + b.Z);

        /// <summary>
        /// Vector subtraction.
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns>The difference of vectors.</returns>
        public static V3D operator -(V3D a, V3D b) => new V3D(a.X - b.X, a.Y - b.Y, a.Z - b.Z);

        /// <summary>
        /// Vector negation.
        /// </summary>
        /// <param name="a">Unary argument.</param>
        /// <returns>The negative vector.</returns>
        public static V3D operator -(V3D a) => new V3D(-a.X, -a.Y, -a.Z);

        /// <summary>
        /// Unary plus. Clones the vector.
        /// </summary>
        /// <param name="a">Unary argument.</param>
        /// <returns>This vector clone.</returns>
        public static V3D operator +(V3D a) => new V3D(+a.X, +a.Y, +a.Z);

        /// <summary>
        /// Scalar product.
        /// </summary>
        /// <param name="a">Left side vector.</param>
        /// <param name="k">Right side scalar.</param>
        /// <returns>The scalar product.</returns>
        public static V3D operator *(V3D a, double k) => new V3D(k * a.X, k * a.Y, k * a.Z);

        /// <summary>
        /// Scalar product.
        /// </summary>
        /// <param name="k">Left side scalar.</param>
        /// <param name="a">Right side vector.</param>
        /// <returns>The scalar product.</returns>
        public static V3D operator *(double k, V3D a) => new V3D(k * a.X, k * a.Y, k * a.Z);

        /// <summary>
        /// Scalar quotient.
        /// </summary>
        /// <param name="a">Left side vector.</param>
        /// <param name="k">Right side scalar.</param>
        /// <returns>The scalar quotient.</returns>
        public static V3D operator /(V3D a, double k) => new V3D(a.X / k, a.Y / k, a.Z / k);

        /// <summary>
        /// Scalar quotient.
        /// </summary>
        /// <param name="k">Left side scalar.</param>
        /// <param name="a">Right side vector.</param>
        /// <returns>The scalar quotient.</returns>
        public static V3D operator /(double k, V3D a) => new V3D(k / a.X, k / a.Y, k / a.Z);

        /// <summary>
        /// Returns dot product.
        /// </summary>
        /// <param name="a">A vector.</param>
        /// <returns>Dot product.</returns>
        public double Dot(V3D a) => X * a.X + Y * a.Y + Z * a.Z;

        /// <summary>
        /// Returns cross product.
        /// </summary>
        /// <param name="a">A vector.</param>
        /// <returns>Cross product.</returns>
        public V3D Cross(V3D a) => new V3D(Y * a.Z - Z * a.Y, Z * a.X - X * a.Z, X * a.Y - Y * a.X);

        /// <summary>
        /// Returns the vector between a and b that divides vector (a - b) in t ratio. For zero it's a, for 1 it's b.
        /// </summary>
        /// <param name="a">Vector a.</param>
        /// <param name="b">Vector b.</param>
        /// <param name="t">A number between 0 and 1.</param>
        /// <returns>the vector between a and b that divides vector (a - b) in t ratio. For zero it's a, for 1 it's b.</returns>
        public static V3D Interpolate(V3D a, V3D b, double t) => new V3D(a.X * (1.0 - t) + b.X * t, a.Y * (1.0 - t) + b.Y * t, a.Z * (1.0 - t) + b.Z * t);

    }

    /// <summary>
    /// Quadratic Bézier curve calculation class.
    /// </summary>
    class Bezier2 {

        protected const double InterpolationPrecision = 0.001;

        /// <summary>
        /// Start point.
        /// </summary>
        public V3D A;
        /// <summary>
        /// Control point.
        /// </summary>
        public V3D B;
        /// <summary>
        /// End point.
        /// </summary>
        public V3D C;

        /// <summary>
        /// Creates a quadratic Bézier curve.
        /// </summary>
        /// <param name="a">Start point.</param>
        /// <param name="b">Control point.</param>
        /// <param name="c">End point.</param>
        public Bezier2(V3D a, V3D b, V3D c) { A = a; B = b; C = c; }

        /// <summary>
        /// Interpolated point at t : 0..1 position
        /// </summary>
        /// <param name="t"></param>
        /// <returns></returns>
        public V3D P(double t) => (1.0 - t) * (1.0 - t) * A + 2.0 * t * (1.0 - t) * B + t * t * C;

        /// <summary>
        /// Gets the calculated length.
        /// </summary>
        /// <remarks>
        /// Integral calculation by Dave Eberly, slightly modified for the edge case with colinear control point.
        /// See: http://www.gamedev.net/topic/551455-length-of-a-generalized-quadratic-bezier-curve-in-3d/
        /// </remarks>
        public double Length {
            get {
                if (A == C) {
                    if (A == B) return 0.0;
                    return (A - B).Length;
                }
                if (B == A || B == C) return (A - C).Length;
                V3D A0 = B - A;
                V3D A1 = A - 2.0 * B + C;
                if (!A1.Zero) {
                    double c = 4.0 * A1.Dot(A1);
                    double b = 8.0 * A0.Dot(A1);
                    double a = 4.0 * A0.Dot(A0);
                    double q = 4.0 * a * c - b * b;
                    double twoCpB = 2.0 * c + b;
                    double sumCBA = c + b + a;
                    var l0 = (0.25 / c) * (twoCpB * Math.Sqrt(sumCBA) - b * Math.Sqrt(a));
                    if (q == 0.0) return l0;
                    var l1 = (q / (8.0 * Math.Pow(c, 1.5))) * (Math.Log(2.0 * Math.Sqrt(c * sumCBA) + twoCpB) - Math.Log(2.0 * Math.Sqrt(c * a) + b));
                    return l0 + l1;
                }
                else return 2.0 * A0.Length;
            }
        }

        /// <summary>
        /// Gets the old slow and inefficient line interpolated length.
        /// </summary>
        public double InterpolatedLength {
            get {
                if (A == C) {
                    if (A == B) return 0;
                    return (A - B).Length;
                }
                if (B == A || B == C) return (A - C).Length;
                double dt = InterpolationPrecision / (C - A).Length, length = 0.0;
                for (double t = dt; t < 1.0; t += dt) length += (P(t - dt) - P(t)).Length;
                return length;
            }
        }

    }

    /// <summary>
    /// Cubic Bézier curve calculation class.
    /// </summary>
    class Bezier3 {

        public static double InterpolationPrecision = 0.001;
        public static double LineInterpolationPrecision = 0.05;

        #region Optimization constants

        protected static double Sqrt3 = Math.Sqrt(3d);
        protected static double Div18Sqrt3 = 18d / Sqrt3;
        protected static double OneThird = 1d / 3d;
        protected static double Sqrt3Div36 = Sqrt3 / 36d;

        #endregion

        /// <summary>
        /// Start point.
        /// </summary>
        public V3D A;
        /// <summary>
        /// Control point 1.
        /// </summary>
        public V3D B;
        /// <summary>
        /// Control point 2.
        /// </summary>
        public V3D C;
        /// <summary>
        /// End point.
        /// </summary>
        public V3D D;

        /// <summary>
        /// Creates a cubic Bézier curve.
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <param name="c"></param>
        /// <param name="d"></param>
        public Bezier3(V3D a, V3D b, V3D c, V3D d) { A = a; B = b; C = c; D = d; }

        /// <summary>
        /// Interpolated point at t : 0..1 position.
        /// </summary>
        /// <param name="t"></param>
        /// <returns></returns>
        public V3D P(double t) => A + 3.0 * t * (B - A) + 3.0 * t * t * (C - 2.0 * B + A) + t * t * t * (D - 3.0 * C + 3.0 * B - A);
        
        /// <summary>
        /// Gets the control point for the mid-point quadratic approximation.
        /// </summary>
        private V3D Q => (3.0 * C - D + 3.0 * B - A) / 4.0;

        /// <summary>
        /// Gets the mid-point quadratic approximation.
        /// </summary>
        public Bezier2 M => new Bezier2(A, Q, D);

        /// <summary>
        /// Splits the curve at given position (t : 0..1).
        /// </summary>
        /// <param name="t">A number from 0 to 1.</param>
        /// <returns>Two curves.</returns>
        /// <remarks>
        /// (De Casteljau's algorithm, see: http://caffeineowl.com/graphics/2d/vectorial/bezierintro.html)
        /// </remarks>
        public Bezier3[] SplitAt(double t) {
            V3D a = V3D.Interpolate(A, B, t);
            V3D b = V3D.Interpolate(B, C, t);
            V3D c = V3D.Interpolate(C, D, t);
            V3D m = V3D.Interpolate(a, b, t);
            V3D n = V3D.Interpolate(b, c, t);
            V3D p = P(t);
            return new[] { new Bezier3(A, a, m, p), new Bezier3(p, n, c, D) };
        }

        /// <summary>
        /// Gets the distance between 0 and 1 quadratic aproximations.
        /// </summary>
        private double D01 => (D - 3.0 * C + 3.0 * B - A).Length / 2.0;

        /// <summary>
        /// Gets the split point for adaptive quadratic approximation.
        /// </summary>
        private double Tmax => Math.Pow(Div18Sqrt3 * InterpolationPrecision / D01, OneThird);
        
        /// <summary>
        /// Gets the length of the curve obtained via line interpolation.
        /// </summary>
        public double InterpolatedLength {
            get {
                double dt = LineInterpolationPrecision / (D - A).Length, length = 0.0;
                for (double t = dt; t < 1.0; t += dt) length += (P(t - dt) - P(t)).Length;
                return length;
            }
        }

        /// <summary>
        /// Gets the calculated length of the mid-point quadratic approximation
        /// </summary>
        public double QLength => M.Length;

        /// <summary>
        /// Gets the calculated length of adaptive quadratic approximation.
        /// </summary>
        public double Length {
            get {
                double tmax = 0.0;
                Bezier3 segment = this;
                List<Bezier3> segments = new List<Bezier3>();
                while ((tmax = segment.Tmax) < 1.0) {
                    var split = segment.SplitAt(tmax);
                    segments.Add(split[0]);
                    segment = split[1];
                }
                segments.Add(segment);
                return segments.Sum(s => s.QLength);
            }
        }

    }

    /// <summary>
    /// Quick demo program.
    /// </summary>
    class Program {

        /// <summary>
        /// Iterates given function for specified period of time and returns iterations number
        /// </summary>
        /// <param name="a">Action to perform.</param>
        /// <param name="t">Time to iterate.</param>
        /// <param name="name">Name of the test.</param>
        /// <returns>Number of iterations made.</returns>
        static int Benchmark(Action a, TimeSpan t, string name) {
            Console.WriteLine(String.Format("Testing {0}...", name));
            DateTime start = DateTime.Now;
            int i = 0;
            while (DateTime.Now - start < t) { a(); i++; }
            return (int)(i / t.TotalSeconds);
        }

        /// <summary>
        /// Performs a quick benchmark on some sample data.
        /// </summary>
        static void QuickBench() {
            var test = new V3D[] {
                new V3D(-21298.4, 0.2, 2627.51),
                new V3D(-11.3359, 0.0, 0.0),
                new V3D(11.2637, 0.0, -1.28198),
                new V3D(-21332.3, 0.2, 2629.43)
            };
            var testCurve = new Bezier3(test[0], test[0] + test[1], test[2] + test[3], test[3]);
            double l0 = 0, l1 = 0, l2 = 0;
            int s0 = 0, s1 = 0, s2 = 0;
            TimeSpan t = new TimeSpan(0, 0, 3);
            s0 = Benchmark(() => { l0 = testCurve.InterpolatedLength; }, t, "line interpolation");
            s1 = Benchmark(() => { l1 = testCurve.Length; }, t, "adaptive quadratic interpolation");
            s2 = Benchmark(() => { l2 = testCurve.QLength; }, t, "midpoint quadratic interpolation");
            Console.WriteLine(String.Format(
                "\r\n\t\tLine int.:\t| Adaptive:\t| Midpoint:\r\n" +
                "  Result[m]:\t{0}\t| {1}\t| {2}\r\n" +
                "Speed[op/s]:\t{3}\t\t| {4}\t| {5}",
                Math.Round(l0, 9), Math.Round(l1, 9), Math.Round(l2, 9),
                s0, s1, s2
            ));
            Console.ReadKey(true);
        }

        /// <summary>
        /// Some edge cases with colinear control point test.
        /// </summary>
        /// <exception cref="InvalidOperationException">Thrown when difference between calculated and interpolated lengts are greater than 1%.</exception>
        static void ColinearEdgeCaseTest() {
            var curves = new Bezier2[] {
                new Bezier2(new V3D(0, 0, 0), new V3D(2, 0, 0), new V3D(1, 0, 0)),
                new Bezier2(new V3D(0, 0, 0), new V3D(0, 2, 0), new V3D(0, 1, 0)),
                new Bezier2(new V3D(0, 0, 0), new V3D(0, 0, 2), new V3D(0, 0, 1)),
                new Bezier2(new V3D(0, 0, 0), new V3D(2, 2, 2), new V3D(1, 1, 1)),
                new Bezier2(new V3D(0, 0, 0), new V3D(-1, -1, -1), new V3D(1, 1, 1)),
            };
            foreach (var curve in curves) {
                var error = Math.Abs(curve.Length - curve.InterpolatedLength);
                if (error > 0.01) throw new InvalidOperationException();
            }
        }

        /// <summary>
        /// Tests the code and returns a quick benchmark
        /// </summary>
        /// <param name="args"></param>
        static void Main(string[] args) {
            ColinearEdgeCaseTest();
            QuickBench();
        }

    }

}
