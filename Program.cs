using System;
using System.Collections.Generic;
using System.Linq;

namespace FastBezier {

    /// <summary>
    /// 3D vector class
    /// </summary>
    class V3D {
        public double X;
        public double Y;
        public double Z;
        public double Length { get { return Math.Sqrt(X * X + Y * Y + Z * Z); } }
        public bool Zero { get { return X == 0 && Y == 0; } }
        public V3D(double x, double y, double z) { X = x; Y = y; Z = z; }
        public static V3D operator +(V3D a, V3D b) { return new V3D(a.X + b.X, a.Y + b.Y, a.Z + b.Z); }
        public static V3D operator -(V3D a, V3D b) { return new V3D(a.X - b.X, a.Y - b.Y, a.Z - b.Z); }
        public static V3D operator -(V3D a) { return new V3D(-a.X, -a.Y, -a.Z); }
        public static V3D operator +(V3D a) { return new V3D(+a.X, +a.Y, +a.Z); }
        public static V3D operator *(V3D a, double k) { return new V3D(k * a.X, k * a.Y, k * a.Z); }
        public static V3D operator *(double k, V3D a) { return new V3D(k * a.X, k * a.Y, k * a.Z); }
        public static V3D operator /(V3D a, double k) { return new V3D(a.X / k, a.Y / k, a.Z / k); }
        public static V3D operator /(double k, V3D a) { return new V3D(k / a.X, k / a.Y, k / a.Z); }
        public double Dot(V3D a) { return X * a.X + Y * a.Y + Z * a.Z; }
        public V3D Cross(V3D a) { return new V3D(Y * a.Z - Z * a.Y, Z * a.X - X * a.Z, X * a.Y - Y * a.X); }
        public static V3D Interpolate(V3D a, V3D b, double t) {
            return new V3D(
                a.X * (1 - t) + b.X * t,
                a.Y * (1 - t) + b.Y * t,
                a.Z * (1 - t) + b.Z * t);
        }
    }

    /// <summary>
    /// Quadratic Bézier curve calculation class
    /// </summary>
    class Bezier2 {

        protected const double InterpolationPrecision = 0.001;

        /// <summary>
        /// Start point
        /// </summary>
        public V3D A;
        /// <summary>
        /// Control point
        /// </summary>
        public V3D B;
        /// <summary>
        /// End point
        /// </summary>
        public V3D C;

        public Bezier2(V3D a, V3D b, V3D c) { A = a; B = b; C = c; }

        /// <summary>
        /// Interpolated point at t : 0..1 position
        /// </summary>
        /// <param name="t"></param>
        /// <returns></returns>
        public V3D P(double t) { return (1 - t) * (1 - t) * A + 2 * t * (1 - t) * B + t * t * C; }

        /// <summary>
        /// Integral calculation by Dave Eberly
        /// See: http://www.gamedev.net/topic/551455-length-of-a-generalized-quadratic-bezier-curve-in-3d/
        /// </summary>
        public double Length {
            get {
                V3D A0 = B - A;
                V3D A1 = A - 2 * B + C;
                if (!A1.Zero) {
                    double c = 4 * A1.Dot(A1);
                    double b = 8 * A0.Dot(A1);
                    double a = 4 * A0.Dot(A0);
                    double q = 4 * a * c - b * b;
                    double twoCpB = 2 * c + b;
                    double sumCBA = c + b + a;
                    double mult0 = 0.25 / c;
                    double mult1 = q / (8 * Math.Pow(c, 1.5));
                    return
                        mult0 * (twoCpB * Math.Sqrt(sumCBA) - b * Math.Sqrt(a)) +
                        mult1 * (Math.Log(2 * Math.Sqrt(c * sumCBA) + twoCpB) - Math.Log(2 * Math.Sqrt(c * a) + b));
                }
                else return 2 * A0.Length;
            }
        }

        /// <summary>
        /// Old slow and inefficient line interpolated length
        /// </summary>
        public double InterpolatedLength {
            get {
                double dt = InterpolationPrecision / (C - A).Length, length = 0;
                for (double t = dt; t < 1; t += dt) length += (P(t - dt) - P(t)).Length;
                return length;
            }
        }

    }

    /// <summary>
    /// Cubic Bézier curve calculation class
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
        /// Start point
        /// </summary>
        public V3D A;
        /// <summary>
        /// Control point 1
        /// </summary>
        public V3D B;
        /// <summary>
        /// Control point 2
        /// </summary>
        public V3D C;
        /// <summary>
        /// End point
        /// </summary>
        public V3D D;
        
        public Bezier3(V3D a, V3D b, V3D c, V3D d) { A = a; B = b; C = c; D = d; }

        /// <summary>
        /// Interpolated point at t : 0..1 position
        /// </summary>
        /// <param name="t"></param>
        /// <returns></returns>
        public V3D P(double t) { return A + 3 * t * (B - A) + 3 * t * t * (C - 2 * B + A) + t * t * t * (D - 3 * C + 3 * B - A); }

        /// <summary>
        /// Control point for mid-point quadratic approximation
        /// </summary>
        private V3D Q { get { return (3 * C - D + 3 * B - A) / 4d; } }

        /// <summary>
        /// Mid-point quadratic approximation
        /// </summary>
        public Bezier2 M { get { return new Bezier2(A, Q, D); } }

        /// <summary>
        /// Splits the curve at given position (t : 0..1)
        /// (De Casteljau's algorithm, see: http://caffeineowl.com/graphics/2d/vectorial/bezierintro.html)
        /// </summary>
        /// <param name="t"></param>
        /// <returns></returns>
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
        /// The distance between 0 and 1 quadratic aproximations
        /// </summary>
        private double D01 { get { return (D - 3 * C + 3 * B - A).Length / 2d; } }

        /// <summary>
        /// Split point for adaptive quadratic approximation
        /// </summary>
        private double Tmax { get { return Math.Pow(Div18Sqrt3 * InterpolationPrecision / D01, OneThird);  } }

        /// <summary>
        /// Length of the curve obtained via line interpolation
        /// </summary>
        public double InterpolatedLength {
            get {
                double dt = LineInterpolationPrecision / (D - A).Length, length = 0;
                for (double t = dt; t < 1; t += dt) length += (P(t - dt) - P(t)).Length;
                return length;
            }
        }

        /// <summary>
        /// Calculated length of the mid-point quadratic approximation
        /// </summary>
        public double QLength { get { return M.Length; } }

        /// <summary>
        /// Calculated length of adaptive quadratic approximation
        /// </summary>
        public double Length {
            get {
                double tmax = 0;
                Bezier3 segment = this;
                List<Bezier3> segments = new List<Bezier3>();
                while ((tmax = segment.Tmax) < 1) {
                    var split = segment.SplitAt(tmax);
                    segments.Add(split[0]);
                    segment = split[1];
                }
                segments.Add(segment);
                return segments.Sum(s => s.QLength);
            }
        }

    }

    class Program {

        /// <summary>
        /// Iterates given function for specified period of time and returns iterations number
        /// </summary>
        /// <param name="a"></param>
        /// <param name="t"></param>
        /// <param name="name"></param>
        /// <returns></returns>
        static int Benchmark(Action a, TimeSpan t, string name) {
            Console.WriteLine(String.Format("Testing {0}...", name));
            DateTime start = DateTime.Now;
            int i = 0;
            while (DateTime.Now - start < t) { a(); i++; }
            return (int)(i / t.TotalSeconds);
        }

        /// <summary>
        /// Tests the code and returns a quick benchmark
        /// </summary>
        /// <param name="args"></param>
        static void Main(string[] args) {
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

    }

}
