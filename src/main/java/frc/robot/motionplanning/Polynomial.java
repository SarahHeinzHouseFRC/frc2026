package frc.robot.motionplanning;

import java.util.ArrayList;
import java.util.List;

public class Polynomial {
    public final int[] powers;
    public final double[] coefficients;

    public Polynomial(int[] powers, double[] coefficients) {
        this.powers = powers;
        this.coefficients = coefficients;
    }

    /**
     * Evaluates the polynomial with input t
     * @param t Input to the polynomial function
     * @return Value of the polynomial function at t
     */
    public double at(double t) {
        double answer = 0d;
        for (int i = 0; i < this.powers.length; ++i) {
            answer += this.coefficients[i] * Math.pow(t, this.powers[i]);
        }
        return answer;
    }

    /**
     * // TODO: add comment
     * @return
     */
    public Polynomial derivative() {
        var newPowers = new int[this.powers.length];
        var newCoefficients = new double[this.coefficients.length];

        for (int i = 0; i < this.powers.length; ++i) {
            newCoefficients[i] = this.powers[i] * this.coefficients[i];
            newPowers[i] = this.powers[i] - 1;
        }

        return new Polynomial(newPowers, newCoefficients);
    }

    public static Polynomial add(Polynomial x, Polynomial y) {
        var newPowers = new int[x.powers.length + y.powers.length];
        var newCoefficients = new double[x.coefficients.length + y.coefficients.length];

        for (int i=0; i<x.powers.length; ++i) {
//            newPowers[]
        }

        return new Polynomial(newPowers, newCoefficients);
    }

    public static Polynomial negate(Polynomial x) {
        for (int i = 0; i < x.coefficients.length; ++i) {
            x.coefficients[i] *= -1;
        }
        return x;
    }

    public Polynomial copy() {
        return new Polynomial(this.powers, this.coefficients);
    }

    public Polynomial buildDistancePolynomial(
            Polynomial x,
            Polynomial y,
            double t1,
            double d
    ) {
        int maxDeg = 6;
        Polynomial result = new Polynomial(new int[maxDeg + 1], new double[maxDeg + 1]);

        for (int i = 0; i <= maxDeg; i++) {
            result.powers[i] = i;
        }

        double x1 = x.at(t1);
        double y1 = y.at(t1);

        // (x(t) - x1)^2
        for (int i = 0; i < x.coefficients.length; i++) {
            for (int j = 0; j < x.coefficients.length; j++) {
                int deg = x.powers[i] + x.powers[j];
                if (deg <= maxDeg) {
                    result.coefficients[deg] +=
                            x.coefficients[i] * x.coefficients[j];
                }
            }

            int deg = x.powers[i];
            if (deg <= maxDeg) {
                result.coefficients[deg] -=
                        2 * x1 * x.coefficients[i];
            }
        }
        result.coefficients[0] += x1 * x1;

        // (y(t) - y1)^2
        for (int i = 0; i < y.coefficients.length; i++) {
            for (int j = 0; j < y.coefficients.length; j++) {
                int deg = y.powers[i] + y.powers[j];
                if (deg <= maxDeg) {
                    result.coefficients[deg] +=
                            y.coefficients[i] * y.coefficients[j];
                }
            }

            int deg = y.powers[i];
            if (deg <= maxDeg) {
                result.coefficients[deg] -=
                        2 * y1 * y.coefficients[i];
            }
        }
        result.coefficients[0] += y1 * y1;

        // subtract d^2
        result.coefficients[0] -= d * d;

        return result;
    }

    public double solveMinRoot(double t) {
        double t_ = 0.1;
        Polynomial ddx = this.derivative();
        for (int i=0; i<10; ++i) {
            t_ = t_-this.at(t_)/ddx.at(t_);
        }
        return t;
    }


}
