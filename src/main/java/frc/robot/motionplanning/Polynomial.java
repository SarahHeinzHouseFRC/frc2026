package frc.robot.motionplanning;

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
            newPowers[i] = x.powers[i];
        }
        for (int i=0; i<y.powers.length; ++i) {
            newPowers[x.powers.length+i] = y.powers[i];
        }

        for (int i=0; i<x.coefficients.length; ++i) {
            newCoefficients[i] = x.coefficients[i];
        }
        for (int i=0; i<y.coefficients.length; ++i) {
            newCoefficients[x.coefficients.length+i] = y.coefficients[i];
        }

        return new Polynomial(newPowers, newCoefficients);
    }

    public static Polynomial add(Polynomial x, double c) {
        var newPowers = new int[x.powers.length + 1];
        var newCoefficients = new double[x.coefficients.length + 1];

        newPowers[newPowers.length-1] = 0;
        newCoefficients[newCoefficients.length-1] = c;

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

    public static Polynomial square(Polynomial x) {
        var newPowers = new int[x.powers.length*2];
        var newCoefficients = new double[x.coefficients.length*2];
        for (int i=0; i<x.powers.length; ++i) {
            for (int j=0; j<x.powers.length; ++j) {
                newPowers[i*x.powers.length+j] = x.powers[i]*x.powers[j];
                newCoefficients[i*x.powers.length+j] = x.coefficients[i]*x.coefficients[j];
            }
        }
        return new Polynomial(newPowers, newCoefficients);
    }

    public double solveMinRoot(double t) {
        double t_ = t;
        Polynomial ddx = this.derivative();
        for (int i=0; i<10; ++i) {
            t_ = t_-this.at(t_)/ddx.at(t_);
        }
        return t_;
    }
}
