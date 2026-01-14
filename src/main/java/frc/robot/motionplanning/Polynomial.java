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
}
