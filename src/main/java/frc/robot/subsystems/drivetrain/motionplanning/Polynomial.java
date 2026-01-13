package frc.robot.subsystems.drivetrain.motionplanning;

public class Polynomial {
    int[] powers;
    double[] coefficients;

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

    public Polynomial add(Polynomial a) {
        // TODO
    }

    public Polynomial negate() throws CloneNotSupportedException {
        Polynomial newPolynomial = (Polynomial) this.clone();
        // TODO
    }
}
