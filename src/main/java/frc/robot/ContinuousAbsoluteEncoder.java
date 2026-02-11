package frc.robot;

public class ContinuousAbsoluteEncoder {
  private double conversionFactor = 1;
  private int accumulator = 0;
  private double position = -1;
  private int accumulatorBoundLow = Integer.MIN_VALUE;
  private int accumulatorBoundHigh = Integer.MAX_VALUE;
  private boolean accumulatorBound = false;

  public ContinuousAbsoluteEncoder() {}

  public void setEncoderConversionFactor(double conversionFactor) {
    this.conversionFactor = conversionFactor;
  }

  public int getAccumulator() {
    return accumulator;
  }

  public void setAccumulator(int accumulator) {
    if (accumulatorBound) {
      this.accumulator = Math.max(Math.min(accumulator, accumulatorBoundHigh), accumulatorBoundLow);
    } else {
      this.accumulator = accumulator;
    }
  }

  public void enableAccumulatorBounds(int low, int high) {
    this.accumulatorBoundLow = low;
    this.accumulatorBoundHigh = high;
    this.accumulatorBound = true;
  }

  public void disableAccumulatorBounds() {
    this.accumulatorBound = false;
  }

  public double update(double position) {
    if (this.position >= 0) {
      if (position > .75 * conversionFactor && this.position < .25 * conversionFactor) {
        setAccumulator(accumulator - 1);
      } else if (position < .25 * conversionFactor && this.position > .75 * conversionFactor) {
        setAccumulator(accumulator + 1);
      }
    }
    this.position = position;
    return getPosition();
  }

  public double getPosition() {
    return (((double) accumulator) * conversionFactor) + position;
  }
}
