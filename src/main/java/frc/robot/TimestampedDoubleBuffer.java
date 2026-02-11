package frc.robot;

public class TimestampedDoubleBuffer {
  private final double[] buffer;
  private final double[] timestamps;
  private int index = 0;
  private final int size;

  public TimestampedDoubleBuffer(int size) {
    this.size = size;
    buffer = new double[size];
    timestamps = new double[size];
  }

  public void add(double value, double timestamp) {
    buffer[index] = value;
    timestamps[index] = timestamp;
    index = (index + 1) % buffer.length;
  }

  public double getValueAtTimestamp(double timestamp) {
    // Search backwards from most recent

    if (timestamps[(index - 1 + size) % size] <= timestamp) return buffer[(index - 1 + size) % size];
    for (int i = 2; i < size; i++) {
      int prevIndex = (index - i + size) % size;
      int nextIndex = (index - i + 1 + size) % size;

      if (timestamps[prevIndex] <= timestamp && timestamps[nextIndex] > timestamp) {
        double timestampRange = timestamps[nextIndex] - timestamps[prevIndex];
        if (timestampRange == 0) return buffer[prevIndex];

        double t = (timestamp - timestamps[prevIndex]) / timestampRange;
        return buffer[prevIndex] * (1 - t) + buffer[nextIndex] * t;
      }
    }

    System.out.println("[WARNING] timestamp not found in buffer range");
    return 0;
  }
}
