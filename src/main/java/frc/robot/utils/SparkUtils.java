package frc.robot.utils;

import com.revrobotics.REVLibError;

import java.util.function.Supplier;

public class SparkUtils {
  public static REVLibError tryUntilOk(Supplier<REVLibError> command, int maxAttempts) {
    REVLibError error = REVLibError.kError;
    for (int i = 0; i < maxAttempts; i++) {
      error = command.get();
      if (error == REVLibError.kOk) {
        break;
      }
    }
    return error;
  }

  public static REVLibError tryUntilOk(Supplier<REVLibError> command) {
    return tryUntilOk(command, 5);
  }
}
