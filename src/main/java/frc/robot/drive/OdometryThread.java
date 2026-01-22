// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.drive;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version includes an overload for Spark signals, which checks for errors to ensure that
 * all measurements in the sample are valid.
 */
public class OdometryThread {
    private final List<DoubleSupplier> genericSignals = new ArrayList<>();
    private final List<Queue<Double>> genericQueues = new ArrayList<>();
    private final List<BooleanSupplier> signalValidators = new ArrayList<>();
    private final List<Queue<Double>> timestampQueues = new ArrayList<>();

    private static OdometryThread instance = null;
    private final Notifier notifier = new Notifier(this::run);

    public static OdometryThread getInstance() {
        if (instance == null) {
            instance = new OdometryThread();
        }
        return instance;
    }

    private OdometryThread() {
        notifier.setName("OdometryThread");
    }

    public void start() {
        if (!timestampQueues.isEmpty()) {
            notifier.startPeriodic(1.0 / DriveConstants.odometryFrequency);
        }
    }

    /** Registers a Spark signal to be read from the thread. */
    public Queue<Double> registerSignal(SparkBase spark, DoubleSupplier signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        Drive.odometryLock.lock();
        try {
            genericSignals.add(signal);
            genericQueues.add(queue);
            signalValidators.add(() -> spark.getLastError() == REVLibError.kOk);
        } finally {
            Drive.odometryLock.unlock();
        }
        return queue;
    }

    /** Registers a generic signal to be read from the thread. */
    public Queue<Double> registerSignal(DoubleSupplier signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        Drive.odometryLock.lock();
        try {
            genericSignals.add(signal);
            genericQueues.add(queue);
            signalValidators.add(null);
        } finally {
            Drive.odometryLock.unlock();
        }
        return queue;
    }

    /** Registers a generic signal with a validator to be read from the thread. */
    public Queue<Double> registerSignal(DoubleSupplier signal, BooleanSupplier validator) {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        Drive.odometryLock.lock();
        try {
            genericSignals.add(signal);
            genericQueues.add(queue);
            signalValidators.add(validator);
        } finally {
            Drive.odometryLock.unlock();
        }
        return queue;
    }

    /** Returns a new queue that returns timestamp values for each sample. */
    public Queue<Double> makeTimestampQueue() {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        Drive.odometryLock.lock();
        try {
            timestampQueues.add(queue);
        } finally {
            Drive.odometryLock.unlock();
        }
        return queue;
    }

    private void run() {
        // Save new data to queues
        Drive.odometryLock.lock();
        try {
            // Get sample timestamp
            double timestamp = Timer.getFPGATimestamp();

            // Read Spark values, mark invalid in case of error
            double[] values = new double[genericSignals.size()];
            boolean isValid = true;
            for (int i = 0; i < genericSignals.size(); i++) {
                values[i] = genericSignals.get(i).getAsDouble();
                BooleanSupplier validator = signalValidators.get(i);
                if (validator != null && !validator.getAsBoolean()) {
                    isValid = false;
                }
            }

            // If valid, add values to queues
            if (isValid) {
                for (int i = 0; i < genericSignals.size(); i++) {
                    genericQueues.get(i).offer(values[i]);
                }
                for (int i = 0; i < timestampQueues.size(); i++) {
                    timestampQueues.get(i).offer(timestamp);
                }
            }
        } finally {
            Drive.odometryLock.unlock();
        }
    }
}