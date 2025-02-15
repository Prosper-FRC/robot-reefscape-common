// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.debugging.LoggedTunableNumber;
import frc.robot.utils.visualizers.PivotVisualizer;

public class Climb extends SubsystemBase {
  /** List of voltage setpoints for the climb in volts */
  public enum ClimbVoltageGoal {
    kGrab(() -> 8.0),
    kRelease(() -> -4.0),
    /** Custom setpoint that can be modified over network tables; 
     * Useful for debugging */
    custom(new LoggedTunableNumber("Climb/customVoltageGoal", 0.0));

    private DoubleSupplier goalVoltage;

    ClimbVoltageGoal(DoubleSupplier goalVoltage) {
      this.goalVoltage = goalVoltage;
    }

    public double getGoalVoltage() {
      return this.goalVoltage.getAsDouble();
    }
  }

  private ClimbVoltageGoal voltageGoal = null;

  private final ClimbIO[] kHardware;
  private final ClimbIOInputsAutoLogged[] kInputs;

  private final DutyCycleEncoderIO kAbsoluteEncoder;
  private final DutyCycleEncoderIOInputsAutoLogged kAbsoluteEncoderInputs = new DutyCycleEncoderIOInputsAutoLogged();

  private boolean isAbsoluteEncoderConnected = false;

  private final PivotVisualizer kClimbVisualizer;

  /** 
   * Creates a new Climb. Note that the first argument is considered 
   * the lead motor 
   */
  public Climb(DutyCycleEncoderIO encoderIO, ClimbIO... io) {
    kHardware = new ClimbIO[io.length];
    kInputs = new ClimbIOInputsAutoLogged[io.length];

    for (int i = 0; i < kHardware.length; i++) {
      kHardware[i] = io[i];
      kInputs[i] = new ClimbIOInputsAutoLogged();
    }

    kAbsoluteEncoder = encoderIO;

    kClimbVisualizer = new PivotVisualizer(ClimbConstants.kPivotVisualizerConfiguration);
  }

  @Override
  public void periodic() {
    for (int i = 0; i < kHardware.length; i++) {
      kHardware[i].updateInputs(kInputs[i]);
      Logger.processInputs("Climb/Inputs" + i, kInputs[i]);
    }

    kAbsoluteEncoder.updateInputs(kAbsoluteEncoderInputs);
    Logger.processInputs("Climb/AbsoluteEncoder/Inputs", kAbsoluteEncoderInputs);

    isAbsoluteEncoderConnected = kAbsoluteEncoderInputs.isConnected;

    if (!DriverStation.isEnabled()) {
      stop();
    }

    if (voltageGoal != null) {
      setVoltage(voltageGoal.getGoalVoltage());
      Logger.recordOutput("Climb/VoltageGoal", voltageGoal);
    } else {
      Logger.recordOutput("Climb/VoltageGoal", "NONE");
    }

    // Continuously check if climb has moved beyond its limitations, note
    // that we only need to compare the left voltage as that is the lead
    // motor
    if (getPosition().getDegrees() > ClimbConstants.kMaxPosition.getDegrees() 
        && kInputs[0].appliedVoltage > 0.0) {
      stop();
    } else if (getPosition().getDegrees() < ClimbConstants.kMinPosition.getDegrees() 
        && kInputs[0].appliedVoltage < 0.0) {
      stop();
    } else {
      // Do nothing if limits are not reached
    }

    // The visualizer needs to be periodically fed the current position of the mechanism
    kClimbVisualizer.updatePosition(getPosition());
  }

  /**
   * Sets the position goal of the mechanism, logic runs in subsystem periodic method
   * 
   * @param desiredGoal The desired position goal
   */
  public void setGoalVoltage(ClimbVoltageGoal desiredGoal) {
    voltageGoal = desiredGoal;
  }

  /**
   * Sets the voltage of the motor. Assumes that positive voltage moves the 
   * motor closer to the maximum possible physical position the mechanism can 
   * be in, while negative voltage moves the motor closer to the minimum 
   * possible physical position the mechanism can be in
   * 
   * @param voltage
   */
  public void setVoltage(double voltage) {
    // Notice how we are not checking if position control is running, it is up 
    // to the caller to check for this before calling this method (I recommend 
    // calling this subsystem's stop() method after completing any action)
    if (getPosition().getDegrees() > ClimbConstants.kMaxPosition.getDegrees() 
        && voltage > 0) {
      return;
    } else if (getPosition().getDegrees() < ClimbConstants.kMinPosition.getDegrees()
        && voltage < 0) {
      return;
    } else {
      for (ClimbIO io : kHardware) {
        io.setVoltage(voltage);
      }
    }
  }

  /** Stops the mechanism */
  public void stop() {
    voltageGoal = null;

    for (ClimbIO io : kHardware) {
      io.stop();
    }
  }

  /** Resets all motor encoders to 0 */
  public void resetPosition() {
    for (ClimbIO io : kHardware) {
      io.resetPosition();
    }
  }

  /**
   * Gets the position of the mechanism. Note that this assumes the caller only
   * desires the position of the mechanism as a whole, not the position of the 
   * individual motor
   * 
   * @return The position of the mechanism
   */
  public Rotation2d getPosition() {
    if (isAbsoluteEncoderConnected) {
      return Rotation2d.fromRotations(
        kAbsoluteEncoderInputs.dutyCycleReading * ClimbConstants.kGearing);
    } else {
      // Only need to return one position since the motors are mechaically linked
      return kInputs[0].position;
    }
  }
}
