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

public class Climb extends SubsystemBase {
  /** List of voltage setpoints for the climb in volts */
  public enum ClimbVoltageGoal {
    kGrab(() -> 8.0),
    kRelease(() -> -4.0),
    /** Custom setpoint that can be modified over network tables; Useful for debugging */
    custom(new LoggedTunableNumber("Climb/customVoltageGoal", 0.0));

    private DoubleSupplier goalVoltage;

    ClimbVoltageGoal(DoubleSupplier goalVoltage) {
      this.goalVoltage = goalVoltage;
    }

    public double getGoalVoltage() {
      return this.goalVoltage.getAsDouble();
    }
  }

  @AutoLogOutput(key = "Climb/Goal")
  private ClimbVoltageGoal voltageGoal = null;

  private final ClimbIO[] kHardware;
  private final ClimbIOInputsAutoLogged[] kInputs;

  /** Creates a new Climb. Note that the first argument is considered the lead motor */
  public Climb(ClimbIO... io) {
    kHardware = new ClimbIO[io.length];
    kInputs = new ClimbIOInputsAutoLogged[io.length];

    for (int i = 0; i < kHardware.length; i++) {
      kHardware[i] = io[i];
      kInputs[i] = new ClimbIOInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < kHardware.length; i++) {
      kHardware[i].updateInputs(kInputs[i]);
      Logger.processInputs("Climb/Inputs" + i, kInputs[i]);
    }

    if (!DriverStation.isEnabled()) {
      stop();
    }

    if (voltageGoal != null) {
      setVoltage(voltageGoal.getGoalVoltage());
    }

    // Add position checking here 
  }

  /**
   * Sets the voltage of the motor. Assumes that positive voltage moves the motor closer to the
   * maximum possible physical position the mechanism can be in, while negative voltage moves 
   * the motor closer to the minimum possible physical position the mechanism can be in
   * 
   * @param voltage
   */
  public void setVoltage(double voltage) {
    // Notice how we are not checking if position control is running, it is up to the caller
    // to check for this before calling this method (I recommend calling this subsystem's stop() 
    // method after completing any action)
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
    // Only need to return one position since the motors are mechaically linked
    return kInputs[0].position;
  }
}
