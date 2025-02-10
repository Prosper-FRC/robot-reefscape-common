// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

/** The elevator subsystem's hardware interface. */
public interface ClimbIO {
  @AutoLog
  public static class ClimbIOInputs {
    public boolean isMotorConnected = false;

    // Logging the position and velocity in meters since this is a linear mechanism
    public Rotation2d absolutePosistion = Rotation2d.fromRotations(0.0);
    public Rotation2d relativePosistion = Rotation2d.fromRotations(0.0);
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
  }

  public default void updateInputs(ClimbIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void setPosistion(Rotation2d posistion) {}

  public default void resetPosistion() {}

  public default void stop() {}

}