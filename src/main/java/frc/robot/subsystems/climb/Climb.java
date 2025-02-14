// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

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

  /** Creates a new Climb. */
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
      // stop
    }

    if (voltageGoal != null) {
      // set voltage
    }

    // Add position checking here 
  }
}
