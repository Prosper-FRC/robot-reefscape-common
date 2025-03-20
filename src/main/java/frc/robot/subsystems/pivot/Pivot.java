// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.utils.debugging.LoggedTunableNumber;
import frc.robot.utils.visualizers.PivotVisualizer;

public class Pivot extends SubsystemBase {
  /** List of position setpoints for the pivot */
  public enum PivotGoal {
    kStowScore(() -> Rotation2d.fromDegrees(64.0)),
    kStowPickup(() -> Rotation2d.fromDegrees(54.0)),
    kIntakeReef(() -> Rotation2d.fromDegrees(5.0)),
    kIntakeGround(() -> Rotation2d.fromDegrees(-55.5)),
    kProcessorScore(() -> Rotation2d.fromDegrees(-30.0)),
    kScore(() -> Rotation2d.fromDegrees(40.0)),
    kBargeScore(() -> Rotation2d.fromDegrees(46.0)),
    /** Custom setpoint that can be modified over network tables; Useful for debugging */
    custom(() -> Rotation2d.fromDegrees(
      new LoggedTunableNumber("Intake/Feedback/PivotSetpointDegrees", 0.0).get()));

    private Supplier<Rotation2d> goalPosition;

    PivotGoal(Supplier<Rotation2d> goalPosition) {
      this.goalPosition = goalPosition;
    }

    public Rotation2d getGoalPosition() {
      return this.goalPosition.get();
    }
  }

  private final PivotIO kPivotHardware;
  private final PivotIOInputsAutoLogged kPivotInputs = new PivotIOInputsAutoLogged();

  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("Pivot/Gains/kP", PivotConstants.kPivotGains.p());
  private final LoggedTunableNumber kI =
      new LoggedTunableNumber("Pivot/Gains/kI", PivotConstants.kPivotGains.i());
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("Pivot/Gains/kD", PivotConstants.kPivotGains.d());
  private final LoggedTunableNumber kS =
      new LoggedTunableNumber("Pivot/Gains/kS", PivotConstants.kPivotGains.s());
  private final LoggedTunableNumber kV =
      new LoggedTunableNumber("Pivot/Gains/kV", PivotConstants.kPivotGains.v());
  private final LoggedTunableNumber kA =
      new LoggedTunableNumber("Pivot/Gains/kA", PivotConstants.kPivotGains.a());
  private final LoggedTunableNumber kG =
      new LoggedTunableNumber("Pivot/Gains/kG", PivotConstants.kPivotGains.g());
  private final LoggedTunableNumber kMaxVelocity =
      new LoggedTunableNumber(
          "Pivot/MotionMagic/kMaxVelocity", 
          PivotConstants.kPivotGains.maxVelocityRotationsPerSecond());
  private final LoggedTunableNumber kMaxAcceleration =
      new LoggedTunableNumber(
          "Pivot/MotionMagic/kMaxAcceleration", 
          PivotConstants.kPivotGains.maxAccelerationRotationsPerSecondSquared());

  private PivotGoal pivotGoal = null;

  // Object used to visualize the mechanism over network tables, useful in simulation
  private final PivotVisualizer kPivotVisualizer;

  public Pivot(PivotIO pivotIO) {
    kPivotHardware = pivotIO;

    kPivotVisualizer = new PivotVisualizer(
      "Pivot/Visualizer", 
      PivotConstants.kPivotVisualizerConfiguration, 
      4.0, 
      new Color8Bit(Color.kBlue));
  }

  @Override
  public void periodic() {
    kPivotHardware.updateInputs(kPivotInputs);
    Logger.processInputs("Pivot/Inputs", kPivotInputs);

    // Stop and clear goal if disabled. Used if copilot is still pressing button to command
    // intake when the disabled key is pressed
    if (DriverStation.isDisabled()) {
      stop();
    }

    if (pivotGoal != null) {
      setPivotPosition(pivotGoal.getGoalPosition());
      Logger.recordOutput("Pivot/PivotGoalValue", pivotGoal.getGoalPosition());
      Logger.recordOutput("Pivot/PivotGoal", pivotGoal);
    } else {
      Logger.recordOutput("Pivot/PivotGoal", "NONE");
    }

    // Check if pivot is attempting to move beyond its limitations
    if (getPivotPosition().getDegrees() > PivotConstants.kMaxPivotPosition.getDegrees() 
        && kPivotInputs.appliedVoltage > 0.0) {
      stop();
    } else if (getPivotPosition().getDegrees() < PivotConstants.kMinPivotPosition.getDegrees() 
        && kPivotInputs.appliedVoltage < 0.0) {
      stop();
    } else {
      // Do nothing if limits are not reached
    }

    // This says that if the value is changed in the advantageScope tool,
    // Then we change the values in the code. Saves deploy time.
    // More found in prerequisites slide
    LoggedTunableNumber.ifChanged(
      hashCode(),
      () -> {
        kPivotHardware.setGains(
            kP.get(), kI.get(), kD.get(), kS.get(), kG.get(), kV.get(), kA.get());
      },
      kP,
      kI,
      kD,
      kS,
      kV,
      kA,
      kG);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          kPivotHardware.setMotionMagicConstraints(kMaxVelocity.get(), kMaxAcceleration.get());
        },
        kMaxVelocity,
        kMaxAcceleration);

    // The visualizer needs to be periodically fed the current position of the mechanism;
    // Invert cause the pivot is flipped on the robot relative to its coordinate field
    kPivotVisualizer.updatePosition(getPivotPosition().times(-1.0));
  }

  /**
   * Sets the voltage goal of the pivo mechanism, logic runs in subsystem periodic method
   * 
   * @param desiredGoal The desired voltage goal
   */
  public void setPivotGoal(PivotGoal desiredGoal) {
    pivotGoal = desiredGoal;
  }

  /** 
   * Stops the motor and sets the desired goal to null so it does not attempt to go 
   * to a setpoint after method is invoked 
   */
  public void stop() {
    pivotGoal = null;
    kPivotHardware.stop();
  }

  /**
   * Sets the voltage of the pivot motor
   * 
   * @param voltage
   */
  public void setPivotVoltage(double voltage) {
    kPivotHardware.setVoltage(voltage);
  }

  /**
   * Sets the desired angular position of the pivot mechanism
   * 
   * @param position
   */
  public void setPivotPosition(Rotation2d position) {
    kPivotHardware.setPosition(position);
  }

  /**
   * Sets the vertical position of the mechanism on the visuzlier, useful as the
   * pivot moves with the elevator
   * 
   * @param positionMeters
   */
  public void setVisualizerVerticalPosition(double positionMeters) {
    kPivotVisualizer.setRootVerticalPositionMeters(positionMeters);
  }
  
  /**
   * Compute the error based off of our current position and current goal
   * 
   * @return The computed error in degrees
   */
  @AutoLogOutput(key = "Pivot/Feedback/ErrorDegrees")
  public double getPivotErrorDegrees() {
    if (pivotGoal != null && getPivotPosition() != null) {
      return pivotGoal.getGoalPosition().getDegrees() - getPivotPosition().getDegrees();
    } else {
      return 0.0;
    }
  }

  /**
   * @return If the pivot is at its desired goal yet
   */
  @AutoLogOutput(key = "Pivot/Feedback/AtGoal")
  public boolean pivotAtGoal() {
    return Math.abs(getPivotErrorDegrees()) < IntakeConstants.kPivotPositionTolerance.getDegrees();
  }

  /**
   * @return The position of the pivot
   */
  public Rotation2d getPivotPosition() {
    return kPivotInputs.position;
  }
}
