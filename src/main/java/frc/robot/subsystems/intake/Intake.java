
package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.debugging.LoggedTunableNumber;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class Intake extends SubsystemBase {
  /** List of voltage setpoints for the intake in voltage */
  public enum RollerGoal {
    kIntake(() -> 4.0),
    kOuttake(() -> -4.0),
    /** Custom setpoint that can be modified over network tables; Useful for debugging */
    custom(new LoggedTunableNumber("Intake/Feedback/RollerSetpointVolts", 0.0));

    private DoubleSupplier goalVoltage;

    RollerGoal(DoubleSupplier goalVoltage) {
      this.goalVoltage = goalVoltage;
    }

    public double getGoalVolts() {
      return this.goalVoltage.getAsDouble();
    }
  }

  /** List of position setpoints for the pivot */
  public enum PivotGoal {
    kStow(() -> Rotation2d.fromDegrees(0.0)),
    kIntake(() -> Rotation2d.fromDegrees(0.0)),
    kScore(() -> Rotation2d.fromDegrees(0.0)),
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

  private final IntakeIO kRollerHardware;
  private final IntakeIOInputsAutoLogged kInputs = new IntakeIOInputsAutoLogged();

  private final SensorIO kSensor;
  private final SensorIOInputsAutoLogged kSensorInputs = new SensorIOInputsAutoLogged();

  private final PivotIO kPivotHardware;
  private final PivotIOInputsAutoLogged kPivotInputs = new PivotIOInputsAutoLogged();

  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("Intake/Gains/kP", IntakeConstants.kPivotGains.p());
  private final LoggedTunableNumber kI =
      new LoggedTunableNumber("Intake/Gains/kI", IntakeConstants.kPivotGains.i());
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("Intake/Gains/kD", IntakeConstants.kPivotGains.d());
  private final LoggedTunableNumber kS =
      new LoggedTunableNumber("Intake/Gains/kS", IntakeConstants.kPivotGains.s());
  private final LoggedTunableNumber kV =
      new LoggedTunableNumber("Intake/Gains/kV", IntakeConstants.kPivotGains.v());
  private final LoggedTunableNumber kA =
      new LoggedTunableNumber("Intake/Gains/kA", IntakeConstants.kPivotGains.a());
  private final LoggedTunableNumber kG =
      new LoggedTunableNumber("Intake/Gains/kG", IntakeConstants.kPivotGains.g());
  private final LoggedTunableNumber kMaxVelocity =
      new LoggedTunableNumber(
          "Intake/MotionMagic/kMaxVelocity", 
          IntakeConstants.kPivotGains.maxVelocityMetersPerSecond());
  private final LoggedTunableNumber kMaxAcceleration =
      new LoggedTunableNumber(
          "Intake/MotionMagic/kMaxAcceleration", 
          IntakeConstants.kPivotGains.maxAccelerationMetersPerSecondSquared());

  private boolean detectedGamepiece = false;
  private LinearFilter ampFilter = LinearFilter.movingAverage(
    IntakeConstants.kLinearFilterSampleCount);

  @AutoLogOutput(key = "Intake/RollerGoal")
  private RollerGoal rollerGoal = null;
  @AutoLogOutput(key = "Intake/PivotGoal")
  private PivotGoal pivotGoal = null;

  public Intake(IntakeIO hardwareIO, SensorIO sensorIO, PivotIO pivotHardwareIO) {
    kRollerHardware = hardwareIO;
    kSensor = sensorIO;
    kPivotHardware = pivotHardwareIO;
  }

  @Override
  public void periodic() {
    kRollerHardware.updateInputs(kInputs);
    Logger.processInputs("Intake/Inputs/Rollers", kInputs);
    kSensor.updateInputs(kSensorInputs);
    Logger.processInputs("Intake/Inputs/Sensor", kSensorInputs);
    kPivotHardware.updateInputs(kPivotInputs);
    Logger.processInputs("Intake/Inputs/Pivot", kPivotInputs);

    // Stop and clear goal if disabled. Used if copilot is still pressing button to command
    // intake when the disabled key is pressed
    if (DriverStation.isDisabled()) {
      stop(true, true);
    }

    // If the CANrange disconnects we can use motor current to detect when we have a coral
    // TODO Test this fallback to see if it actually works
    if (kSensorInputs.isConnected) {
      detectedGamepiece = kSensorInputs.detectsObject;
    } else {
      // Checks for spike in amperage, and if greater than the value then
      // the intake motor probbaly has the note
      detectedGamepiece = ampFilter.calculate(
        kInputs.statorCurrentAmps) > IntakeConstants.kAmpFilterThreshold;
    }

    Logger.recordOutput("Intake/Goal", rollerGoal);

    if (rollerGoal != null) {
      setRollerVoltage(rollerGoal.getGoalVolts());
    }
    if (pivotGoal != null) {
      kPivotHardware.setPosition(pivotGoal.getGoalPosition());
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
  }

  /**
   * Sets the voltage goal of the roller mechanism, logic runs in subsystem periodic method
   * 
   * @param desiredGoal The desired voltage goal
   */
  public void setRollerGoal(RollerGoal desiredGoal) {
    rollerGoal = desiredGoal;
  }

  /**
   * Sets the voltage goal of the pivo mechanism, logic runs in subsystem periodic method
   * 
   * @param desiredGoal The desired voltage goal
   */
  public void setPivotGoal(PivotGoal desiredGoal) {
    pivotGoal = desiredGoal;
  }

  /** Stops the motor */
  public void stop(boolean stopRollers, boolean stopPivot) {
    if (stopRollers) {
      rollerGoal = null;
      kRollerHardware.stop();
    }
    if (stopPivot) {
      pivotGoal = null;
      kPivotHardware.stop();
    }
  }

  /**
   * Sets the voltage of the roller motor
   * 
   * @param voltage
   */
  public void setRollerVoltage(double voltage) {
    kRollerHardware.setVoltage(voltage);
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
   * The subsystem runs a linear filter that accepts the motor's current and 
   * compares the moving average against a threshold. If that threshold is exceeded, 
   * it is very likely that the motor has a gamepiece (or is jammed)
   * 
   * @return If the motor thinks it has a gamepiece
   */
  @AutoLogOutput(key = "Intake/detectedGamepiece")
  public boolean detectedGamepiece() {
    return detectedGamepiece;
  }
}