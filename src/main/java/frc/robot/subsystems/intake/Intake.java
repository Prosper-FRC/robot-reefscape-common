
package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.debugging.LoggedTunableNumber;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;

public class Intake extends SubsystemBase {
  /** List of voltage setpoints for the intake in voltage */
  public enum IntakeGoal {
    kIntake(() -> 4.0),
    kOuttake(() -> -4.0),
    /** Custom setpoint that can be modified over network tables; Useful for debugging */
    custom(new LoggedTunableNumber("Intake/Feedback/Setpoint", 0.0));

    private DoubleSupplier intakeVolts;

    IntakeGoal(DoubleSupplier intakeVolts) {
      this.intakeVolts = intakeVolts;
    }

    public double getGoalVolts() {
      return this.intakeVolts.getAsDouble();
    }
  }

  private final IntakeIO kHardware;
  private final IntakeIOInputsAutoLogged kInputs = new IntakeIOInputsAutoLogged();

  private final SensorIO kSensor;
  private final SensorIOInputsAutoLogged kSensorInputs = new SensorIOInputsAutoLogged();

  private final PivotIO kPivotHardware;
  private final PivotIOInputsAutoLogged kivotInputs = new PivotIOInputsAutoLogged();

  private boolean detectedGamepiece = false;
  private LinearFilter ampFilter = LinearFilter.movingAverage(
    IntakeConstants.kLinearFilterSampleCount);

  @AutoLogOutput(key = "Intake/Goal")
  private IntakeGoal goal = null;

  public Intake(IntakeIO hardwareIO, SensorIO sensorIO, PivotIO pivotHardwareIO) {
    kHardware = hardwareIO;
    kSensor = sensorIO;
    kPivotHardware = pivotHardwareIO;
  }

  @Override
  public void periodic() {
    kHardware.updateInputs(kInputs);
    Logger.processInputs("Intake/Inputs", kInputs);
    kSensor.updateInputs(kSensorInputs);
    Logger.processInputs("Intake/Inputs/Sensor", kSensorInputs);

    // Stop and clear goal if disabled. Used if copilot is still pressing button to command
    // intake when the disabled key is pressed
    if (DriverStation.isDisabled()) {
      stop();
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

    Logger.recordOutput("Intake/Goal", goal);

    if (goal != null) {
      setVoltage(goal.getGoalVolts());
    }
  }

  /**
   * Sets the voltage goal of the mechanism, logic runs in subsystem periodic method
   * 
   * @param desiredGoal The desired voltage goal
   */
  public void setGoal(IntakeGoal desiredGoal) {
    goal = desiredGoal;
  }

  /** Stops the motor */
  public void stop() {
    goal = null;
    kHardware.stop();
  }

  /**
   * Sets the voltage of the motor,
   * 
   * @param voltage
   */
  public void setVoltage(double voltage) {
    kHardware.setVoltage(voltage);
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