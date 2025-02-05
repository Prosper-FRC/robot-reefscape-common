// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.debugging.LoggedTunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
    /** List of position setpoints for the pivot */
    public enum PivotGoal {
        // NOTE These setpoints must be updated per robot, "DEMO" is merely an example setpoint.

        DEMO(() -> Rotation2d.fromDegrees(90.0)),
        /** We get the setpoint in degrees from the LoggedTunableNumber */
        CUSTOM(() -> Rotation2d.fromDegrees(new LoggedTunableNumber("Pivot/Custom", 0.0).get()));

        // Suppliers are used to keep in line with this team's programming convention. In some
        // instances, we may need to have a goal that dynamically changes based on a state from another
        // class. If we do not use a Supplier, java will only create an instance of the setpoint once
        // and never check if it is updated again.
        Supplier<Rotation2d> goal;

        PivotGoal(Supplier<Rotation2d> goal) {
            this.goal = goal;
        }

        public Rotation2d getGoal() {
            return this.goal.get();
        }
    }

    // Hardware is where we interact with the motors and sensors directly
    private final ClimbKrakenHardware kPivotHardware;
    // "Inputs" is telemetry data from the motors or sensors, such as motor temperature
    private final PivotInputsAutoLogged kPivotInputs = new PivotInputsAutoLogged();

    @AutoLogOutput(key = "Pivot/CurrentGoal")
    private PivotGoal currentPivotGoal = null;

    @AutoLogOutput(key = "Pivot/CurrentGoalPosition")
    private Rotation2d currentPivotGoalPosition = new Rotation2d();

    private final LoggedTunableNumber kP = new LoggedTunableNumber("Pivot/Gains/kP",
                                                                   ClimbConstants.kPivotGains.kP());
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Pivot/Gains/kI",
                                                                   ClimbConstants.kPivotGains.kI());
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Pivot/Gains/kD",
                                                                   ClimbConstants.kPivotGains.kD());
    private final LoggedTunableNumber kS = new LoggedTunableNumber("Pivot/Gains/kS",
                                                                   ClimbConstants.kPivotGains.kS());
    private final LoggedTunableNumber kV = new LoggedTunableNumber("Pivot/Gains/kV",
                                                                   ClimbConstants.kPivotGains.kV());
    private final LoggedTunableNumber kA = new LoggedTunableNumber("Pivot/Gains/kA",
                                                                   ClimbConstants.kPivotGains.kA());
    private final LoggedTunableNumber kG = new LoggedTunableNumber("Pivot/Gains/kG",
                                                                   ClimbConstants.kPivotGains.kG());
    private final LoggedTunableNumber kMaxVelocity = new LoggedTunableNumber("Pivot/MotionMagic/kMaxVelocity",
                                                                             ClimbConstants.kPivotGains.kMaxVelocity());
    private final LoggedTunableNumber kMaxAcceleration = new LoggedTunableNumber(
            "Pivot/MotionMagic/kMaxAcceleration", ClimbConstants.kPivotGains.kMaxAcceleration());

    /** Creates a new Pivot. */
    public Climb(ClimbKrakenHardware pivotHardware) {
        kPivotHardware = pivotHardware;
    }

    /** Code called every loop. */
    @Override
    public void periodic() {
        // We pass in kPivotInputs to kPivotHardware to "update" it with data from the hardware layer
        // (E.g. Motor temperatures, motor positions). This must be called periodically in order to
        // properly update and call values from kPivotInputs.
        kPivotHardware.updateInputs(kPivotInputs);
        Logger.processInputs("Pivot/Inputs", kPivotInputs);

        if (currentPivotGoal != null) {
            currentPivotGoalPosition = currentPivotGoal.getGoal();
            setPosition(currentPivotGoalPosition.getRadians());

            Logger.recordOutput("Pivot/Goal", currentPivotGoal);
        } else {
            Logger.recordOutput("Pivot/Goal", "NONE");
        }

        // Update feedback, feedforward, and motion magic gains if we change them from network tables
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> {
                kPivotHardware.setGains(
                    kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get(), kG.get()
                );
            },
            kP,
            kI,
            kD,
            kS,
            kV,
            kA,
            kG
        );
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> {
                kPivotHardware.setMotionMagicConstraints(kMaxVelocity.get(), kMaxAcceleration.get());
            },
            kMaxVelocity,
            kMaxAcceleration);
    }

    /**
    * Sets the desired goal of the pivot
    *
    * @param desiredGoal The desired goal of the pivot
    */
    public void setGoal(PivotGoal desiredGoal) {
        currentPivotGoal = desiredGoal;
    }

    /**
    * Sets the voltage of the motor. Note that {@link #setGoal} should be called and set to null in
    * order to not run position control while also attempting to control voltage.
    *
    * @param voltage The voltage to set the motor to
    */
    public void setVoltage(double voltage) {
        if (getPosition().getDegrees() > ClimbConstants.kUpperPositionLimit.getDegrees()
            && voltage > 0.0) {
            // do nothing
        } else if (getPosition().getDegrees() < ClimbConstants.kLowerPositionLimit.getDegrees()
            && voltage < 0.0) {
            // do nothing
        } else {
            kPivotHardware.setVoltage(voltage);
        }
    }

    /** Sets the motor control to neutral, then switching to the default neutral control mode */
    public void stop() {
        currentPivotGoal = null;
        kPivotHardware.stop();
    }

    public void resetPosition() {
        kPivotHardware.resetEncoder();
    }

    /**
    * Sets the desired position of the motor. Runs using internal PID controller
    *
    * @param positionRads The desired position in radians
    */
    private void setPosition(double positionRads) {
        kPivotHardware.setPosition(positionRads);
    }

    /**
    * Computs the current position error by subtracting the current position from the goal position
    *
    * @return The position error in radians
    */
    @AutoLogOutput(key = "Pivot/Feedback/ErrorRads")
    public double getErrorRads() {
        return currentPivotGoalPosition.minus(getPosition()).getRadians();
    }

    /**
    * Gets the internal position of the rotar adjusted for the mechanism's gear ratio
    *
    * @return The position of the mechanism
    */
    public Rotation2d getPosition() {
        return kPivotInputs.inteneralPosition;
    }
}
