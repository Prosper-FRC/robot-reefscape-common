package frc.robot.subsystems.drive.controllers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.utils.debugging.LoggedTunableNumber;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class HeadingController {
    public static final LoggedTunableNumber kSnapP = new LoggedTunableNumber("SwerveHeadingController/Snap/kP", 4.0);
    public static final LoggedTunableNumber kSnapI = new LoggedTunableNumber("SwerveHeadingController/Snap/kI", 0.0);
    public static final LoggedTunableNumber kSnapD = new LoggedTunableNumber("SwerveHeadingController/Snap/kD", 0.0);
    public static final LoggedTunableNumber kSnapMaxVDPS = new LoggedTunableNumber("SwerveHeadingController/Snap/kMaxV", 1000.0);
    public static final LoggedTunableNumber kSnapMaxADPSS = new LoggedTunableNumber("SwerveHeadingController/Snap/kMaxA", 1000.0);

    public static final LoggedTunableNumber kS = new LoggedTunableNumber("SwerveHeadingController/Snap/kD", 50);

    public static final LoggedTunableNumber kStablizingP = new LoggedTunableNumber("SwerveHeadingController/Stabilizing/kP", 2.5);
    public static final LoggedTunableNumber kStablizingI = new LoggedTunableNumber("SwerveHeadingController/Stabilizing/kI", 0.0);
    public static final LoggedTunableNumber kStablizingD = new LoggedTunableNumber("SwerveHeadingController/Stabilizing/kD", 0.0);
     public static final LoggedTunableNumber kToleranceDegrees = new LoggedTunableNumber("SwerveHeadingController/Stabilizing/kD", 0.75);

    private ProfiledPIDController snapController;
  
    private PIDController stabilizingController;
  
    private Supplier<Rotation2d> goal;

    private double invert;

    public HeadingController() {
        snapController = new ProfiledPIDController(
            kSnapP.get(),
            kSnapI.get(),
            kSnapD.get(),
            new TrapezoidProfile.Constraints(kSnapMaxVDPS.get(), kSnapMaxADPSS.get()));

        // Invert this if robot starts to oscillating //
        invert = (RobotBase.isReal()) ? 1.0 : 1.0;
        snapController.enableContinuousInput(0, 360);
        snapController.setTolerance(1.0);

        stabilizingController = new PIDController(
            kStablizingP.get(), 
            kStablizingI.get(), 
            kStablizingD.get());
            
        stabilizingController.enableContinuousInput(0, 360);
        stabilizingController.setTolerance(0.0);
    }

    public void setHeadingGoal(Supplier<Rotation2d> goalSupplier) {
        goal = goalSupplier;
    }

    public void resetController(Rotation2d robotRotation, Rotation2d robotRotationPerSecond) {
        snapController.reset(robotRotation.getDegrees(), robotRotationPerSecond.getDegrees());
    }

    public double getControlledHeadingOutput(Rotation2d robotRotation) {
        return (snapController.getGoal().equals(snapController.getSetpoint())
            && snapController.getPositionError() < kToleranceDegrees.get())
            ? getStabilizingOutput(robotRotation, snapController.getSetpoint().velocity)
            : getSnapOutput(robotRotation) + snapController.getSetpoint().velocity;
    }

    // Designed for large jumps toward a setpoint, kind of like an azimuth alignment
    public double getSnapOutput(Rotation2d robotRotation) {
        Logger.recordOutput("Drive/HeadingController/HeadingSetpoint", Rotation2d.fromDegrees(snapController.getSetpoint().position));

        double output = invert * Math.toRadians(
            snapController.calculate(robotRotation.getDegrees(), goal.get().getDegrees()))
            + Math.signum(snapController.getPositionError()) * kS.get();

        Logger.recordOutput("Drive/HeadingController/pidOutput", snapController.calculate(robotRotation.getDegrees(), goal.get().getDegrees()));
        Logger.recordOutput("Drive/HeadingController/unAdjustedOutput", output);

        double setpointErrorDegrees = snapController.getSetpoint().position - robotRotation.getDegrees();
        double goalErrorDegrees = snapController.getGoal().position - robotRotation.getDegrees();

        Logger.recordOutput("Drive/HeadingController/setpointErrorDegrees", setpointErrorDegrees);
        Logger.recordOutput("Drive/HeadingController/goalErrorDegrees", goalErrorDegrees);

        if(Math.abs(goalErrorDegrees) < kToleranceDegrees.get()) output *= 0.0;
        Logger.recordOutput("Drive/HeadingController/adjustedOutput", output);

        return output;
    }

    // Designed for shoot on move and short distance. In most cases velocityDPS is 0.
    public double getStabilizingOutput(Rotation2d robotRotation, double velocityDPS) {
        return invert * Math.toRadians(
            stabilizingController.calculate(robotRotation.getDegrees(), goal.get().getDegrees())
            + velocityDPS) + Math.signum(stabilizingController.getPositionError()) * kS.get();
    }

    public void updateHeadingControllerConfig() {
        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            snapController.setPID(kSnapP.get(), kSnapI.get(), kSnapD.get());
            snapController.setConstraints(new Constraints(kSnapMaxVDPS.get(), kSnapMaxADPSS.get()));
        }, kSnapP, kSnapI, kSnapD, kSnapMaxVDPS, kSnapMaxADPSS);

        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            stabilizingController.setPID(kStablizingP.get(), kStablizingI.get(), kStablizingD.get());
        }, kStablizingP, kStablizingI, kStablizingD);
    }
}