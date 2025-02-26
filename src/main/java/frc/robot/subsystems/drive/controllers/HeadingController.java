package frc.robot.subsystems.drive.controllers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.utils.debugging.LoggedTunableNumber;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

/* Controls the heading of the robot using PID and Feedforward 
 * Thoroughly tested at NTX and showed positive results
 * https://docs.google.com/document/d/1NHhyKmQlgkni1lrRiwqWf7-vbZUfIpNTPCeMLEWlYcE/edit?usp=sharing
 * Past research on topic. PID+FF strategy is commonly used for small distance(0-5 meter) drive to pose control
*/
public class HeadingController {
    public static final LoggedTunableNumber snapP =
        new LoggedTunableNumber("SwerveHeadingController/Snap/kP", 4.0);
    public static final LoggedTunableNumber snapI =
        new LoggedTunableNumber("SwerveHeadingController/Snap/kI", 0.0);
    public static final LoggedTunableNumber snapD =
        new LoggedTunableNumber("SwerveHeadingController/Snap/kD", 0.0);
    public static final LoggedTunableNumber snapMaxVDPS =
        new LoggedTunableNumber("SwerveHeadingController/Snap/kMaxV", 1000.0);
    public static final LoggedTunableNumber snapMaxADPSS =
        new LoggedTunableNumber("SwerveHeadingController/Snap/kMaxA", 1000.0);

    // public static final LoggedTunableNumber stablizingP =
    //     new LoggedTunableNumber("SwerveHeadingController/Stabilizing/kP", 2.5);
    // public static final LoggedTunableNumber stablizingI =
    //     new LoggedTunableNumber("SwerveHeadingController/Stabilizing/kI", 0.0);
    // public static final LoggedTunableNumber stablizingD =
    //     new LoggedTunableNumber("SwerveHeadingController/Stabilizing/kD", 0.0);

    public static final LoggedTunableNumber toleranceDegrees =
        new LoggedTunableNumber("SwerveHeadingController/Tolerance", 0.75);

    private ProfiledPIDController snapController;
  
    private PIDController stabilizingController;
  
    private Supplier<Rotation2d> goal;

    public HeadingController() {
        snapController = new ProfiledPIDController(
            snapP.get(),
            snapI.get(),
            snapD.get(),
            new TrapezoidProfile.Constraints(snapMaxVDPS.get(), snapMaxADPSS.get()));

        // invert = (RobotBase.isReal()) ? -1.0 : 1.0;
        snapController.enableContinuousInput(0, 360);
        snapController.setTolerance(1.0);

        // stabilizingController =
        //     new PIDController(stablizingP.get(), stablizingI.get(), stablizingD.get());
        // stabilizingController.enableContinuousInput(0, 360);
        // stabilizingController.setTolerance(0.0);
    }

    public void setHeadingGoal(Supplier<Rotation2d> goalSupplier) {
        goal = goalSupplier;
    }

    public void reset(Rotation2d robotRotation, Rotation2d robotRotationPerSecond) {
        snapController.reset(robotRotation.getDegrees(), robotRotationPerSecond.getDegrees());
    }

    // Designed for large jumps toward a setpoint, kind of like an azimuth alignment
    public double getSnapOutput(Rotation2d robotRotation) {
        Logger.recordOutput("Drive/HeadingController/HeadingSetpoint", Rotation2d.fromDegrees(snapController.getSetpoint().position));

        double pidOutput = snapController.calculate(robotRotation.getDegrees(), goal.get().getDegrees());
        double ffOutput = snapController.getSetpoint().velocity;
        double output = Math.toRadians(pidOutput + ffOutput);

        double setpointErrorDegrees = snapController.getSetpoint().position - robotRotation.getDegrees();
        double goalErrorDegrees = snapController.getGoal().position - robotRotation.getDegrees();

        double adjustedOutput = output;
        if(Math.abs(goalErrorDegrees) < toleranceDegrees.get()) adjustedOutput *= 0.0;

        Logger.recordOutput("Drive/HeadingController/unAdjustedOutput", output);

        // Logger.recordOutput("Drive/HeadingController/setpointErrorDegrees", setpointErrorDegrees);
        // Logger.recordOutput("Drive/HeadingController/goalErrorDegrees", goalErrorDegrees);

        Logger.recordOutput("Drive/HeadingController/adjustedOutput", adjustedOutput);

        Logger.recordOutput("Drive/HeadingController/pidOutput", pidOutput);
        Logger.recordOutput("Drive/HeadingController/ffOutput", ffOutput);

        return output;
    }

    // Designed for shoot on move and short distance. In most cases velocityDPS is 0.
    public double getStabilizingOutput(Rotation2d robotRotation, double velocityDPS) {
        return Math.toRadians(
            stabilizingController.calculate(robotRotation.getDegrees(), goal.get().getDegrees())
            + velocityDPS);
    }

    public void updateHeadingController() {
        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            snapController.setPID(snapP.get(), snapI.get(), snapD.get());
            snapController.setConstraints(new Constraints(snapMaxVDPS.get(), snapMaxADPSS.get()));
        }, snapP, snapI, snapD, snapMaxVDPS, snapMaxADPSS);

        // LoggedTunableNumber.ifChanged(hashCode(), () -> {
        //     stabilizingController.setPID(stablizingP.get(), stablizingI.get(), stablizingD.get());
        // }, stablizingP, stablizingI, stablizingD);
    }
}