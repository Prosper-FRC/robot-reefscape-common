package frc.robot.subsystems.drive.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.utils.debugging.LoggedTunableNumber;

import static frc.robot.subsystems.drive.DriveConstants.kMaxLinearSpeedMPS;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/* Controls the pose of the robot using 3 PID controllers and Feedforward */
public class ManualTeleopController {
    public static final LoggedTunableNumber linearScalar =
        new LoggedTunableNumber("Drive/Teleop/LinearScalar", 1);
    public static final LoggedTunableNumber linearDeadBand =
        new LoggedTunableNumber("Drive/Teleop/Deadband", 0.075);
    public static final LoggedTunableNumber linearInputsExponent =
        new LoggedTunableNumber("Drive/Teleop/LinearInputsExponent", 2);
    public static final LoggedTunableNumber rotationScalar =
        new LoggedTunableNumber("Drive/Teleop/RotationScalar", 0.5);
    public static final LoggedTunableNumber rotationInputsExponent =
        new LoggedTunableNumber("Drive/Teleop/RotationInputExponent", 1.0);
    public static final LoggedTunableNumber rotationDeadband =
        new LoggedTunableNumber("Drive/Teleop/RotationDeadband", 0.1);
    public static final LoggedTunableNumber sniperControl =
        new LoggedTunableNumber("Drive/Teleop/SniperControl", 0.2);

    private boolean fieldRelative = true;

    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;
    private DoubleSupplier omegaSupplier;

    private DoubleSupplier povSupplierDegrees;

    public ManualTeleopController() {}

    public void acceptJoystickInputs(
        DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier, DoubleSupplier povSupplierDegrees) {
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.omegaSupplier = omegaSupplier;

        this.povSupplierDegrees = povSupplierDegrees;
    }

    public ChassisSpeeds computeChassiSpeeds(Rotation2d robotAngle, ChassisSpeeds currentRobotRelativeSpeeds, boolean joystickSniper) {
        double xAdjustedJoystickInput = MathUtil.applyDeadband(xSupplier.getAsDouble(), linearDeadBand.get());
        double yAdjustedJoystickInput = MathUtil.applyDeadband(ySupplier.getAsDouble(), linearDeadBand.get());
        double omegaAdjustedJoystickInput = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), rotationDeadband.get());

        // EXPONENTS CAN ONLY BE INTEGERS FOR THIS TO WORK DUE TO MODULUS
        int linearExp = (int) Math.round(linearInputsExponent.get());
        int rotationExp = (int) Math.round(rotationInputsExponent.get());

        // Should never exceed 1 for exponent control to work. Clamped later as an edge case, but not a concern with XBox Controllers HID class
        double xJoystickScalar = getSniperScalar(joystickSniper) * linearScalar.get();
        double yJoystickScalar = getSniperScalar(joystickSniper) * linearScalar.get();
        double omegaJoystickScalar = getSniperScalar(joystickSniper) * linearScalar.get();

        double xScaledJoystickInput =
            zerotoOneClamp(xJoystickScalar)
            * Math.pow(xAdjustedJoystickInput, linearExp);
        double yScaledJoystickInput =
            zerotoOneClamp(yJoystickScalar)
            * Math.pow(yAdjustedJoystickInput, linearExp);
        double omegaJoystickInput =
            zerotoOneClamp(omegaJoystickScalar)
            * Math.pow(omegaAdjustedJoystickInput, rotationExp);

        /* If the exponent is an even number it's always positive */
        /* You lose the sign when it's squared, so you have to multiply it back in  */
        if (linearExp % 2 == 0) {
            xScaledJoystickInput *= Math.signum(xAdjustedJoystickInput);
            yScaledJoystickInput *= Math.signum(yAdjustedJoystickInput);
        }

        if (rotationExp % 2 == 0) {
            omegaJoystickInput *= Math.signum(omegaAdjustedJoystickInput);
        }


        Logger.recordOutput("Drive/Teleop/preOffsetAngle", robotAngle);
        /* 
         * Field relative only works if the robot starts on blue side
         * Because the driver faces the other direction relative to field when on red
         * So we flip the true field perspective to the red side by adding 180 to 
         * re-align directions
        */
        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red)) 
            robotAngle = robotAngle.plus(Rotation2d.k180deg);
        Logger.recordOutput("Drive/Teleop/offsetAngle", robotAngle);

        ChassisSpeeds desiredSpeeds = new ChassisSpeeds( 
            DriveConstants.kMaxLinearSpeedMPS * xScaledJoystickInput, 
            DriveConstants.kMaxLinearSpeedMPS * yScaledJoystickInput, 
            DriveConstants.kMaxRotationSpeedRadiansPS * omegaJoystickInput);

        if (fieldRelative) {
            desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                desiredSpeeds, robotAngle);
        }

        return desiredSpeeds;
    }

    /* Wheter to use the snipler scalar or not based on the cnodition */
    public double getSniperScalar(boolean isSniper) {
        return isSniper ? sniperControl.get() : 1.0;
    }

    /*
     * Takes in the POV angle and then moves the robot in that angle at sniper speeds
     * If the driver wants simply controlled direction at low speeds for the robot to make little linear adjustments
     */
    public ChassisSpeeds computeSniperPOVChassisSpeeds(Rotation2d robotAngle) {
        double omegaAdjustedJoystickInput = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), rotationDeadband.get());

        int rotationExp = (int) Math.round(rotationInputsExponent.get());

        /* If the exponent is an even number it's always positive */
        /* You lose the sign when it's squared, so you have to multiply it back in  */
        double omegaJoystickScalar = getSniperScalar(true) * linearScalar.get();

        double omegaJoystickInput =
            zerotoOneClamp(omegaJoystickScalar)
            * Math.pow(omegaAdjustedJoystickInput, rotationExp);

        if (rotationExp % 2 == 0) {
            omegaJoystickInput *= Math.signum(omegaAdjustedJoystickInput);
        }

        /* Does polar to rectangular where POV degree is theta, kSniperControl * maxSpeed is r */
        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(
            sniperControl.get() * kMaxLinearSpeedMPS * Math.cos(Math.toRadians(povSupplierDegrees.getAsDouble())), 
            sniperControl.get() * kMaxLinearSpeedMPS * Math.sin(Math.toRadians(povSupplierDegrees.getAsDouble())), 
            DriveConstants.kMaxRotationSpeedRadiansPS * omegaJoystickInput);

        if (fieldRelative) {
            desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                desiredSpeeds, robotAngle);
        }

        return desiredSpeeds;
    }

    public void toggleFieldOriented() {
        fieldRelative = !fieldRelative;
    }

    private double zerotoOneClamp(double val) {
        return MathUtil.clamp(val, 0.0, 1.0);
    }
}
