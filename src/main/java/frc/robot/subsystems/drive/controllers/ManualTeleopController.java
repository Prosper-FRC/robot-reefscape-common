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

public class ManualTeleopController {
    public static final LoggedTunableNumber kLinearScalar =
        new LoggedTunableNumber("Drive/Teleop/LinearScalar", 1);
    public static final LoggedTunableNumber kLinearDeadBand =
        new LoggedTunableNumber("Drive/Teleop/Deadband", 0.075);
    public static final LoggedTunableNumber kLinearInputsExponent =
        new LoggedTunableNumber("Drive/Teleop/LinearInputsExponent", 2);
    public static final LoggedTunableNumber kRotationScalar =
        new LoggedTunableNumber("Drive/Teleop/RotationScalar", 0.5);
    public static final LoggedTunableNumber kRotationInputsExponent =
        new LoggedTunableNumber("Drive/Teleop/RotationInputExponent", 1.0);
    public static final LoggedTunableNumber kRotationDeadband =
        new LoggedTunableNumber("Drive/Teleop/RotationDeadband", 0.1);
    public static final LoggedTunableNumber kSniperControl =
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
        double xAdjustedJoystickInput = MathUtil.applyDeadband(xSupplier.getAsDouble(), kLinearDeadBand.get());
        double yAdjustedJoystickInput = MathUtil.applyDeadband(ySupplier.getAsDouble(), kLinearDeadBand.get());
        double omegaAdjustedJoystickInput = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), kRotationDeadband.get());

        // EXPONENTS CAN ONLY BE INTEGERS FOR THIS TO WORK DUE TO MODULUS
        int linearExp = (int) Math.round(kLinearInputsExponent.get());
        int rotationExp = (int) Math.round(kRotationInputsExponent.get());

        // Should never exceed 1 for exponent control to work
        double xJoystickScalar = getSniperScalar(joystickSniper) * kLinearScalar.get();
        double yJoystickScalar = getSniperScalar(joystickSniper) * kLinearScalar.get();
        double omegaJoystickScalar = getSniperScalar(joystickSniper) * kLinearScalar.get();

        double xScaledJoystickInput =
            zerotoOneClamp(xJoystickScalar)
            * Math.pow(xAdjustedJoystickInput, linearExp);
        double yScaledJoystickInput =
            zerotoOneClamp(yJoystickScalar)
            * Math.pow(yAdjustedJoystickInput, linearExp);
        double omegaJoystickInput =
            zerotoOneClamp(omegaJoystickScalar)
            * Math.pow(omegaAdjustedJoystickInput, rotationExp);

        if (linearExp % 2 == 0) {
            xScaledJoystickInput *= Math.signum(xAdjustedJoystickInput);
            yScaledJoystickInput *= Math.signum(yAdjustedJoystickInput);
        }

        if (rotationExp % 2 == 0) {
            omegaJoystickInput *= Math.signum(omegaAdjustedJoystickInput);
        }

        Logger.recordOutput("Drive/Teleop/preOffsetAngle", robotAngle);
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

    public double getSniperScalar(boolean isSniper) {
        return isSniper ? kSniperControl.get() : 1.0;
    }

    public ChassisSpeeds computeSniperPOVChassisSpeeds(Rotation2d robotAngle) {
        double omegaAdjustedJoystickInput = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), kRotationDeadband.get());

        int rotationExp = (int) Math.round(kRotationInputsExponent.get());

        double omegaJoystickScalar = getSniperScalar(true) * kLinearScalar.get();

        double omegaJoystickInput =
            zerotoOneClamp(omegaJoystickScalar)
            * Math.pow(omegaAdjustedJoystickInput, rotationExp);

        if (rotationExp % 2 == 0) {
            omegaJoystickInput *= Math.signum(omegaAdjustedJoystickInput);
        }

        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(
            kSniperControl.get() * kMaxLinearSpeedMPS * Math.cos(Math.toRadians(povSupplierDegrees.getAsDouble())), 
            kSniperControl.get() * kMaxLinearSpeedMPS * Math.sin(Math.toRadians(povSupplierDegrees.getAsDouble())), 
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
