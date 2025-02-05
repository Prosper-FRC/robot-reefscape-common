package frc.robot.subsystems.drive.controllers;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.utils.debugging.LoggedTunableNumber;

public class TeleopController {

    // Suppliers are essentially the values that are constantly grabbed from the controller //
    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;
    private DoubleSupplier omegaSupplier;

    // Determines if the robot should be field or robot oriented //
    private boolean fieldRelative = true;

    // Logged Tuneable Numbers make it easier to change the controller to the drivers preferance //

    // Changes how much the joystick needs to move before its value is no longer 0 //
    public static final LoggedTunableNumber linearDeadband = new LoggedTunableNumber("Drive/Teleop/LinearDeadband", 0.1);
    public static final LoggedTunableNumber omegaDeadband = new LoggedTunableNumber("Drive/Teleop/RotationalDeadband", 0.1);

    // The Scalar can proportionally make the linear/rotational speed faster or slower //
    public static final LoggedTunableNumber linearScalar = new LoggedTunableNumber("Drive/Teleop/LinearScalar", 1);
    public static final LoggedTunableNumber omegaScalar = new LoggedTunableNumber("Drive/Teleop/RotationScalar", 1);

    // 
    public static final LoggedTunableNumber rotationInputsExponent = new LoggedTunableNumber("Drive/Teleop/RotationInputExponent", 1);
    public static final LoggedTunableNumber linearInputsExponent = new LoggedTunableNumber("Drive/Teleop/LinearInputExponent", 1);



    public TeleopController(){}

    public void acceptJoystickInputs(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier){
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.omegaSupplier = omegaSupplier;
    }

    public ChassisSpeeds computeChassisSpeeds(Rotation2d robotAngle, ChassisSpeeds currentRelativeSpeeds){

        // Scales the filtered value if the person driving wanted a slower or faster robot //
        double xChangedDemand = linearScalar.get() * applyDeadband(xSupplier.getAsDouble(), linearDeadband.get());
        double yChangedDemand = linearScalar.get() * applyDeadband(ySupplier.getAsDouble(), linearDeadband.get());
        double omegaChangedDemand = omegaScalar.get() * applyDeadband(omegaSupplier.getAsDouble(), omegaDeadband.get());

        // Makes sure that the exponenet is a whole number //
        int linearExp = (int) Math.round(linearInputsExponent.get());
        int rotationExp = (int) Math.round(rotationInputsExponent.get());

        // Converts the value to m/s for the chassis speeds //
        double xVelocityMPS = DriveConstants.kMaxLinearSpeed * Math.pow(xChangedDemand, linearExp);
        double yVelocityMPS = DriveConstants.kMaxLinearSpeed * Math.pow(yChangedDemand, linearExp);
        double rotationVelocityRPS = DriveConstants.kMaxRotationalSpeedRadians * Math.pow(omegaChangedDemand, rotationExp);

        if (linearExp % 2 == 0) {
            // Signum is used to changed the direction of the velocity //

            // Returns -1 for a value less than 0 //
            // Returns 0 for a value that is 0 //
            // Returns 1 for a value greater than 0 //
            xVelocityMPS *= Math.signum(xChangedDemand);
            yVelocityMPS *= Math.signum(yChangedDemand);
        }

        if (rotationExp % 2 == 0) {
            rotationVelocityRPS *= Math.signum(omegaChangedDemand);
        }

        // Chassis Speeds that are robot-relative //
        ChassisSpeeds desiredSpeeds = new ChassisSpeeds( 
            xVelocityMPS, 
            yVelocityMPS, 
            rotationVelocityRPS);

        // Converted to field relative as its easier to drive //
        if(fieldRelative){
            desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredSpeeds, robotAngle);
        }

        return desiredSpeeds;
    }

    // Can be binded to a button //
    public void toggleFieldOrientation(){
        fieldRelative = !fieldRelative;
    }

    /**
     * Deadband to be appplied to joysticks to account for stick drift
     * @param input value from the joystick
     * @param range positive double that 
     * @return filtered input
     */
    private double applyDeadband(double input, double range){
        if(-range < input && input < range){
            return 0;
        }
        return input;
    }
}
