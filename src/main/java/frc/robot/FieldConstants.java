package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FieldConstants {
    public static final Pose2d kReefCenter = new Pose2d(4.48249, Constants.kFieldWidthMeters / 2.0, Rotation2d.fromDegrees(0.0));
    public static final double kXNetLineMeters = 7.15;

    public static final Pose2d AL = new Pose2d(3.2, 4.4, Rotation2d.k180deg);
    public static final Pose2d AR = new Pose2d(3.22, 4.08, Rotation2d.k180deg);
    public static final Pose2d AM = new Pose2d(average(AL.getX(), AR.getX()), average(AL.getY(), AR.getY()), Rotation2d.k180deg);

    public static final Pose2d BL = new Pose2d(4.18, 5.31, Rotation2d.fromDegrees(120.0));
    // public static final Pose2d BL = new Pose2d(3.496, 3.068, Rotation2d.fromDegrees(120.0));

    public static final Pose2d BR = new Pose2d(3.91, 5.15, Rotation2d.fromDegrees(120.0));
    public static final Pose2d BM = new Pose2d(average(BL.getX(), BR.getX()), average(BL.getY(), BR.getY()), Rotation2d.fromDegrees(120.0));

    public static final Pose2d CL = new Pose2d(5.49, 4.94, Rotation2d.fromDegrees(60.0));
    public static final Pose2d CR = new Pose2d(5.22, 5.05, Rotation2d.fromDegrees(60.0));
    public static final Pose2d CM = new Pose2d(average(CL.getX(), CR.getX()), average(CL.getY(), CR.getY()), Rotation2d.fromDegrees(60.0));

    public static final Pose2d DL = new Pose2d(5.77, 3.62, Rotation2d.kZero);
    public static final Pose2d DR = new Pose2d(5.76, 3.95, Rotation2d.kZero);
    public static final Pose2d DM = new Pose2d(average(DL.getX(), DR.getX()), average(DL.getY(), DR.getY()), Rotation2d.kZero);

    public static final Pose2d EL = new Pose2d(4.78, 2.72, Rotation2d.fromDegrees(-60.0));
    public static final Pose2d ER = new Pose2d(5.06, 2.89, Rotation2d.fromDegrees(-60.0));
    public static final Pose2d EM = new Pose2d(average(EL.getX(), ER.getX()), average(EL.getY(), ER.getY()), Rotation2d.fromDegrees(-60.0));

    // public static final Pose2d FL = new Pose2d(3.496, 3.068, Rotation2d.fromDegrees(-120.0));
    public static final Pose2d FL = new Pose2d(3.52, 3.1, Rotation2d.fromDegrees(-120.0));

    public static final Pose2d FR = new Pose2d(3.79, 2.96, Rotation2d.fromDegrees(-120.0));
    public static final Pose2d FM = new Pose2d(average(FL.getX(), FR.getX()), average(FL.getY(), FR.getY()), Rotation2d.fromDegrees(-120.0));
    
    public static final Pose2d B_IL = new Pose2d(1.62, 7.4, Rotation2d.fromRadians(2.20));
    public static final Pose2d B_IR = new Pose2d(1.32, 0.86, Rotation2d.fromRadians(-2.20));

    public static final Pose2d R_IL = new Pose2d(15.9, 0.58, Rotation2d.fromRadians(2.20).plus(Rotation2d.k180deg));
    public static final Pose2d R_IR = new Pose2d(16.27, 7.05, Rotation2d.fromRadians(-2.20).plus(Rotation2d.k180deg));

    public static double average(double a, double b) {
        return (a + b) / 2.0;
    }
}