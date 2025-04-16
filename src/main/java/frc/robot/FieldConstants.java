package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FieldConstants {
    public static final Pose2d kReefCenter = new Pose2d(4.48249, Constants.kFieldWidthMeters / 2.0, Rotation2d.fromDegrees(0.0));
    public static final double kXNetLineMeters = 7.15;

    public static final Pose2d AL = new Pose2d(3.23, 4.44, Rotation2d.k180deg);
    public static final Pose2d AR = new Pose2d(3.22, 4.14, Rotation2d.k180deg);
    public static final Pose2d AM = new Pose2d(average(AL.getX(), AR.getX()), average(AL.getY(), AR.getY()), Rotation2d.k180deg);

    public static final Pose2d BL = new Pose2d(4.22, 5.32, Rotation2d.fromDegrees(120.0));
    public static final Pose2d BR = new Pose2d(3.94, 5.17, Rotation2d.fromDegrees(120.0));
    public static final Pose2d BM = new Pose2d(average(BL.getX(), BR.getX()), average(BL.getY(), BR.getY()), Rotation2d.fromDegrees(120.0));

    public static final Pose2d CL = new Pose2d(5.49, 4.91, Rotation2d.fromDegrees(60.0));
    public static final Pose2d CR = new Pose2d(5.21, 5.07, Rotation2d.fromDegrees(60.0));
    public static final Pose2d CM = new Pose2d(average(CL.getX(), CR.getX()), average(CL.getY(), CR.getY()), Rotation2d.fromDegrees(60.0));

    public static final Pose2d DL = new Pose2d(5.75, 3.61, Rotation2d.kZero);
    public static final Pose2d DR = new Pose2d(5.75, 3.93, Rotation2d.kZero);
    public static final Pose2d DM = new Pose2d(average(DL.getX(), DR.getX()), average(DL.getY(), DR.getY()), Rotation2d.kZero);

    public static final Pose2d EL = new Pose2d(4.76, 2.72, Rotation2d.fromDegrees(-60.0));
    public static final Pose2d ER = new Pose2d(5.03, 2.88, Rotation2d.fromDegrees(-60.0));
    public static final Pose2d EM = new Pose2d(average(EL.getX(), ER.getX()), average(EL.getY(), ER.getY()), Rotation2d.fromDegrees(-60.0));

    public static final Pose2d FL = new Pose2d(3.48, 3.15, Rotation2d.fromDegrees(-120.0));
    public static final Pose2d FR = new Pose2d(3.78, 2.98, Rotation2d.fromDegrees(-120.0));
    public static final Pose2d FM = new Pose2d(average(FL.getX(), FR.getX()), average(FL.getY(), FR.getY()), Rotation2d.fromDegrees(-120.0));
    
    public static final Pose2d B_IL = new Pose2d(1.42, 7.21, Rotation2d.fromRadians(2.20));
    public static final Pose2d B_IR = new Pose2d(1.17, 1.02, Rotation2d.fromRadians(-2.20));

    public static final Pose2d R_IL = new Pose2d(16.08, 0.78, Rotation2d.fromRadians(2.20).plus(Rotation2d.k180deg));
    public static final Pose2d R_IR = new Pose2d(16.45, 7.01, Rotation2d.fromRadians(-2.20).plus(Rotation2d.k180deg));

    public static double average(double a, double b) {
        return (a + b) / 2.0;
    }
}