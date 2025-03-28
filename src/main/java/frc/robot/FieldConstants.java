package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
public class FieldConstants {
    public static final Pose2d kReefCenter = new Pose2d(4.48249, Constants.kFieldWidthMeters / 2.0, Rotation2d.fromDegrees(0.0));
    public static final double kXNetLineMeters = 7.15;
    
    public static final Pose2d AL = new Pose2d(3.22, 4.42, Rotation2d.k180deg);
    public static final Pose2d AM = new Pose2d(3.14, 4.02, Rotation2d.k180deg);
    public static final Pose2d AR = new Pose2d(3.22, 4.10, Rotation2d.k180deg);

    public static final Pose2d BL = new Pose2d(4.2, 5.34, Rotation2d.fromDegrees(120.0));
    public static final Pose2d BM = new Pose2d(3.85, 5.13, Rotation2d.fromDegrees(120.0));
    public static final Pose2d BR = new Pose2d(3.93, 5.17, Rotation2d.fromDegrees(120.0));

    public static final Pose2d CL = new Pose2d(5.48, 4.91, Rotation2d.fromDegrees(60.0));
    public static final Pose2d CM = new Pose2d(5.15, 5.14, Rotation2d.fromDegrees(60.0));
    public static final Pose2d CR = new Pose2d(5.18, 4.36, Rotation2d.fromDegrees(60.0));

    public static final Pose2d DL = new Pose2d(5.75, 3.61, Rotation2d.kZero);
    public static final Pose2d DM = new Pose2d(5.76, 4, Rotation2d.kZero);
    public static final Pose2d DR = new Pose2d(5.76, 3.96, Rotation2d.kZero);

    public static final Pose2d FL = new Pose2d(3.50, 3.13, Rotation2d.fromDegrees(-120.0));
    public static final Pose2d FM = new Pose2d(3.86, 2.88, Rotation2d.fromDegrees(-120.0));
    public static final Pose2d FR = new Pose2d(3.80, 2.95, Rotation2d.fromDegrees(-120.0));

    public static final Pose2d EL = new Pose2d(4.76, 2.71, Rotation2d.fromDegrees(-60.0));
    public static final Pose2d EM = new Pose2d(5.28, 2.46, Rotation2d.fromDegrees(-60.0));
    public static final Pose2d ER = new Pose2d(5.06, 2.88, Rotation2d.fromDegrees(-60.0));
    

    // get 5411 constants //
    public static final Pose2d B_IL = new Pose2d(1.45, 7.27, Rotation2d.fromRadians(2.20));
    public static final Pose2d B_IR = new Pose2d(1.13, 1.04, Rotation2d.fromRadians(-2.20));

    public static final Pose2d R_IL = new Pose2d(16.09, 0.75, Rotation2d.fromRadians(2.20).plus(Rotation2d.k180deg));
    public static final Pose2d R_IR = new Pose2d(16.39, 7.01, Rotation2d.fromRadians(-2.20).plus(Rotation2d.k180deg));
}