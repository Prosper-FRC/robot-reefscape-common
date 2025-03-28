package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
public class FieldConstants {
    public static final Pose2d kReefCenter = new Pose2d(4.48249, Constants.kFieldWidthMeters / 2.0, Rotation2d.fromDegrees(0.0));
    public static final double kXNetLineMeters = 7.15;
    public static final Pose2d AL = new Pose2d(3.22, 4.41, Rotation2d.k180deg);
    public static final Pose2d AM = new Pose2d(3.14, 4.02, Rotation2d.k180deg);
    public static final Pose2d AR = new Pose2d(3.22, 4.09, Rotation2d.k180deg);
    public static final Pose2d BL = new Pose2d(4.17, 5.31, Rotation2d.fromDegrees(120.0));
    public static final Pose2d BM = new Pose2d(3.85, 5.13, Rotation2d.fromDegrees(120.0));
    public static final Pose2d BR = new Pose2d(3.90, 5.15, Rotation2d.fromDegrees(120.0));
    public static final Pose2d CL = new Pose2d(5.46, 4.92, Rotation2d.fromDegrees(60.0));
    public static final Pose2d CM = new Pose2d(5.15, 5.14, Rotation2d.fromDegrees(60.0));
    public static final Pose2d CR = new Pose2d(5.18, 5.09, Rotation2d.fromDegrees(60.0));
    public static final Pose2d DL = new Pose2d(5.76, 3.63, Rotation2d.kZero);
    public static final Pose2d DM = new Pose2d(5.76, 4, Rotation2d.kZero);
    public static final Pose2d DR = new Pose2d(5.76, 3.95, Rotation2d.kZero);

    public static final Pose2d FL = new Pose2d(3.51, 3.12, Rotation2d.fromDegrees(-120.0));
    public static final Pose2d FM = new Pose2d(3.86, 2.88, Rotation2d.fromDegrees(-120.0));
    public static final Pose2d FR = new Pose2d(3.80, 2.96, Rotation2d.fromDegrees(-120.0));

    public static final Pose2d EL = new Pose2d(4.79, 2.72, Rotation2d.fromDegrees(-60.0));
    public static final Pose2d EM = new Pose2d(5.15, 2.94, Rotation2d.fromDegrees(-60.0));
    public static final Pose2d ER = new Pose2d(5.07, 2.88, Rotation2d.fromDegrees(-60.0));
    
    public static final Pose2d B_IL = new Pose2d(1.58, 7.38, Rotation2d.fromRadians(2.20));
    public static final Pose2d B_IR = new Pose2d(1.10, 1.03, Rotation2d.fromRadians(-2.20));

    public static final Pose2d R_IL = new Pose2d(16.03, 0.72, Rotation2d.fromRadians(2.20).plus(Rotation2d.k180deg));
    public static final Pose2d R_IR = new Pose2d(16.42, 7.01, Rotation2d.fromRadians(-2.20).plus(Rotation2d.k180deg));
}