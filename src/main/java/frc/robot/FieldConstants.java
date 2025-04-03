package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
public class FieldConstants {
    public static final Pose2d kReefCenter = new Pose2d(4.48249, Constants.kFieldWidthMeters / 2.0, Rotation2d.fromDegrees(0.0));
    public static final double kXNetLineMeters = 7.15;
    
    public static final Pose2d AL = new Pose2d(3.23, 4.42, Rotation2d.k180deg);
    public static final Pose2d AM = new Pose2d(3.22, 4.25, Rotation2d.k180deg);
    public static final Pose2d AR = new Pose2d(3.22, 4.08, Rotation2d.k180deg);

    public static final Pose2d BL = new Pose2d(4.21, 5.32, Rotation2d.fromDegrees(120.0));
    public static final Pose2d BM = new Pose2d(4.06, 5.24, Rotation2d.fromDegrees(120.0));
    public static final Pose2d BR = new Pose2d(3.91, 5.16, Rotation2d.fromDegrees(120.0));

    public static final Pose2d CL = new Pose2d(5.46, 4.92, Rotation2d.fromDegrees(60.0));
    public static final Pose2d CM = new Pose2d(5.32, 5, Rotation2d.fromDegrees(60.0));
    public static final Pose2d CR = new Pose2d(5.18, 5.08, Rotation2d.fromDegrees(60.0));

    public static final Pose2d DL = new Pose2d(5.76, 3.63, Rotation2d.kZero);
    public static final Pose2d DM = new Pose2d(5.76, 3.78, Rotation2d.kZero);
    public static final Pose2d DR = new Pose2d(5.76, 3.93, Rotation2d.kZero);

    public static final Pose2d EL = new Pose2d(4.77, 2.72, Rotation2d.fromDegrees(-60.0));
    public static final Pose2d EM = new Pose2d(4.915, 2.805, Rotation2d.fromDegrees(-60.0));
    public static final Pose2d ER = new Pose2d(5.06, 2.89, Rotation2d.fromDegrees(-60.0));

    public static final Pose2d FL = new Pose2d(3.51, 3.13, Rotation2d.fromDegrees(-120.0));
    public static final Pose2d FM = new Pose2d(3.66, 3.035, Rotation2d.fromDegrees(-120.0));
    public static final Pose2d FR = new Pose2d(3.81, 2.94, Rotation2d.fromDegrees(-120.0));
    

    // get 5411 constants //
    public static final Pose2d B_IL = new Pose2d(1.45, 7.21, Rotation2d.fromRadians(2.20));
    public static final Pose2d B_IR = new Pose2d(1.16, 0.99, Rotation2d.fromRadians(-2.20));

    public static final Pose2d R_IL = new Pose2d(16.0, 0.72, Rotation2d.fromRadians(2.20).plus(Rotation2d.k180deg));
    public static final Pose2d R_IR = new Pose2d(16.39, 7.06, Rotation2d.fromRadians(-2.20).plus(Rotation2d.k180deg));
}