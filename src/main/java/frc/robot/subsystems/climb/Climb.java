package frc.robot.subsystems.climb;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase{
    
    public enum ClimbGoal {
        kGrab(() -> 8.0),
        kRelease(() -> -4.0),
        kStop(() -> 0.0);
    

        private DoubleSupplier climbVolts;

        ClimbGoal(DoubleSupplier climbVolts){
            this.climbVolts = climbVolts;
        }

        public double getGoalVolts() {
            return this.climbVolts.getAsDouble();
        }
    }

    @AutoLogOutput(key="Climb/Goal")
    private ClimbGoal goal = null;

    private final ClimbIO kLeftIo;
    private final ClimbIOInputsAutoLogged kLeftInputs = new ClimbIOInputsAutoLogged();

    private final ClimbIO kRightIo;
    private final ClimbIOInputsAutoLogged kRightInputs = new ClimbIOInputsAutoLogged();


    public Climb(ClimbIO[] motors){
        kLeftIo = motors[0];
        kRightIo = motors[1];
    }

    @Override
    public void periodic() {
        kLeftIo.updateInputs(kLeftInputs);
        Logger.processInputs("Climb/LeftInputs", kLeftInputs);

        kRightIo.updateInputs(kRightInputs);
        Logger.processInputs("Climb/RightInputs", kRightInputs);

        if(!DriverStation.isEnabled()){
            stop();
        }

        if(goal != null){
            setVoltage(goal.getGoalVolts());
        }

        // Continuously check if climb has moved beyond its limitations, note
        // that we only need to compare the left voltage as that is the lead
        // motor
        if (getPosistion().getDegrees() > ClimbConstants.kMaxPosistionAngle.getDegrees() && (kLeftInputs.appliedVoltage > 0.0)) {
            stop();
        } else if (getPosistion().getDegrees() > ClimbConstants.kMinPosistionAngle.getDegrees() && (kLeftInputs.appliedVoltage < 0.0)) {
            stop();
        } else {
            // Do nothing if limits are not reached
        }
    }

    public void setGoal(ClimbGoal desiredGoal){
        goal = desiredGoal;
    }

    public void stop(){
        goal = null;

        if(RobotBase.isReal()){
            kLeftIo.stop();
        }

        else{
            kLeftIo.stop();
            kRightIo.stop();
        }
    }

    public Rotation2d getPosistion(){
        return kLeftInputs.relativePosistion;
    }

    public void setVoltage(double voltage){
        if(getPosistion().getRotations() > ClimbConstants.kMaxPosistionAngle.getRotations() && voltage > 0){
            return;
        }

        else if(getPosistion().getRotations() < ClimbConstants.kMinPosistionAngle.getRotations() && voltage < 0){
            return;
        }

        else{
            if(RobotBase.isReal()){
                kLeftIo.setVoltage(voltage);
            }

            else{
                kLeftIo.setVoltage(voltage);
                kRightIo.setVoltage(voltage);
            }
        }
    }
}

