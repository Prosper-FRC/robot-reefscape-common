package frc.robot.subsystems.climb;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
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

    private final ClimbIO io;
    private final ClimbIOInputsAutoLogged kInputs = new ClimbIOInputsAutoLogged();

    public Climb(ClimbIO io){
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(kInputs);
        Logger.processInputs("Climb/Inputs", kInputs);

        if(!DriverStation.isEnabled()){
            stop();
        }

        if(goal != null){
            setVoltage(goal.getGoalVolts());
        }
    }

    public void setGoal(ClimbGoal desiredGoal){
        goal = desiredGoal;
    }

    public void stop(){
        goal = null;
        io.stop();
    }

    public Rotation2d getPosistion(){
        return kInputs.relativePosistion;
    }

    public void setVoltage(double voltage){
        if(getPosistion().getRotations() > ClimbConstants.kMaxPosistionAngle.getRotations() && voltage > 0){
            return;
        }

        else if(getPosistion().getRotations() < ClimbConstants.kMinPosistionAngle.getRotations() && voltage < 0){
            return;
        }

        else{
            io.setVoltage(voltage);
        }
    }
}

