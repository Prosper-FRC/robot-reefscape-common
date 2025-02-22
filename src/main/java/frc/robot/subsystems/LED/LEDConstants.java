package frc.robot.subsystems.LED;

public class LEDConstants {

    public record LEDConfig(
        int port,
        int bufferLength) {}
    
    public static final LEDConfig kLeftLED = new LEDConfig(9, 33);
    public static final LEDConfig kRightLED = new LEDConfig(8, 33);
}
