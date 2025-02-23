package frc.robot.subsystems.LED;

public class LEDConstants {

    public record LEDConfig(
        int port,
        int bufferLength) {}
    
    public static final LEDConfig kLeftLED = new LEDConfig(8, 10);
    public static final LEDConfig kRightLED = new LEDConfig(9, 10);
}
