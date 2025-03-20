package frc.robot.subsystems.LED;

public class LEDConstants {

    public record LEDConfig(
        int port,
        int bufferLength) {}
    
    public static final LEDConfig kLED = new LEDConfig(8, 70);
}