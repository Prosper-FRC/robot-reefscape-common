package frc.robot.subsystems.LED;

public class LEDConstants {

    public record LEDConfig(
        int port,
        int bufferLength) {}

    public static final LEDConfig kLEDConfig = new LEDConfig(9, 67);
}
