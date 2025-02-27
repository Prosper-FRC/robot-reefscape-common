package frc.robot.subsystems.LED;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LED.LEDConstants.LEDConfig;

public class LED extends SubsystemBase implements ILED {

    AddressableLED m_led;
    AddressableLEDBuffer m_ledBuffer;
    LEDPattern pattern = LEDPattern.kOff;
    private Color m_EyeColor = Color.kGreen;
    private Color m_BackgroundColor = Color.kPurple;
    private int m_eyePosition = 0;
    private int m_scanDirection = 1;

    public LED(LEDConfig configuration){
        m_led = new AddressableLED(configuration.port());
        m_ledBuffer = new AddressableLEDBuffer(configuration.bufferLength());
        m_led.setLength(m_ledBuffer.getLength());
        setSolidColor(0, 0, 0);
        m_led.start();
    }

    public void setScanner() {
        int bufferLength = m_ledBuffer.getLength();
        double intensity;
        double red;
        double green;
        double blue;
        double distanceFromEye;

        for (int index = 0; index < bufferLength; index++) {
            distanceFromEye = MathUtil.clamp(Math.abs(m_eyePosition - index), 0, 2);
            intensity = 1.0; //1 - distanceFromEye / 2;
            red = MathUtil.interpolate(m_BackgroundColor.red, m_EyeColor.red, intensity);
            green = MathUtil.interpolate(m_BackgroundColor.green, m_EyeColor.green, intensity);
            blue = MathUtil.interpolate(m_BackgroundColor.blue, m_EyeColor.blue, intensity);

            m_ledBuffer.setLED(index, new Color(red, green, blue));
        }

        if (m_eyePosition == 0) {
            m_scanDirection = 1;
        } else if (m_eyePosition == bufferLength - 1) {
         m_scanDirection = -1;
        }

        m_eyePosition += m_scanDirection;
  }

    // Low level method, use only for custom LED animations
    public void setSingleRGB(int index, int r, int g, int b){
        m_ledBuffer.setRGB(index,r,g,b);
    }

    // Sets solid LED color
    public void setSolidColor(int r, int g, int b){
        pattern = LEDPattern.solid(new Color(r, g, b));
    }

    // Sets rainbow LED color
    public void setRainbowAnimation(int saturation, int brightness){
        pattern = LEDPattern.rainbow(saturation, brightness);
    }

    // There are two types of gradients
    // kContinuous (When the gradient moves it loops seamlessly)
    // kDiscontinuous (this should not be animated to move)
    public void setGradientAnimation(int percentage, GradientType type, Color... colors){
        pattern = LEDPattern.gradient(type, colors);
        animatePatternRelative(percentage);
    }

    // Sets a blink rate by a on and off blink offset
    public void setBlinkAnimation(double blinkOnRate, double blinkOffRate){
        pattern = pattern.blink(Time.ofBaseUnits(blinkOnRate, Units.Seconds), Time.ofBaseUnits(blinkOffRate, Units.Seconds));
    }

    // Sets a blink rate in blinks per second
    public void setBlinkAnimation(double blinkRate){
        pattern = pattern.blink(Time.ofBaseUnits(blinkRate, Units.Seconds));
    }

    // Fades the LEDs in and out
    public void setBreatheAnimation(double timePeriod, Color color){
        pattern = LEDPattern.solid(color);
        pattern = pattern.breathe(Time.ofBaseUnits(timePeriod, Units.Seconds));
    }

    public void setSolidBlinkAnimation(double blinkRate, Color color){
        pattern = LEDPattern.solid(color);
        pattern = pattern.blink(Time.ofBaseUnits(blinkRate, Units.Seconds));
    }

    // Sets the brightness of the LED strip
    public void setBrightness(double percent){
        pattern = pattern.atBrightness(Units.Percent.of(percent));
    }

    // Percentage is the percent of the LED strip covers per second
    public void animatePatternRelative(int percentage){
        pattern = pattern.scrollAtRelativeSpeed(Units.Percent.per(Units.Second).of(percentage));
    }

    // Stops animating the LEDs (LEDs stop scrolling)
    public void stopAnimatingPattern(){
        pattern = pattern.scrollAtRelativeSpeed(Units.Percent.per(Units.Second).of(0));
    }

    // Turns LEDs off (Also pauses animation progression)
    public void disable(){
        pattern = LEDPattern.kOff;
    }

    @Override
    public void periodic() {
        pattern.applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);
    }

}