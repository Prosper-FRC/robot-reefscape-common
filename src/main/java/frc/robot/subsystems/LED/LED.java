package frc.robot.subsystems.LED;

import java.util.regex.Pattern;

import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase implements ILED {

    AddressableLED m_led;
    AddressableLEDBuffer m_ledBuffer;
    LEDPattern pattern = LEDPattern.kOff;
    boolean isEnabled = false;

    public LED(int id,int bufferLength){
        m_led = new AddressableLED(id);
        m_ledBuffer = new AddressableLEDBuffer(bufferLength);
        m_led.setLength(m_ledBuffer.getLength());
        setSolidColor(0, 0, 0);
        m_led.start();
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
    public void setGradientAnimation(GradientType type, Color... colors){
        pattern = LEDPattern.gradient(type, colors);
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
        // If the LEDs are set to enabled then the LEDs will update periodically
        pattern.applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);
    }

}
