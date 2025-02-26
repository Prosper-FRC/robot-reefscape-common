package frc.robot.subsystems.LED;
import edu.wpi.first.wpilibj.util.Color;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.LEDPattern.GradientType;

interface ILED{
    public void setSingleRGB(int index, int r, int g, int b);

    public void setSolidColor(int r, int g, int b);

    public void setRainbowAnimation(int saturation, int brightness);

    public void setGradientAnimation(int percentage, GradientType type, Color... colors);

    public void setBlinkAnimation(double blinkOnRate, double blinkOffRate);
    
    public void setBlinkAnimation(double blinkRate);

    public void setBreatheAnimation(double timePeriod);

    public void setBrightness(double percent);

    public void animatePatternRelative(int percentage);

    public void stopAnimatingPattern();

    public void disable();
}