package frc.robot.subsystems.LED;
import edu.wpi.first.wpilibj.util.Color;
<<<<<<< HEAD
import edu.wpi.first.wpilibj.LEDPattern.GradientType;

interface ILED{

    public void setSingleRGB(int index, int r, int g, int b);

    //RGB values bust be between 0-255 inclusive
=======

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.LEDPattern.GradientType;

interface ILED{
    public void setSingleRGB(int index, int r, int g, int b);

>>>>>>> dev/merged
    public void setSolidColor(int r, int g, int b);

    public void setRainbowAnimation(int saturation, int brightness);

<<<<<<< HEAD
    public void setGradientAnimation(GradientType type, Color... colors);
=======
    public void setGradientAnimation(int percentage, GradientType type, Color... colors);

    public void setBlinkAnimation(double blinkOnRate, double blinkOffRate);
    
    public void setBlinkAnimation(double blinkRate);

    public void setBreatheAnimation(double timePeriod);

    public void setBrightness(double percent);
>>>>>>> dev/merged

    public void animatePatternRelative(int percentage);

    public void stopAnimatingPattern();

    public void disable();
}