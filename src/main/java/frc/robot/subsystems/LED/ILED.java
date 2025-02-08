package frc.robot.subsystems.LED;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;

interface ILED{

    public void setSingleRGB(int index, int r, int g, int b);

    //RGB values bust be between 0-255 inclusive
    public void setSolidColor(int r, int g, int b);

    public void setRainbowAnimation(int saturation, int brightness);

    public void setGradientAnimation(GradientType type, Color... colors);

    public void animatePatternRelative(int percentage);

    public void stopAnimatingPattern();

    public void disable();
}