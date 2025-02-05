package subsystems.LED;

interface ILED{

    public void setSingleRGB(int index, int r, int g, int b);

    //RGB values bust be between 0-255 inclusive
    public void setRGB(int r, int g, int b);

    public void off();
}