package frc.robot.util;

public class LEDColor {

    public static final LEDColor NONE = new LEDColor(0.0f, 0.0f, 0.0f);
    public static final LEDColor WHITE = new LEDColor(1.0f, 1.0f, 1.0f);

    public static final LEDColor RED = new LEDColor(1.0f, 0.0f, 0.0f);
    public static final LEDColor GREEN = new LEDColor(0.0f, 1.0f, 0.0f);
    public static final LEDColor BLUE = new LEDColor(0.0f, 0.0f, 1.0f);

    public static final LEDColor YELLOW = new LEDColor(1.0f, 0.8f, 0.0f);
    public static final LEDColor ORANGE = new LEDColor(1.0f, 0.4f, 0.0f);

    private final float r;
    private final float g;
    private final float b;

    /**
     * @param r Red component, 0-1
     * @param g Green component, 0-1
     * @param b Blue component, 0-1 
     */
    public LEDColor(float r, float g, float b) {
        this.r = r;
        this.g = g;
        this.b = b;
    }

    public float getR() {
        return r;
    }

    public float getG() {
        return g;
    }

    public float getB() {
        return b;
    }
}
