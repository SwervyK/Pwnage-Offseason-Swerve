package com.pwnagerobotics.lib.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ColorFactory {

    public static Color fromRGB(double r, double g, double b) {
        int newR = (int)(r * 255.0);
        int newG = (int)(g * 255.0);
        int newB = (int)(b * 255.0);
        Color8Bit newColor = new Color8Bit(newR, newG, newB);
        return new Color(newColor);
    }


    /**
     * @param hue the hue of the color
     * @param saturation the saturation of the color
     * @param value the value of the color
     * @return a Color instance
     */
    public static Color fromHSV(float hue, float saturation, float value) {

        int h = (int)(hue * 6);
        float f = hue * 6 - h;
        float p = value * (1 - saturation);
        float q = value * (1 - f * saturation);
        float t = value * (1 - (1 - f) * saturation);
    
        switch (h) {
            case 0: return fromRGB(value, t, p);
            case 1: return fromRGB(q, value, p);
            case 2: return fromRGB(p, value, t);
            case 3: return fromRGB(p, q, value);
            case 4: return fromRGB(t, p, value);
            case 5: return fromRGB(value, p, q);
            default: return null;
        }
    }
}
