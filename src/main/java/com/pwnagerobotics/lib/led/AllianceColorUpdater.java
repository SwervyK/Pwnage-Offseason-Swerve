package com.pwnagerobotics.lib.led;

import java.util.HashMap;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


/**
 * updates instances of Color arrays with the color of the robots current alliance
 */
public class AllianceColorUpdater
{
    private Alliance mPreviousAlliance;

    /**
     * the variable where the Color arrays are stored along with which indexes to update
     */
    private HashMap<Color[], int[]> mColorArrays;

    public AllianceColorUpdater() {
        mColorArrays = new HashMap<Color[], int[]>(); // added generic types because compiler was warning. DL
    }

    public static Color getAllianceColor() {
        return getAllianceColor(DriverStation.getAlliance());
    }

    public static Color getAllianceColor(Alliance alliance) {
        switch(alliance) {
            case Red:
                return Color.kRed;
            case Blue:
                return Color.kBlue;
            default:
                return Color.kWhite;
        }
    }

    /**
     * adds the color array to the list of arrays to update when {@link #update()} is called
     * @param colors the color array to update
     * @param indexes the list of indexes in the Color array to update with the alliance Color
     * @return the array with updated colors
     */
    public Color[] update(Color[] colors, int[] indexes) {
        mColorArrays.put(colors, indexes);
        return updateArray(colors, indexes, DriverStation.getAlliance());
    }

    /**
     * updates all the arrays to the new alliance Color
     */
    public void update() {
        Alliance alliance = DriverStation.getAlliance();

        if(mPreviousAlliance == alliance)
            return;
        else
            mPreviousAlliance = alliance;

        mColorArrays.forEach((k,v) -> updateArray(k, v, alliance));
    }

    /**
     * updates an individual array without adding it to the list to update in the future
     * @param colors the color array to update
     * @param indexes the list of indexes in the Color array to update with the alliance Color
     * @return the array with updated colors
     */
    public Color[] updateArray(Color[] colors, int[] indexes, Alliance alliance) {
        Color color;

        switch(alliance)
        {
            case Blue:
                color = Color.kBlue;
                break;
            case Red:
                color = Color.kRed;
                break;
            default:
                color = Color.kWhite;
                break;
        }

        for(int i : indexes) {
            colors[i] = color;
        }

        return colors;
    }
}
