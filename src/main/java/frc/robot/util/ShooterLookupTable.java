package frc.robot.util;

import java.util.NavigableMap;
import java.util.Objects;
import java.util.TreeMap;

public class ShooterLookupTable {
    private NavigableMap<Double, Double> dataPoints = new TreeMap<Double, Double>(); // First value is distance, second is angle

    public ShooterLookupTable() {
        dataPoints.put(1.0, 10.0); //TODO: actually plot values. This is only a placeholder
    }

    /**
     * Gets an interpolated angle from pre-recorded datapoints.
     * @param distance Distance in whatever unit you set the data in.
     * @return The shooter angle.
     */
    public double getAngleAt(double distance) {
        if(dataPoints.containsKey(distance)) {
            return dataPoints.get(distance);
        }else{
            double higherKey = dataPoints.ceilingKey(distance);
            double lowerKey = dataPoints.floorKey(distance);

            if(Objects.isNull(higherKey)) { // No idea why it won't just let me use == null
                higherKey = lowerKey;
                lowerKey = dataPoints.lowerKey(higherKey); // Use the last two known points to extrapolate
            }

            if(Objects.isNull(lowerKey)) {
                lowerKey = higherKey;
                higherKey = dataPoints.higherKey(lowerKey); // Ditto
            }

            double slope = (dataPoints.get(higherKey) - dataPoints.get(lowerKey)) / (higherKey - lowerKey);

            double intercept = dataPoints.get(lowerKey) - slope * lowerKey;

            return (slope * distance) + intercept; // y=mx+b
        }
    }
}
