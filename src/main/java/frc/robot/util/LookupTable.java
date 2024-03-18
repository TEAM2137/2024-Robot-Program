package frc.robot.util;

import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;

/**
 * A table of keys and values that can get an interpolated result between data points.
 */
public class LookupTable {
    private NavigableMap<Double, Double> dataPoints = new TreeMap<Double, Double>(); // First value is distance, second is angle

    /**
     * Creates a new LookupTable using the provided map
     * @param map A map of keys and values to act as the data points
     */
    public LookupTable(Map<Double, Double> map) {
        dataPoints.putAll(map);
    }

    /**
     * Gets an interpolated value based on a key.
     * @param key Key to the requested value.
     * @return Value.
     */
    public double getInterpolated(double key) {
        if(dataPoints.containsKey(key)) {
            return dataPoints.get(key);
        }else{
            Double higherKey = dataPoints.ceilingKey(key);
            Double lowerKey = dataPoints.floorKey(key);

            if (higherKey == null) { // No idea why it won't just let me use == null
                higherKey = lowerKey;
                lowerKey = dataPoints.lowerKey(higherKey); // Use the last two known points to extrapolate
            }

            if (lowerKey == null) {
                lowerKey = higherKey;
                higherKey = dataPoints.higherKey(lowerKey); // Ditto
            }

            double slope = (dataPoints.get(higherKey) - dataPoints.get(lowerKey)) / (higherKey - lowerKey);

            double intercept = dataPoints.get(lowerKey) - slope * lowerKey;

            return (slope * key) + intercept; // y=mx+b
        }
    }
}