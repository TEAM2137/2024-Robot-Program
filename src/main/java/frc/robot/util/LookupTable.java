package frc.robot.util;

import java.util.Map;
import java.util.NavigableMap;
import java.util.Objects;
import java.util.TreeMap;

public class LookupTable {
    private NavigableMap<Double, Double> dataPoints = new TreeMap<Double, Double>(); // First value is distance, second is angle

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
            double higherKey = dataPoints.ceilingKey(key);
            double lowerKey = dataPoints.floorKey(key);

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

            return (slope * key) + intercept; // y=mx+b
        }
    }
}
