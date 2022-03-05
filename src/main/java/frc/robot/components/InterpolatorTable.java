// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.components;

import java.util.Map;
import java.util.Set;

/**
 * A component for generating output values based on a set of input-output pairs,
 * linearly interpolating between values and optionally either extrapolating past
 * the domain of input points or clamping between the minimum or maximum.
 */

public class InterpolatorTable {
    private InterpolatorTableEntry[] entries;

    private double minimumX = 0, maximumX = 0;

    /**
     * Constructs an InterpolatorTable object.
     * Optionally allows for extrapolation past the lower and upper bounds
     * (it clamps the input between the minimum and maximum otherwise)
     * @param entries_ The array of entries
     */
    public InterpolatorTable(InterpolatorTableEntry... entries_) {
        entries = entries_;
        if (entries.length > 0) {
            double lowestX = entries[0].x;
            double highestX = entries[0].x;
            for (InterpolatorTableEntry entry : entries) {
                if (entry.x < lowestX) {
                    lowestX = entry.x;
                }
                if (entry.x > highestX) {
                    highestX = entry.x;
                }
            }
            minimumX = lowestX;
            maximumX = highestX;
        }
    }

    /**
     * Gets minimum of X domain.
     * @return The minimum X value of any entry.
     */
    public double getMinimumX() {
        return minimumX;
    }

    /**
     * Gets minimum of X domain.
     * @return The minimum X value of any entry.
     */
    public double getMaximumX() {
        return maximumX;
    }

    public double getValue(double input) {
        int minIdx = -1;
        int maxIdx = -1;
        double minDistance = 1000000000;
        double maxDistance = -1000000000;
        for (int i = 0; i < entries.length; i++) {
            double value = entries[i].x;
            double distance = value - input;
            if (distance < 0 && Math.abs(distance) < Math.abs(minDistance)) {
                minIdx = i;
                minDistance = distance;
            } else if (distance >= 0 && Math.abs(distance) < Math.abs(maxDistance)) {
                maxIdx = i;
                maxDistance = distance;
            }
        }
        if (minIdx == -1) {
            return entries[maxIdx].y;
        } else if (maxIdx == -1) {
            return entries[minIdx].y;
        }
        double interpolationValue = (input - entries[minIdx].x) / (entries[maxIdx].x - entries[minIdx].x);
        return entries[minIdx].y + (entries[maxIdx].y - entries[minIdx].y) * interpolationValue;
    }

    /**
     * Simple struct for entries in InterpolatorTable.
     */
    public static class InterpolatorTableEntry {
        double x = 0;
        double y = 0;

        public InterpolatorTableEntry(double x_, double y_) {
            x = x_;
            y = y_;
        }
    }
}