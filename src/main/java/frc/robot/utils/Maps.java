// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.Map;
import java.util.TreeMap;

/** Contains tables for positions between shooter and elevator */
public class Maps
{
    /** The map of the maximum shooter position based on the given elevator position */
    public static final TreeMap<Double, Double> ElevatorToShooter = new TreeMap<>
    (
        Map.ofEntries
        (
            Map.entry(12.2, 14.3)
        )
    );

    /** The map of the maximum elevator position based on the given shooter position */
    public static final TreeMap<Double, Double> ShooterToElevator = new TreeMap<>
    (
        Map.ofEntries
        (
            Map.entry(12.2, 14.3)
        )
    );
}
