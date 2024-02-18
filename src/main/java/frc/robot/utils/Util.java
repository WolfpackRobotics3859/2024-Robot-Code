// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/** A class to store various basic functions that are used across the entire robot */
public class Util
{
    /**
     * Checks if a value is at a set value given a tolerance
     * @param a The value to check
     * @param b The desired value for a to be at
     * @param epsilon The tolerance of the check
     * @return True if the A value is close to the B value based on the epsilon (tolerance)
     */
    public static boolean epsilonEquals(double a, double b, double epsilon)
    {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }
    
    /**
     * Finds if the given value is between the given min and max
     * @param value The value to compare
     * @param min The minimum value of the range
     * @param max The value of the range
     * @return Whether or on the given value is within the given range
     */
    public static boolean inRange(double value, double min, double max)
    {
        return value > min && value < max;
    }
}
