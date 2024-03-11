// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.shuffletabs;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.utils.Tabs;

/** Add your docs here. */
public class Subsystems extends Tabs
{
    @Override
    public void createEntries()
    {
        tab = Shuffleboard.getTab("Subsystems");
    }

    @Override
    public void update()
    {
        // Intentionally Empty
    }
}
