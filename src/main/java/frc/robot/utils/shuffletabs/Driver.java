// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.shuffletabs;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Tabs;

/** Add your docs here. */
public class Driver extends Tabs
{
    private final Field2d m_Field = new Field2d();
    private final Drivetrain m_Drivetrain;

    public Driver (Drivetrain drivetrain)
    {
        this.m_Drivetrain = drivetrain;
    }

    @Override
    public void createEntries()
    {
        tab = Shuffleboard.getTab("Driver");
        
        tab.add("Field", m_Field);
    }

    @Override
    public void update()
    {
        m_Field.setRobotPose(m_Drivetrain.getOdometry().getEstimatedPosition());
        Shuffleboard.update();
    }
}
