// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Orchestrator;
import frc.robot.utils.shuffletabs.Auto;
import frc.robot.utils.shuffletabs.Driver;
import frc.robot.utils.shuffletabs.Subsystems;
import frc.robot.utils.shuffletabs.Vision;


/**  */
public class ShuffleboardManager
{
    private ArrayList<Tabs> tabs = new ArrayList<>();

    private final Vision m_VisionTab;
    private final Auto m_AutoTab;
    private final Driver m_DriverTab;
    private final Subsystems m_SubsystemsTab;

    public ShuffleboardManager(Orchestrator orchestrator, Drivetrain drivetrain)
    {
        m_VisionTab = new Vision();
        m_AutoTab =  new Auto(orchestrator, drivetrain);
        m_DriverTab = new Driver(drivetrain);
        m_SubsystemsTab = new Subsystems();

        tabs.add(m_VisionTab);
        tabs.add(m_AutoTab);
        tabs.add(m_DriverTab);
        tabs.add(m_SubsystemsTab);

        for (Tabs tab : tabs)
        {
            tab.createEntries();
        }
    }

    public void update()
    {
        for (Tabs tab : tabs){
            tab.update();
        }
        
        Shuffleboard.update();
    }

    public Command getAutoCommand()
    {
        return m_AutoTab.getChooser().getSelected();
    }
}
