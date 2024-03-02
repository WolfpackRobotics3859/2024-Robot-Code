// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.orchestrator.BumperShot;
import frc.robot.subsystems.Orchestrator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Shoot extends SequentialCommandGroup
{
  /** Creates a new Shoot. */
  public Shoot(Orchestrator orchestrator)
  {
    addCommands(new BumperShot(orchestrator, false).withTimeout(5));
  }
}
