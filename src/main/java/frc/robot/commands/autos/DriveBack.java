// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.Drive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Orchestrator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveBack extends SequentialCommandGroup
{
  /** Creates a new DriveBack. */
  public DriveBack(Orchestrator orchestrator, Drivetrain drivetrain)
  {
    addCommands(new Drive(drivetrain, () -> -0.2, () -> 0.0, () -> 0.0).withTimeout(1));
  }
}
