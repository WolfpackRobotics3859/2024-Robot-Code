// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.Drive;
import frc.robot.commands.drivetrain.SeedFieldRelativeWithAngle;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Orchestrator;

public class DriveBack extends SequentialCommandGroup
{
  /** Creates a new DriveBack. */
  public DriveBack(Orchestrator orchestrator, Drivetrain drivetrain, Double seedAngle)
  {
    addCommands(new SeedFieldRelativeWithAngle(drivetrain, seedAngle), new Drive(drivetrain, () -> 0.2, () -> 0.0, () -> 0.0).withTimeout(1));
  }
}
