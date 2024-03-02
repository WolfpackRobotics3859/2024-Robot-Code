// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.Drive;
import frc.robot.commands.drivetrain.SeedFieldRelativeWithAngle;
import frc.robot.commands.orchestrator.BumperShot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Orchestrator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootAndDrive extends SequentialCommandGroup
{
  /** Creates a new BasicAuto. */
  public ShootAndDrive(Orchestrator orchestrator, Drivetrain drivetrain)
  {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new BumperShot(orchestrator, false).withTimeout(5), new Drive(drivetrain, () -> -0.2, () -> 0.0, () -> 0.0).withTimeout(0.5), new SeedFieldRelativeWithAngle(drivetrain, 180));
  }
}
