// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class SetIntakeRollersPercent extends Command
 {
  /** Creates a new RunIntakeCommand. */

  private IntakeSubsystem m_intakeSubsystem;
  double m_Percent;

  public SetIntakeRollersPercent(IntakeSubsystem intake, double percent)
  {
    this.m_intakeSubsystem = intake;
    this.m_Percent = percent;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_intakeSubsystem.setIntakePercent(this.m_Percent);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    //Intentionally empty
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    //Intentionally empty
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
