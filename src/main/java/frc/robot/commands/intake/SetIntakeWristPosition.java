// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class SetIntakeWristPosition extends Command 
{
  
  IntakeSubsystem m_Intake;
  double m_Position;

  /**
   * @brief Spins the wrist motor towards a given position.
   * @param shooter The shooter subsystem object.
   * @param velocity The desired position to send the motor towards, measured in rotations.
  */
  public SetIntakeWristPosition(IntakeSubsystem intake, double position)
  {
    this.m_Intake = intake;
    this.m_Position = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_Intake.setWristPosition(this.m_Position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    // Intentionally Empty
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    // Intentionally Empty
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
