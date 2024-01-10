// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SetVelocity extends Command {
  private Shooter m_Shooter;
  private double m_Velocity;
  private int m_Motor;

  /** Creates a new SetVelocity. */
  public SetVelocity(Shooter shooter, double velocity, int motor) {
    this.m_Shooter = shooter;
    this.m_Velocity = velocity;
    this.m_Motor = motor;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (m_Motor) {
      case 1:
        m_Shooter.setMotor1Velocity(m_Velocity);
        break;
      case 2:
        m_Shooter.setMotor2Velocity(m_Velocity);
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
