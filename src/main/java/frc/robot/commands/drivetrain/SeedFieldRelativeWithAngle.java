// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

public class SeedFieldRelativeWithAngle extends InstantCommand
{
  private final Drivetrain m_Drivetrain;
  private final double m_Angle;

  public SeedFieldRelativeWithAngle(Drivetrain drivetrain, double angle)
  {
    this.m_Drivetrain = drivetrain;
    this.m_Angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_Drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(this.m_Angle)));
  }
}
