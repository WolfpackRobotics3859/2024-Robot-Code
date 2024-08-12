// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.climb.ClimbConstants;

public class Climb extends SubsystemBase
{
  private final TalonFX m_ClimbMotor = new TalonFX(Ports.CLIMB_MOTOR_ID);

  public Climb()
  {
    m_ClimbMotor.getConfigurator().apply(ClimbConstants.CLIMB_MOTOR_CONFIGURATION);
  }

  @Override
  public void periodic()
  {
    // Empty for now
  }
}
