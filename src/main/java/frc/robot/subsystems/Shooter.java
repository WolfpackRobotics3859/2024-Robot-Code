// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase
{
  private final TalonFX m_shooterMotor1 = new TalonFX(9);
  private final TalonFX m_shooterMotor2 = new TalonFX(10);

  /** Creates a new Shooter. */
  public Shooter() 
  {
    // Intentionally Empty
  }

  public void setMotor1Velocity(double rpm)
  {
    MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(rpm, 1, false, 0, 1, false, false, false);
    m_shooterMotor1.setControl(request);
  }

  public void setMotor2Velocity(double rpm)
  {
    MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(rpm, 1, false, 0, 1, false, false, false);
    m_shooterMotor2.setControl(request);
  }

  @Override
  public void periodic() 
  {
    // Intentionally Empty
  }
}
