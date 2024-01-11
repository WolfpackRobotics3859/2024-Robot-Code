// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.constants.Hardware;
import frc.robot.constants.shooter.ShooterConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase
{
  private final TalonFX m_ShooterMotor1 = new TalonFX(Hardware.SHOOTER_MOTOR_1_ID);
  private final TalonFX m_ShooterMotor2 = new TalonFX(Hardware.SHOOTER_MOTOR_2_ID);
  private final TalonFX m_WristMotor = new TalonFX(Hardware.WRIST_MOTOR_ID);

  /** Creates a new Shooter. */
  public Shooter() 
  {
    m_ShooterMotor1.getConfigurator().apply(ShooterConstants.SHOOTER_GAINS);
    m_ShooterMotor2.getConfigurator().apply(ShooterConstants.SHOOTER_GAINS);
    m_WristMotor.getConfigurator().apply(ShooterConstants.WRIST_GAINS);
  }

  // Shooter Functions
  public void setMotor1Velocity(double rpm)
  {
    MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(rpm, 40, false, 70, 0, false, false, false);
    m_ShooterMotor1.setControl(request);
  }

  public void setMotor2Velocity(double rpm)
  {
    MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(rpm, 1, false, 0, 0, false, false, false);
    m_ShooterMotor2.setControl(request);
  }

  public void setShooterMotor1Percent(double percent)
  {
    DutyCycleOut request = new DutyCycleOut(percent, false, false, false, false);
    m_ShooterMotor1.setControl(request);
  }

  public void setShooterMotor2Percent(double percent)
  {
    DutyCycleOut request = new DutyCycleOut(percent, false, false, false, false);
    m_ShooterMotor2.setControl(request);
  }

  // Wrist Functions
  public void setWristPos(double position)
  {
    PositionDutyCycle request = new PositionDutyCycle(position, 4, false, 0, 0, false, false, false);
    m_WristMotor.setControl(request);
  }

  public void setWristPercent(double percent)
  {
    DutyCycleOut request = new DutyCycleOut(percent, false, false, false, false);
    m_WristMotor.setControl(request);
  }

  // Telemetry Functions
  public StatusSignal<Double> getShooterMotor1Velocity()
  {
    return m_ShooterMotor1.getVelocity();
  }

  public StatusSignal<Double> getShooterMotor2Velocity()
  {
    return m_ShooterMotor2.getVelocity();
  }

  public StatusSignal<Double> getWristMotorPos()
  {
    return m_ShooterMotor1.getPosition();
  }

  @Override
  public void periodic()
  {
    // Intentionally Empty
  }
}
