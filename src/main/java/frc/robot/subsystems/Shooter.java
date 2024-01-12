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

    /**
   * @brief Creates a new Shooter subsystem.
  */
  public Shooter() 
  {
    m_ShooterMotor1.getConfigurator().apply(ShooterConstants.SHOOTER_1_GAINS);
    m_ShooterMotor2.getConfigurator().apply(ShooterConstants.SHOOTER_2_GAINS);
    m_WristMotor.getConfigurator().apply(ShooterConstants.WRIST_GAINS);
  }

  // Shooter Functions

  /**
   * @brief Applies a Motion Magic request to motor 1 with the set velocity.
   * @param velocity The desired velocity to run the motor at, measured in rotations per second.
  */
  public void setMotor1Velocity(double velocity)
  {
    MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(velocity, 40, false, 0, 0, false, false, false);
    m_ShooterMotor1.setControl(request);
  }

  /**
   * @brief Applies a Motion Magic Velocity request to motor 2 with the set velocity.
   * @param velocity The desired velocity to run the motor at, measured in rotations per second.
  */
  public void setMotor2Velocity(double velocity)
  {
    MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(velocity, 40, false, 0, 0, false, false, false);
    m_ShooterMotor2.setControl(request);
  }

  /**
   * @brief Applies a duty cycle to motor 1 with the set percentage of the available voltage.
   * @param percent The percent of the available voltage to apply to the motor, from -1 to 1.
  */
  public void setShooterMotor1Percent(double percent)
  {
    DutyCycleOut request = new DutyCycleOut(percent, false, false, false, false);
    m_ShooterMotor1.setControl(request);
  }

  /**
   * @brief Applies a duty cycle to motor 2 with the set percentage of the available voltage.
   * @param percent The percent of the available voltage to apply to the motor, from -1 to 1.
  */
  public void setShooterMotor2Percent(double percent)
  {
    DutyCycleOut request = new DutyCycleOut(percent, false, false, false, false);
    m_ShooterMotor2.setControl(request);
  }

  // Wrist Functions

  /**
   * @brief Applies a Position Duty Cycle request to the wrist motor with the set position.
   * @param position The desired position to spin the motor toward, measured in rotations.
  */
  public void setWristPos(double position)
  {
    PositionDutyCycle request = new PositionDutyCycle(position, 4, false, 0, 0, false, false, false);
    m_WristMotor.setControl(request);
  }

  /**
   * @brief Applies a duty cycle to the wrist motor with the set percentage of the available voltage.
   * @param percent The percent of the available voltage to apply to the motor, from -1 to 1.
  */
  public void setWristPercent(double percent)
  {
    DutyCycleOut request = new DutyCycleOut(percent, false, false, false, false);
    m_WristMotor.setControl(request);
  }

  // Telemetry Functions

  /**
   * @brief Gets the velocity of shooter motor 1.
   * @return Returns a Status Signal of the current velocity.
  */
  public StatusSignal<Double> getShooterMotor1Velocity()
  {
    return m_ShooterMotor1.getVelocity();
  }

  /**
   * @brief Gets the velocity of shooter motor 2.
   * @return Returns a Status Signal of the current velocity.
  */
  public StatusSignal<Double> getShooterMotor2Velocity()
  {
    return m_ShooterMotor2.getVelocity();
  }

  /**
   * @brief Gets the position of the wrist motor.
   * @return Returns a Status Signal of the current position.
  */
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
