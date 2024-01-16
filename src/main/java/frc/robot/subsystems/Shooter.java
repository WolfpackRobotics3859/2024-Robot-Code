// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.constants.Hardware;
import frc.robot.constants.shooter.ShooterConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase
{
  private final TalonFX m_ShooterMotor1 = new TalonFX(Hardware.SHOOTER_MOTOR_1_ID);
  private final TalonFX m_ShooterMotor2 = new TalonFX(Hardware.SHOOTER_MOTOR_2_ID);
  private final TalonFX m_WristMotor = new TalonFX(Hardware.WRIST_MOTOR_ID);
  private final CANcoder m_WristCANCoder = new CANcoder(Hardware.SHOOTER_CANCODER_ID);

  /**
   * @brief Creates a new Shooter subsystem.
  */
  public Shooter() 
  {
    m_ShooterMotor1.getConfigurator().apply(ShooterConstants.SHOOTER_1_GAINS);
    m_ShooterMotor2.getConfigurator().apply(ShooterConstants.SHOOTER_2_GAINS);
    m_WristMotor.getConfigurator().apply(ShooterConstants.WRIST_GAINS);

    m_WristCANCoder.getConfigurator().apply(ShooterConstants.MAGNET_SENSOR_CONFIGS);

    m_WristMotor.getConfigurator().apply(ShooterConstants.MOTION_MAGIC_CONFIGS);
    m_WristMotor.getConfigurator().apply(ShooterConstants.BRAKE_CONFIG);
    m_WristMotor.getConfigurator().apply(ShooterConstants.FEEDBACK_CONFIGS);
    m_WristMotor.getConfigurator().apply(ShooterConstants.SOFT_LIMIT_CONFIGS);
  }

  // Shooter Functions

  /**
   * @brief Applies a Motion Magic request to the selected motor with the set velocity.
   * @param motor The motor you want to send the request to MOTOR_1 or MOTOR_2, setting this to WRIST_MOTOR will do nothing.
   * @param velocity The desired velocity to run the motor at, measured in rotations per second.
  */
  public void setMotorVelocity(ShooterConstants.MOTOR motor, double velocity)
  {
    MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(velocity, ShooterConstants.SHOOTER_MOTOR_ACCELERATION, false, 0, 0, false, false, false);

    switch (motor)
    {
      case MOTOR_1:
        m_ShooterMotor1.setControl(request);
        break;
      case MOTOR_2:
        m_ShooterMotor2.setControl(request);
        break;
      case WRIST_MOTOR:
        break;
      default:
        DutyCycleOut defaultRequest = new DutyCycleOut(0, false, false, false, false);
        m_ShooterMotor1.setControl(defaultRequest);
        m_ShooterMotor2.setControl(defaultRequest);
        m_WristMotor.setControl(defaultRequest);
        break;
    }
  }

  /**
   * @brief Applies a duty cycle to the given motor with the set percentage of the available voltage.
   * @param motor The desired motor to send the request to.
   * @param percent The percent of the available voltage to apply to the motor, from -1 to 1.
  */
  public void setShooterMotorPercent(ShooterConstants.MOTOR motor, double percent)
  {
    DutyCycleOut request = new DutyCycleOut(percent, false, false, false, false);

    switch (motor){
      case MOTOR_1:
        m_ShooterMotor1.setControl(request);
        break;
      case MOTOR_2:
        m_ShooterMotor2.setControl(request);
        break;
      case WRIST_MOTOR:
        m_WristMotor.setControl(request);
        break;
      default:
        DutyCycleOut defaultRequest = new DutyCycleOut(0, false, false, false, false);
        m_ShooterMotor1.setControl(defaultRequest);
        m_ShooterMotor2.setControl(defaultRequest);
        m_WristMotor.setControl(defaultRequest);
        break;
    }
  }

  // Wrist Functions

  /**
   * @brief Applies a Position Duty Cycle request to the wrist motor with the set position.
   * @param position The desired position to spin the motor toward, measured in rotations.
  */
  public void setWristPos(double position)
  {
    MotionMagicVoltage request = new MotionMagicVoltage(position, false, ShooterConstants.SHOOTER_WRIST_FEED_FORWARD, 0, false, false, false);
    m_WristMotor.setControl(request);
  }

  // Telemetry Functions

  /**
   * @brief Gets the velocity of a motor in the shooter subsystem.
   * @param motor The desired motor to get the velocity from.
   * @return Returns a Status Signal of the current velocity.
  */
  public StatusSignal<Double> getShooterMotorVelocity(ShooterConstants.MOTOR motor)
  {
    switch (motor){
      case MOTOR_1:
        return m_ShooterMotor1.getVelocity();
      case MOTOR_2:
        return m_ShooterMotor2.getVelocity();
      case WRIST_MOTOR:
        return m_WristMotor.getVelocity();
      default:
        return m_ShooterMotor1.getVelocity();
    }
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
