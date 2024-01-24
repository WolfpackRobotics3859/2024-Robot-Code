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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase
{
  private final TalonFX m_ShooterMotor1 = new TalonFX(Hardware.SHOOTER_MOTOR_1_ID);
  private final TalonFX m_ShooterMotor2 = new TalonFX(Hardware.SHOOTER_MOTOR_2_ID);
  private final TalonFX m_WristMotor = new TalonFX(Hardware.WRIST_MOTOR_ID);
  private final TalonFX m_FeederMotor = new TalonFX(Hardware.FEEDER_MOTOR_ID);
  private final CANcoder m_WristCANCoder = new CANcoder(Hardware.SHOOTER_WRIST_CANCODER_ID);

  private final Timer m_timer;

  private double m_GoalPosition;
  private double m_BottomLimit;
  private double m_TopLimit;

  /**
   * @brief Creates a new Shooter subsystem.
  */
  public Shooter() 
  {
    // Motor configuration
    m_ShooterMotor1.getConfigurator().apply(ShooterConstants.SHOOTER_MOTOR_1_CONFIGURATION);
    m_ShooterMotor2.getConfigurator().apply(ShooterConstants.SHOOTER_MOTOR_2_CONFIGURATION);
    m_WristMotor.getConfigurator().apply(ShooterConstants.WRIST_MOTOR_CONFIGURATION);
    m_FeederMotor.getConfigurator().apply(ShooterConstants.FEEDER_GAINS);

    // CANCoder configuration
    m_WristCANCoder.getConfigurator().apply(ShooterConstants.WRIST_CANCODER_CONFIGURATION);

    this.m_timer = new Timer();
    m_timer.start();

    m_BottomLimit = 0;
    m_TopLimit = 0;
    m_GoalPosition = 0;
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
      case FEEDER_MOTOR:
        m_FeederMotor.setControl(request);
        break;
      default:
        DutyCycleOut defaultRequest = new DutyCycleOut(0, false, false, false, false);
        m_ShooterMotor1.setControl(defaultRequest);
        m_ShooterMotor2.setControl(defaultRequest);
        m_WristMotor.setControl(defaultRequest);
        m_FeederMotor.setControl(defaultRequest);
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
      case FEEDER_MOTOR:
        m_FeederMotor.setControl(request);
        break;
      default:
        DutyCycleOut defaultRequest = new DutyCycleOut(0, false, false, false, false);
        m_ShooterMotor1.setControl(defaultRequest);
        m_ShooterMotor2.setControl(defaultRequest);
        m_WristMotor.setControl(defaultRequest);
        m_FeederMotor.setControl(defaultRequest);
        break;
    }
  }

  // Wrist Functions

  /**
   * @brief Applies a Position Duty Cycle request to the wrist motor with the set position.
   * @param position The desired position to spin the motor toward, measured in rotations.
  */
  public void setWristPosition(double position)
  {
    MotionMagicVoltage request = new MotionMagicVoltage(position, false, ShooterConstants.SHOOTER_WRIST_FEED_FORWARD, 0, false, false, false);
    m_WristMotor.setControl(request);
  }

  public void setWristTopLimit(double limit)
  {
    m_TopLimit = limit;
  }

  public void setWristBottomLimit(double limit)
  {
    m_BottomLimit = limit;
  }

  public void setGoalPosition(double position)
  {
    m_GoalPosition = position;
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
      case FEEDER_MOTOR:
        return m_FeederMotor.getVelocity();
      default:
        return m_ShooterMotor1.getVelocity();
    }
  }

  /**
   * @brief Gets the position of the wrist motor.
   * @return Returns a Status Signal of the current position.
  */
  public StatusSignal<Double> getWristMotorPosition()
  {
    return m_WristCANCoder.getAbsolutePosition();
  }

  public boolean getWristWithinLimits()
  {
    final double position = m_WristMotor.getPosition().getValueAsDouble();

    if (position < m_BottomLimit || position > m_TopLimit)
    {
      return false;
    }
    else
    {
      return true;
    }
  }

  @Override
  public void periodic()
  {
    if (m_timer.get() > 0.5)
    {
      m_timer.reset();
      SmartDashboard.putNumber("Shooter Wrist position", this.getWristMotorPosition().getValue());
      SmartDashboard.putNumber("Top Limit", m_TopLimit);
      SmartDashboard.putNumber("goal pos", m_GoalPosition);
    }
  }
}
