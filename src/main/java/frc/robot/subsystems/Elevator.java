// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Hardware;
import frc.robot.constants.elevator.ElevatorConstants;

public class Elevator extends SubsystemBase
{
  private final TalonFX m_ElevatorMotor1 = new TalonFX(Hardware.ELEVATOR_MOTOR_1_ID);
  private final TalonFX m_ElevatorMotor2 = new TalonFX(Hardware.ELEVATOR_MOTOR_2_ID);
  private final CANcoder m_CANCoder = new CANcoder(Hardware.ELEVATOR_CANCODER_ID);
  private final Timer m_timer;

  private double m_ElevatorGoalPosition;

  /** Creates a new Elevator. */
  public Elevator()
  {
    m_ElevatorMotor1.getConfigurator().apply(ElevatorConstants.ELEVATOR_MOTOR_CONFIG);

    m_CANCoder.getConfigurator().apply(ElevatorConstants.ELEVATOR_CANCODER_CONFIGURATION);
    
    // Send a request to the second motor to follow the first
    Follower followRequest = new Follower(Hardware.ELEVATOR_MOTOR_1_ID, false);
    m_ElevatorMotor2.setControl(followRequest);

    this.m_ElevatorGoalPosition = 0;

    // Start a timer
    this.m_timer = new Timer();
    m_timer.start();
  }

  /**
   * @brief Sets the elevator motors to a given position.
   * @param position The position to send the motors to.
   */
  public void setElevatorPosition(double position)
  {
      MotionMagicVoltage request = new MotionMagicVoltage(position, false, ElevatorConstants.ELEVATOR_FEED_FORWARD, 0, false, false, false);
      m_ElevatorMotor1.setControl(request);
  }

  /**
   * @brief Sets the elevator motors to a given percentage of the available voltage.
   * @param percent The percentage to set the elevator motor to (-1 to 1).
   */
  public void setElevatorPercent(double percent)
  {
    DutyCycleOut request = new DutyCycleOut(percent, false, false, false, false);
    m_ElevatorMotor1.setControl(request);  
  }

  public void setGoalElevatorPosition(double position)
  {
    if (position == 0 || position == 1)
    {
      this.m_ElevatorGoalPosition = position;
    }
  }

  public void setBrakeMode(boolean brake)
  {
    if (brake)
    {
      m_ElevatorMotor1.setNeutralMode(NeutralModeValue.Brake);
      m_ElevatorMotor2.setNeutralMode(NeutralModeValue.Brake);
    }
    else
    {
      m_ElevatorMotor1.setNeutralMode(NeutralModeValue.Coast);
      m_ElevatorMotor2.setNeutralMode(NeutralModeValue.Coast);
    }
  }

  public double getElevatorGoalPosition()
  {
    return this.m_ElevatorGoalPosition;
  }

  public StatusSignal<Double> getElevatorMovementError()
  {
    return this.m_ElevatorMotor1.getClosedLoopError();
  }

  // Telemetry

  /** 
   * @brief Gets the elevator's current position.
   * @return A Status Signal of the current elevator position.
   */
  public StatusSignal<Double> getElevatorPosition()
  {
    return m_ElevatorMotor1.getPosition();
  }

  /**
   * @brief Gets the CANcoder's current absolute position.
   * @return A Status Signal of CANcoder's current absolute position.
   */
  public StatusSignal<Double> getCANCoderPosition()
  {
    return m_CANCoder.getAbsolutePosition();
  }

  public boolean isAtPosition(double goalPosition, double tolerance)
  {
    return (getElevatorPosition().getValueAsDouble() - tolerance <= goalPosition) && (getElevatorPosition().getValueAsDouble() + tolerance >= goalPosition);
  }

  @Override
  public void periodic()
  {
    if (m_timer.get() > 0.5)
    {
      m_timer.reset();
      SmartDashboard.putNumber("Elevator position", this.getElevatorPosition().getValue());
    }
  }
}
