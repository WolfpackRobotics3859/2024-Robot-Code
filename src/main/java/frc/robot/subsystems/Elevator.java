// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Global;
import frc.robot.constants.Ports;
import frc.robot.constants.elevator.ElevatorConstants;

public class Elevator extends SubsystemBase
{
  private final TalonFX m_ElevatorMotor1 = new TalonFX(Ports.ELEVATOR_MOTOR_1_ID);
  private final TalonFX m_ElevatorMotor2 = new TalonFX(Ports.ELEVATOR_MOTOR_2_ID);

  private final CANcoder m_CANCoder = new CANcoder(Ports.ELEVATOR_CANCODER_ID);
  
  private final Timer m_TelemetryTimer = new Timer();
  private final Timer m_ExtraTelemetryTimer = new Timer();
  
  public Elevator()
  {
    m_ElevatorMotor1.getConfigurator().apply(ElevatorConstants.ELEVATOR_MOTOR_1_CONFIG);
    m_ElevatorMotor2.getConfigurator().apply(ElevatorConstants.ELEVATOR_MOTOR_2_CONFIG);

    m_CANCoder.getConfigurator().apply(ElevatorConstants.ELEVATOR_CANCODER_CONFIGURATION);

    Follower followRequest = new Follower(Ports.ELEVATOR_MOTOR_1_ID, false);
    m_ElevatorMotor2.setControl(followRequest);
    
    // Telemetry Configuration
    if(Global.ENABLE_TELEMETRY)
    {
      m_TelemetryTimer.start();
    }
    if(Global.ENABLE_EXTRA_TELEMETRY)
    {
      m_ExtraTelemetryTimer.start();
    }

    SmartDashboard.putData(this);
  }

  @Override
  public void periodic()
  {
    if(Global.ENABLE_TELEMETRY)
    {
      if (m_TelemetryTimer.get() > Global.TELEMETRY_UPDATE_SPEED)
      {
        m_TelemetryTimer.reset();
        SmartDashboard.putNumber("Current Elevator Position", this.m_ElevatorMotor1.getPosition().getValueAsDouble());
      }
    }
  }

  /**
   * Sets the elevator motors to a given position.
   * @param position The position to send the motors to.
   */
  public void setElevatorPosition(double position)
  {
    MotionMagicVoltage request = new MotionMagicVoltage(position, false, ElevatorConstants.ELEVATOR_FEED_FORWARD, 0, false, false, false);
    m_ElevatorMotor1.setControl(request);
  }

  /** 
   * Gets the elevator's current position.
   * @return The elevator's current position.
   */
  public double getPosition()
  {
    return m_ElevatorMotor1.getPosition().getValueAsDouble();
  }

  public boolean isInPosition(double position)
  {
    return Math.abs(getPosition() - position) <  ElevatorConstants.CLOSED_LOOP_ERROR_TOLERANCE;
  }
}
