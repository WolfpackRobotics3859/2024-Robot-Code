// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Global;
import frc.robot.constants.Hardware;
import frc.robot.constants.elevator.ElevatorConstants;

public class Elevator extends SubsystemBase
{
  private final TalonFX m_ElevatorMotor1 = new TalonFX(Hardware.ELEVATOR_MOTOR_1_ID);
  private final TalonFX m_ElevatorMotor2 = new TalonFX(Hardware.ELEVATOR_MOTOR_2_ID);

  private final CANcoder m_CANCoder = new CANcoder(Hardware.ELEVATOR_CANCODER_ID);
  private final Timer m_TelemetryTimer = new Timer();
  private final Timer m_ExtraTelemetryTimer = new Timer();
  
  private StatusSignal<Double> m_ShooterWristSignal;

  public Elevator()
  {
    m_ElevatorMotor1.getConfigurator().apply(ElevatorConstants.ELEVATOR_MOTOR_CONFIG);
    m_ElevatorMotor2.getConfigurator().apply(ElevatorConstants.CURRENT_LIMITS_CONFIGS);
    Follower followRequest = new Follower(Hardware.ELEVATOR_MOTOR_1_ID, false);
    m_ElevatorMotor2.setControl(followRequest);
    m_CANCoder.getConfigurator().apply(ElevatorConstants.ELEVATOR_CANCODER_CONFIGURATION);
    
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

  public void configureElevator(StatusSignal<Double> wristSignal)
  {
    this.m_ShooterWristSignal = wristSignal;
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
        SmartDashboard.putNumber("Goal Elevator Position", 0);
      }
    }
    // if(Global.ENABLE_EXTRA_TELEMETRY)
    // {
    //   if(m_ExtraTelemetryTimer.get() > Global.EXTRA_TELEMETRY_UPDATE_SPEED)
    //   {
    //     // Intentionally Empty
    //   }
    // }
  }

  /**
   * Creates and submits a control request to the elevator.
   * @param mode Available modes: voltage, percent, position, brake.
   * @param value When mode is VOLTAGE the voltage (-12 <= value <= 12). When mode is PERCENT the percent output (-1 <= value <= 1).
   *              When mode is POSITION the position in rotations. When mode is BRAKE then value is ignored.
   */
  public void elevatorRequest(ElevatorConstants.MODE mode, double value)
  {
    switch(mode)
    {
      case VOLTAGE:
        setElevatorVoltage(value);
        break;
      case PERCENT:
        this.setElevatorPercent(value);
        break;
      case POSITION:
        this.setElevatorPosition(value);
        break;
      case BRAKE:
        this.setElevatorBrake();
        break;
    }
  }

  /** 
   * @brief Gets the elevator's current position.
   * @return A Status Signal of the current elevator position.
   */
  public StatusSignal<Double> getPositionSignal()
  {
    return m_ElevatorMotor1.getPosition();
  }

  public boolean isAboveBar()
  {
    return this.m_ElevatorMotor1.getPosition().getValueAsDouble() > ElevatorConstants.BAR_TOP_CLEAR;
  }

  public boolean isPositionAboveBar(double position)
  {
    return position >= ElevatorConstants.BAR_TOP_CLEAR;
  }

  public boolean isBelowBar()
  {
    return this.m_ElevatorMotor1.getPosition().getValueAsDouble() < ElevatorConstants.BAR_BOTTOM_CLEAR;
  }

  public boolean isPositionBelowBar(double position)
  {
    return position < ElevatorConstants.BAR_BOTTOM_CLEAR;
  }

  public boolean isInPosition(double position)
  {
    return Math.abs(m_ElevatorMotor1.getPosition().getValueAsDouble() - position) <  ElevatorConstants.CLOSED_LOOP_ERROR_TOLERANCE;
  }

  /**
   * @brief Sets the elevator motors to a given percentage of the available voltage.
   * @param percent The percentage to set the elevator motor to (-1 to 1).
   * @details If this function behaves weirdly consider increasing the update frequency of the request.
   */
  private void setElevatorPercent(double percent)
  {
    DutyCycleOut request = new DutyCycleOut(percent, false, false, false, false);
    m_ElevatorMotor1.setControl(request);  
  }

  /**
   * @brief Sets the elevator motors to a voltage output.
   * @param voltage The voltage (-12 <= voltage <= 12).
   * @details If this function behaves weirdly consider increasing the update frequency of the request.
   */
  private void setElevatorVoltage(double voltage)
  {
    VoltageOut request = new VoltageOut(voltage, false, false, false, false);
    m_ElevatorMotor1.setControl(request);
  }

  /**
   * @brief Sets the elevator motors to a given position.
   * @param position The position to send the motors to.
   */
  private void setElevatorPosition(double position)
  {
      MotionMagicVoltage request = new MotionMagicVoltage(position, false, ElevatorConstants.ELEVATOR_FEED_FORWARD, 0, false, false, false);
      m_ElevatorMotor1.setControl(request);
  }

  /**
   * @brief Sets the elevator to a static brake mode.
   */
  private void setElevatorBrake()
  {
    StaticBrake request = new StaticBrake();
    m_ElevatorMotor1.setControl(request);
  }
}
