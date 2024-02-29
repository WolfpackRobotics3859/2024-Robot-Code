// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Hardware;
import frc.robot.constants.elevator.ElevatorConstants;
import frc.robot.utils.Util;

public class Elevator extends SubsystemBase
{
  private final TalonFX m_ElevatorMotor1 = new TalonFX(Hardware.ELEVATOR_MOTOR_1_ID);
  private final TalonFX m_ElevatorMotor2 = new TalonFX(Hardware.ELEVATOR_MOTOR_2_ID);
  private final CANcoder m_CANCoder = new CANcoder(Hardware.ELEVATOR_CANCODER_ID);
  private final Timer m_timer;

  /** Creates a new Elevator. */
  public Elevator()
  {
    m_ElevatorMotor1.getConfigurator().apply(ElevatorConstants.ELEVATOR_MOTOR_CONFIG);

    m_CANCoder.getConfigurator().apply(ElevatorConstants.ELEVATOR_CANCODER_CONFIGURATION);
    
    // Send a request to the second motor to follow the first
    Follower followRequest = new Follower(Hardware.ELEVATOR_MOTOR_1_ID, false);
    m_ElevatorMotor2.setControl(followRequest);

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

  public void setElevatorBrake()
  {
    StaticBrake request = new StaticBrake();
    m_ElevatorMotor1.setControl(request);
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

  // Telemetry

  /** 
   * @brief Gets the elevator's current position.
   * @return A Status Signal of the current elevator position.
   */
  public StatusSignal<Double> getElevatorPosition()
  {
    return m_ElevatorMotor1.getPosition();
  }

  // ALL SHOULD PROBABLY BE REMOVED AT SOME POINT
  // if the elevator is above or below the crossbar
  public BooleanSupplier elevatorIsAboveBar = () -> Util.inRange(getElevatorPosition().getValueAsDouble(), ElevatorConstants.ELEVATOR_BAR_POSITION, ElevatorConstants.ELEVATOR_MAX_FORWARD_POS);
  public BooleanSupplier elevatorIsBelowBar = () -> Util.inRange(getElevatorPosition().getValueAsDouble(), ElevatorConstants.ELEVATAOR_MAX_REVERSE_POS, ElevatorConstants.ELEVATOR_BAR_POSITION);

  // if the elevator is above the position to begin moving the shooter to clear the bar
  public BooleanSupplier elevatorIsAboveClearancePosition = () -> getElevatorPosition().getValueAsDouble() > ElevatorConstants.ELEVATOR_BOTTOM_CLEARANCE_POSITION;

  // if the elevator is above the position where the intake must be moved before sending the elevator down
  public BooleanSupplier elevatorIsAboveIntakeClearancePosition = () -> getElevatorPosition().getValueAsDouble() > ElevatorConstants.ELEVATOR_INTAKE_CLEAR_POSITION;

  // if the elevator is at the top
  public BooleanSupplier elevatorIsAtTop = () -> Util.epsilonEquals(getElevatorPosition().getValueAsDouble(), ElevatorConstants.ELEVATOR_TOP_POSITION, ElevatorConstants.ELEVATOR_POSITION_TOLERANCE);

  @Override
  public void periodic()
  {
    if (m_timer.get() > 0.5)
    {
      m_timer.reset();
      SmartDashboard.putNumber("Elevator position", this.getElevatorPosition().getValueAsDouble());
    }

    SmartDashboard.putData(this);
  }
}
