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


  /** Creates a new Elevator. */
  public Elevator()
  {
    // Applies gains to the motor
    m_ElevatorMotor1.getConfigurator().apply(ElevatorConstants.ELEVATOR_GAINS);

    // Applies Motion Magic configs to the motor
    m_ElevatorMotor1.getConfigurator().apply(ElevatorConstants.MOTION_MAGIC_CONFIGS);

    // Applies the software position limit to the first motor
    m_ElevatorMotor1.getConfigurator().apply(ElevatorConstants.SOFT_LIMIT_CONFIGS);

    // Applies a brake neutral mode to both motors
    m_ElevatorMotor1.getConfigurator().apply(ElevatorConstants.BRAKE_CONFIG);

    // Configures the CANCoder
    m_CANCoder.getConfigurator().apply(ElevatorConstants.MAGNET_SENSOR_CONFIGS);

    // Applies the cancoder as the feedback source for the motor
    m_ElevatorMotor1.getConfigurator().apply(ElevatorConstants.FEEDBACK_CONFIGS);
    
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
    if (position <= ElevatorConstants.ELEVATOR_MAX_FORWARD_POS || position >= ElevatorConstants.ELEVATAOR_MAX_REVERSE_POS)
    {
      MotionMagicVoltage request = new MotionMagicVoltage(position, false, ElevatorConstants.ELEVATOR_FEED_FORWARD, 0, false, false, false);
      m_ElevatorMotor1.setControl(request);
    }
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

  @Override
  public void periodic()
  {
    if (m_timer.get() > 0.5)
    {
      m_timer.reset();
      SmartDashboard.putNumber("Elevator position", this.getElevatorPosition().getValue());
      SmartDashboard.putNumber("Cancoder position", this.getCANCoderPosition().getValue());
    }
  }
}
