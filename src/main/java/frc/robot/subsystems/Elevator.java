// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.Position;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.elevator.ElevatorConstants;

public class Elevator extends SubsystemBase
{
  private final TalonFX m_ElevatorMotor1 = new TalonFX(Hardware.ELEVATOR_MOTOR_1_ID);
  private final TalonFX m_ElevatorMotor2 = new TalonFX(Hardware.ELEVATOR_MOTOR_2_ID);

  /** Creates a new Elevator. */
  public Elevator()
  {
    // Apply gains to both motors
    m_ElevatorMotor1.getConfigurator().apply(ElevatorConstants.ELEVATOR_GAINS);
    m_ElevatorMotor2.getConfigurator().apply(ElevatorConstants.ELEVATOR_GAINS);

    // Applies the software position limit to the first motor
    m_ElevatorMotor1.getConfigurator().apply(ElevatorConstants.SOFT_LIMIT_CONFIGS);
    
    // Applies limit switch configuration to the first motor
    m_ElevatorMotor1.getConfigurator().apply(ElevatorConstants.HARD_LIMIT_CONFIGS);

    // Applies a brake neutral mode to both motors
    m_ElevatorMotor1.getConfigurator().apply(ElevatorConstants.BRAKE_CONFIG);
    m_ElevatorMotor2.getConfigurator().apply(ElevatorConstants.BRAKE_CONFIG);
    
    // Send a request to the second motor to follow the first
    Follower followRequest = new Follower(Hardware.ELEVATOR_MOTOR_1_ID, false);
    m_ElevatorMotor2.setControl(followRequest);
  }

  /**
   * @brief Sets the elevator motors to a given position.
   * @param position The position to send the motors to.
   */
  public void setElevatorPosition(double position)
  {
    PositionDutyCycle request = new PositionDutyCycle(position, ElevatorConstants.ELEVATOR_VELOCITY, false, 0, 0, false, false, false);
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
   * @brief Gets the current state of the limit switch.
   * @return The current state of the limit switch as a ReverseLimitValue.
   */
  public ReverseLimitValue getLimitSwitch()
  {
    return m_ElevatorMotor1.getReverseLimit().getValue();
  }

  @Override
  public void periodic()
  {
    // Intentionally Empty
  }
}
