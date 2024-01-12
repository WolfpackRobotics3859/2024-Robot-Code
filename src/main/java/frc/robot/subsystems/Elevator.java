// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.Position;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

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
    
    // Send a request to the second motor to follow the first
    Follower followRequest = new Follower(Hardware.ELEVATOR_MOTOR_1_ID, false);
    m_ElevatorMotor2.setControl(followRequest);
  }

  public void setElevatorPosition(double position)
  {
    PositionDutyCycle request = new PositionDutyCycle(position, ElevatorConstants.ELEVATOR_VELOCITY, false, 0, 0, false, false, false);
    m_ElevatorMotor1.setControl(request);
  }

  public void setElevatorPercent(double percent)
  {
    DutyCycleOut request = new DutyCycleOut(percent, false, false, false, false);
    m_ElevatorMotor1.setControl(request);
  }

  public StatusSignal<Double> getElevatorPosition()
  {
    return m_ElevatorMotor1.getPosition();
  }

  @Override
  public void periodic()
  {
    // Intentionally Empty
  }
}
