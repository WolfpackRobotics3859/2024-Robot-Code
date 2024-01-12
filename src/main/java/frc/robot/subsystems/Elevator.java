// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase
{

  private final TalonFX m_ElevatorMotor1 = new TalonFX(Hardware.ELEVATOR_MOTOR_1_ID);
  private final TalonFX m_ElevatorMotor2 = new TalonFX(Hardware.ELEVATOR_MOTOR_2_ID);

  /** Creates a new Elevator. */
  public Elevator()
  {
    m_ElevatorMotor1.getConfigurator().apply(ELEVATOR_GAINS);
    m_ElevatorMotor2.getConfigurator().apply(ELEVATOR_GAINS);
    
    Follower followRequest = new Follower(Hardware.ELEVATOR_MOTOR_1_ID, false);
    m_ElevatorMotor2.setControl(followRequest);
  }

  @Override
  public void periodic()
  {
    // Intentionally Empty
  }
}
