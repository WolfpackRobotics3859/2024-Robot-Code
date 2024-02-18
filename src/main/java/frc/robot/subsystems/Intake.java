// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Hardware;
import frc.robot.constants.intake.IntakeConstants;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;


public class Intake extends SubsystemBase 
{
  /** Creates a new IntakeSubsystem. */

  private final TalonFX m_RollerMotor = new TalonFX(Hardware.INTAKE_ROLLER_MOTOR_ID);
  private final TalonFX m_WristMotor = new TalonFX(Hardware.INTAKE_WRIST_MOTOR_ID);


  public Intake()
  {
    this.m_RollerMotor.getConfigurator().apply(IntakeConstants.INTAKE_ROLLER_GAINS);
    this.m_WristMotor.getConfigurator().apply(IntakeConstants.INTAKE_WRIST_GAINS);
    this.m_WristMotor.getConfigurator().apply(IntakeConstants.WRIST_MOTOR_MOTION_MAGIC_CONFIGS);
  }

  public void setRollerPercent(double percent)
  {
    DutyCycleOut request = new DutyCycleOut(percent, false, false, false, false);
    m_RollerMotor.setControl(request);
  }

  public void setWristPosition (double position)
  {
    MotionMagicVoltage request = new MotionMagicVoltage(position, false, IntakeConstants.INTAKE_WRIST_FEED_FORWARD, 0, false, false, false);
    m_WristMotor.setControl(request);
  }

  public StatusSignal<Double> getWristPosition()
  {
    return this.m_WristMotor.getPosition();
  }

  public boolean isAtPosition(double goalPosition, double tolerance)
  {
    return (getWristPosition().getValueAsDouble() - tolerance <= goalPosition) && (getWristPosition().getValueAsDouble() + tolerance >= goalPosition);
  }

   @Override
  public void periodic()
  {
    SmartDashboard.putNumber("Intake Wrist Position", this.getWristPosition().getValueAsDouble());
  }
}
