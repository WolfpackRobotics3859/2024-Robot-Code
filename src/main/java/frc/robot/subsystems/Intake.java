// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Hardware;
import frc.robot.constants.intake.IntakeConstants;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;


public class Intake extends SubsystemBase 
{
  /** Creates a new IntakeSubsystem. */

  private final TalonFX m_RollerMotor = new TalonFX(Hardware.INTAKE_ROLLER_MOTOR_ID);
  private final TalonFX m_WristMotor = new TalonFX(Hardware.INTAKE_WRIST_MOTOR_ID);

  private final Timer m_Timer;

  public Intake()
  {
    this.m_RollerMotor.getConfigurator().apply(IntakeConstants.INTAKE_ROLLER_CONFIGURATION);
    this.m_WristMotor.getConfigurator().apply(IntakeConstants.INTAKE_WRIST_CONFIGURATION);

    this.m_WristMotor.setPosition(0);

    this.m_Timer = new Timer();
    m_Timer.start();
  }

  // remove at some point
  public void setRollerPercent(double percent)
  {
    DutyCycleOut request = new DutyCycleOut(percent, false, false, false, false);
    m_RollerMotor.setControl(request);
  }

  public void setRollersVelocity(double velocity)
  {
    MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(velocity, IntakeConstants.INTAKE_ROLLERS_ACCELERATION, false, 0, 0, false, false, false);
    m_RollerMotor.setControl(request);
  }

  public void setWristPosition (double position)
  {
    MotionMagicVoltage request = new MotionMagicVoltage(position, false, IntakeConstants.INTAKE_WRIST_FEED_FORWARD, 0, false, false, false);
    m_WristMotor.setControl(request);
  }

  public void setWristPercent(double percent)
  {
    DutyCycleOut request = new DutyCycleOut(percent, false, false, false, false);
    m_WristMotor.setControl(request);
  }

  public void zeroWrist()
  {
    m_WristMotor.setPosition(0);
  }

  public StatusSignal<Double> getWristPosition()
  {
    return this.m_WristMotor.getPosition();
  }

  // remove at some point
  public StatusSignal<Double> getRollerVelocity()
  {
    return this.m_RollerMotor.getVelocity();
  }

   @Override
  public void periodic()
  {
    if(m_Timer.get() > 0.5)
    {
      m_Timer.reset();
      SmartDashboard.putNumber("Intake Wrist Position", this.getWristPosition().getValueAsDouble());
    }
  }
}
