// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Global;
import frc.robot.constants.Hardware;
import frc.robot.constants.intake.IntakeConstants;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;


public class Intake extends SubsystemBase 
{
  private final TalonFX m_RollerMotor = new TalonFX(Hardware.INTAKE_ROLLER_MOTOR_ID);
  private final TalonFX m_WristMotor = new TalonFX(Hardware.INTAKE_WRIST_MOTOR_ID);
  private final Timer m_TelemetryTimer = new Timer();
  private final Timer m_ExtraTelemetryTimer = new Timer();

  public Intake()
  {
    this.m_RollerMotor.getConfigurator().apply(IntakeConstants.INTAKE_ROLLER_CONFIGURATION);
    this.m_WristMotor.getConfigurator().apply(IntakeConstants.INTAKE_WRIST_CONFIGURATION);

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
    this.setWristZero();
  }

  @Override
  public void periodic()
  {
    if(Global.ENABLE_TELEMETRY)
    {
      if (m_TelemetryTimer.get() > Global.TELEMETRY_UPDATE_SPEED)
      {
        m_TelemetryTimer.reset();
        SmartDashboard.putNumber("Intake Wrist Position", m_WristMotor.getPosition().getValueAsDouble());
      }
    }
    // if(Global.ENABLE_EXTRA_TELEMETRY)
    // {
    //   if(m_ExtraTelemetryTimer.get() > Global.EXTRA_TELEMETRY_UPDATE_SPEED)
    //   {
    //    m_ExtraTelemetryTimer.reset();
    //     // Intentionally Empty
    //   }
    // }
  }

  public void setRollerVoltage(double voltage)
  {
    VoltageOut voltageRequest = new VoltageOut(voltage);
    m_RollerMotor.setControl(voltageRequest);
  }

  public void setWristPosition (double position)
  {
    MotionMagicVoltage request = new MotionMagicVoltage(position, false, IntakeConstants.INTAKE_WRIST_FEED_FORWARD, 0, false, false, false);
    m_WristMotor.setControl(request);
  }

  public void setWristPercent(double percent)
  {
    DutyCycleOut request = new DutyCycleOut(percent);
    m_WristMotor.setControl(request);
  }

  public void setWristZero()
  {
    m_WristMotor.setPosition(0);
  }

  public StatusSignal<Double> getPositionSignal()
  {
    return this.m_WristMotor.getPosition();
  }
<<<<<<< HEAD

  // remove at some point
  public StatusSignal<Double> getRollerVelocity()
  {
