// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.Global;
import frc.robot.constants.Hardware;
import frc.robot.constants.shooter.ShooterConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase
{
  private final TalonFX m_ShooterMotor1 = new TalonFX(Hardware.SHOOTER_MOTOR_1_ID);
  private final TalonFX m_ShooterMotor2 = new TalonFX(Hardware.SHOOTER_MOTOR_2_ID);
  private final TalonFX m_WristMotor = new TalonFX(Hardware.WRIST_MOTOR_ID);
  private final TalonFX m_FeederMotor = new TalonFX(Hardware.FEEDER_MOTOR_ID);

  private final CANcoder m_WristCANCoder = new CANcoder(Hardware.SHOOTER_WRIST_CANCODER_ID);

  private final DigitalInput m_BeamBreak1 = new DigitalInput(Hardware.BEAM_BREAK_1_ID);
  private final DigitalInput m_BeamBreak2 = new DigitalInput(Hardware.BEAM_BREAK_2_ID);

  private final Timer m_TelemetryTimer = new Timer();
  private final Timer m_ExtraTelemetryTimer = new Timer();

  /**
   * @brief Creates a new Shooter subsystem.
  */
  public Shooter() 
  {
    // Motor Configuration
    m_WristMotor.getConfigurator().apply(ShooterConstants.WRIST_MOTOR_CONFIGURATION);

    // Encoder Configuration
    m_WristCANCoder.getConfigurator().apply(ShooterConstants.WRIST_CANCODER_CONFIGURATION);

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
        SmartDashboard.putNumber("Shooter Wrist Position", m_WristMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Shooter Motor 1 Velocity", m_ShooterMotor1.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter Motor 2 Velocity", m_ShooterMotor2.getVelocity().getValueAsDouble());
        SmartDashboard.putBoolean("Beam Break 1", m_BeamBreak1.get());
        SmartDashboard.putBoolean("Beam Break 2", m_BeamBreak2.get());
      }
    }
  }

  public void setMotor(ShooterConstants.MOTOR motor, ShooterConstants.MODE controlMode, double value)
  {
    switch(controlMode)
    {
      case PERCENT:
        this.setMotorPercent(motor, value);
        break;
      case VOLTAGE:
        this.setMotorVoltage(motor, value);
        break;
      case VELOCITY:
        this.setMotorVelocity(motor, value);
        break;
      case BRAKE:
        this.brakeMotor(motor);
        break;
      case POSITION:
        this.setMotorPosition(motor, value);
      default:
        System.out.println("Warning: Invalid control mode for shooter.");
        break;
    }
  }

  public boolean getShooterReady()
  {
    return this.motor1Ready() && this.motor2Ready();
  }

  public boolean getBeamBreak1()
  {
    return m_BeamBreak1.get();
  }

  public boolean getBeamBreak2()
  {
    return m_BeamBreak2.get();
  }

  /**
   * @brief Gets the position of the wrist motor.
   * @return Returns a Status Signal of the current position.
  */
  public StatusSignal<Double> getPositionSignal()
  {
    return m_WristMotor.getPosition();
  }

  /**
   * @brief Applies a Motion Magic request to the selected motor with the set velocity.
   * @param motor The motor you want to send the request to MOTOR_1 or MOTOR_2, setting this to WRIST_MOTOR will do nothing.
   * @param velocity The desired velocity to run the motor at, measured in rotations per second.
  */
  private void setMotorVelocity(ShooterConstants.MOTOR motor, double velocity)
  {
    MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(velocity, ShooterConstants.SHOOTER_MOTOR_ACCELERATION, false, 0, 0, false, false, false);
    switch (motor)
    {
      case MOTOR_1:
        m_ShooterMotor1.setControl(request);
        break;
      case MOTOR_2:
        m_ShooterMotor2.setControl(request);
        break;
      case WRIST_MOTOR:
        System.out.println("WARNING: Setting a velocity to the wrist motor is not permitted.");
        break;
      case FEEDER_MOTOR:
        m_FeederMotor.setControl(request);
        break;
      default:
        System.out.println("WARNING: Shooter cannot assign a velocity to a motor that doesn't exist.");
        break;
    }
  }

  /**
   * @brief Applies a duty cycle to the given motor with the set percentage of the available voltage.
   * @param motor The desired motor to send the request to.
   * @param percent The percent of the available voltage to apply to the motor, from -1 to 1.
  */
  private void setMotorPercent(ShooterConstants.MOTOR motor, double percent)
  {
    DutyCycleOut percentRequest = new DutyCycleOut(percent);

    switch (motor){
      case MOTOR_1:
        m_ShooterMotor1.setControl(percentRequest);
        break;
      case MOTOR_2:
        m_ShooterMotor2.setControl(percentRequest);
        break;
      case WRIST_MOTOR:
        m_WristMotor.setControl(percentRequest);
        break;
      case FEEDER_MOTOR:
        m_FeederMotor.setControl(percentRequest);
        break;
      default:
        System.out.println("Warning: Shooter cannot set a percent to a motor that doesn't exist.");
        break;
    }
  }

  private void setMotorVoltage(ShooterConstants.MOTOR motor, double voltage)
  {
    VoltageOut voltageRequest = new VoltageOut(voltage);
    switch(motor)
    {
      case MOTOR_1:
        m_ShooterMotor1.setControl(voltageRequest);
        break;
      case MOTOR_2:
        m_ShooterMotor2.setControl(voltageRequest);
        break;
      case WRIST_MOTOR:
        m_WristMotor.setControl(voltageRequest);
        break;
      case FEEDER_MOTOR:
        m_FeederMotor.setControl(voltageRequest);
        break;
      default:
        System.out.println("Warning: Shooter cannot set a voltage to a motor that doesn't exist.");
        break;
    }
  }

  private void brakeMotor(ShooterConstants.MOTOR motor)
  {
    StaticBrake brakeRequest = new StaticBrake();
    switch(motor)
    {
      case MOTOR_1:
        m_ShooterMotor1.setControl(brakeRequest);
        break;
      case MOTOR_2:
        m_ShooterMotor2.setControl(brakeRequest);
        break;
      case WRIST_MOTOR:
        m_WristMotor.setControl(brakeRequest);
        break;
      case FEEDER_MOTOR:
        m_FeederMotor.setControl(brakeRequest);
        break;
      default:
        System.out.println("Warning: Shooter cannot brake a motor that doesn't exist.");
        break;
    }
  }

  private void setMotorPosition(ShooterConstants.MOTOR motor, double position)
  {
    MotionMagicVoltage positionRequest = new MotionMagicVoltage(position);
    switch(motor)
    {
      case MOTOR_1:
        m_ShooterMotor1.setControl(positionRequest);
        break;
      case MOTOR_2:
        m_ShooterMotor2.setControl(positionRequest);
        break;
      case WRIST_MOTOR:
        // Later once data is acquired we will most likely have to call upon a more complicated wrist position function.
        positionRequest = new MotionMagicVoltage(position, false, ShooterConstants.SHOOTER_WRIST_FEED_FORWARD, 0, false, false, false);
        m_WristMotor.setControl(positionRequest);
        break;
      case FEEDER_MOTOR:
        m_FeederMotor.setControl(positionRequest);
        break;
      default:
        System.out.println("Warning: Shooter cannot brake a motor that doesn't exist.");
        break;
    }
  }

  private boolean motor1Ready()
  {
    return m_ShooterMotor1.getClosedLoopError().getValueAsDouble() < ShooterConstants.VELOCITY_CLOSED_LOOP_ERROR_TOLERANCE;
  }

  private boolean motor2Ready()
  {
    return m_ShooterMotor1.getClosedLoopError().getValueAsDouble() < ShooterConstants.VELOCITY_CLOSED_LOOP_ERROR_TOLERANCE;
  }
}
