// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.NavigableMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.elevator.ElevatorConstants;
import frc.robot.constants.intake.IntakeConstants;
import frc.robot.constants.shooter.ShooterConstants;
import frc.robot.constants.shooter.ShooterConstants.MOTOR;
import frc.robot.utils.Maps;
import frc.robot.utils.Util;

public class Orchestrator extends SubsystemBase
{
  private final Drivetrain m_Drivetrain; // this will be used later once we have auto aiming/pathplanner
  private final Elevator m_Elevator;
  private final Shooter m_Shooter;
  private final Intake m_Intake;

  private final Timer m_Timer;
  private double m_odometryUpdateFrequency = 0.1;

  public boolean noteGrabbed = false;
  public boolean noteStowed = false;
  public boolean shotReady = false;
  public boolean shotRequested = false;
  public boolean climbing = false;

  /**
   * Creates a new orchestrator subsystem.
   * @param drivetrain The drivetrain subsystem.
   * @param elevator The elevator subsytem.
   * @param shooter The shooter subsystem.
   * @param intake The intake subsystem.
   */
  public Orchestrator(Drivetrain drivetrain, Elevator elevator, Shooter shooter, Intake intake) 
  {
    this.m_Drivetrain = drivetrain;
    this.m_Elevator = elevator;
    this.m_Shooter = shooter;
    this.m_Intake = intake;

    this.m_Timer = new Timer();
    m_Timer.start();
  }

  // odometry
  private double m_ElevatorPosition;
  
  private double m_ShooterWristPosition;
  private double m_ShooterMotor1Velocity;
  private double m_ShooterMotor2Velocity;

  private double m_IntakeWristPosition;

  // public numbers for subsystems
  public double m_DesiredElevatorPosition;
  
  public double m_DesiredShooterWristPosition;
  public double m_DesiredShooterMotor1Velocity;
  public double m_DesiredShooterMotor2Velocity;
  public double m_DesiredShooterFeederVoltage;

  public double m_DesiredIntakeWristPosition;
  public double m_DesiredIntakeRollersVelocity;

  // High Level methods

  public void newStow()
  {
    // set all wheels to off
    this.m_DesiredShooterMotor1Velocity = 0;
    this.m_DesiredShooterMotor2Velocity = 0;
    this.m_DesiredShooterFeederVoltage = 0;
    
    this.m_DesiredIntakeRollersVelocity = 0;

    // if intake needs to clear
    if (m_ElevatorPosition > ElevatorConstants.ELEVATOR_INTAKE_CLEAR_POSITION)
    {
      // set intake out of the way
      this.m_DesiredIntakeWristPosition = IntakeConstants.INTAKE_CLEAR_POSITION;
    }
    else
    {
      // otherwise set to up position
      this.m_DesiredIntakeWristPosition = IntakeConstants.INTAKE_UP_POSITION;
    }

    // if elevator is above bar 
    if (m_ElevatorPosition >= ElevatorConstants.ELEVATOR_BAR_POSITION)
    {
      // if shooter is at clearance position set elevator down
      if(Util.epsilonEquals(m_ShooterWristPosition, ShooterConstants.WRIST_CLEARANCE_POSITION, ShooterConstants.WRIST_MOVEMENT_TOLERANCE))
      {
        this.m_DesiredElevatorPosition = ElevatorConstants.ELEVATOR_BAR_POSITION - 0.12;
      }
      else
      {
        // set elevator up
        this.m_DesiredElevatorPosition = ElevatorConstants.ELEVATOR_TOP_POSITION;

        if (Util.epsilonEquals(m_ElevatorPosition, ElevatorConstants.ELEVATOR_TOP_POSITION, ElevatorConstants.ELEVATOR_POSITION_TOLERANCE))
        {
          this.m_DesiredShooterWristPosition = ShooterConstants.WRIST_CLEARANCE_POSITION;
        }
      }
    }
    // if elevator is below bar
    else
    {
      // set elevator to bottom position
      this.m_DesiredShooterWristPosition = ShooterConstants.WRIST_DEFAULT_POSITION;
      // set wrist flat
      this.m_DesiredElevatorPosition = ElevatorConstants.ELEVATOR_BOTTOM_POSITION;
    }
  }

  public void Stow()
  {
    // set all wheels to off
    this.m_DesiredShooterMotor1Velocity = 0;
    this.m_DesiredShooterMotor2Velocity = 0;
    this.m_DesiredIntakeRollersVelocity = 0;
    this.m_DesiredShooterFeederVoltage = 0;

    this.climbing = false;

    // set elevator down
    ElevatorDown();
  }

  public void Intake()
  {
    this.climbing = false;

    // if elevator is not at bottom position
    if (!Util.epsilonEquals(m_ElevatorPosition, ElevatorConstants.ELEVATOR_BOTTOM_POSITION, ElevatorConstants.ELEVATOR_POSITION_TOLERANCE))
    {
      // send elevator down
      ElevatorDown();
    }
    else
    {
      // set intake down 
      m_DesiredIntakeWristPosition = IntakeConstants.INTAKE_DOWN_POSITION;

      // run rollers
      if (Util.epsilonEquals(m_DesiredShooterWristPosition, ShooterConstants.WRIST_DEFAULT_POSITION, ShooterConstants.WRIST_MOVEMENT_TOLERANCE))
      {
        m_DesiredIntakeRollersVelocity = IntakeConstants.INTAKE_ROLLERS_VELOCITY;
        m_DesiredShooterMotor1Velocity = ShooterConstants.INTAKE_SHOOTERS_VELOCITY;
        m_DesiredShooterMotor2Velocity = ShooterConstants.INTAKE_SHOOTERS_VELOCITY;
        m_DesiredShooterFeederVoltage = ShooterConstants.INTAKE_FEEDER_VOLTAGE;
      }
    }
  }

  public void ShootSpeakerUp()
  {
    // Empty for now
  }

  public void ShootSpeakerDown()
  {
    this.climbing = false;

    this.m_DesiredIntakeWristPosition = IntakeConstants.INTAKE_DOWN_POSITION;
    // if elevator is above bar
    if (m_ElevatorPosition > ElevatorConstants.ELEVATOR_BAR_POSITION)
    {
      // if elevator is not at the top position and wrist is not up
      if (m_ElevatorPosition < ElevatorConstants.ELEVATOR_BAR_POSITION - 0.02 && !Util.epsilonEquals(m_ShooterWristPosition, ShooterConstants.WRIST_CLEARANCE_POSITION, ShooterConstants.WRIST_MOVEMENT_TOLERANCE))
      {
        m_DesiredElevatorPosition = ElevatorConstants.ELEVATOR_TOP_POSITION;
      }

      // once elevator is up
      if (Util.epsilonEquals(m_ElevatorPosition, ElevatorConstants.ELEVATOR_TOP_POSITION, ElevatorConstants.ELEVATOR_TOP_POSITION))
      {
        // set shooter to clearance position
        m_DesiredShooterWristPosition = ShooterConstants.WRIST_CLEARANCE_POSITION;
      }

      // if shooter is at clearance position
      if (Util.epsilonEquals(m_ShooterWristPosition, ShooterConstants.WRIST_CLEARANCE_POSITION, ShooterConstants.WRIST_MOVEMENT_TOLERANCE))
      {
        m_DesiredElevatorPosition = ElevatorConstants.ELEVATOR_BAR_POSITION - 0.08;
      }
    }
    // if elevator is down
    else
    {
      m_DesiredElevatorPosition = ElevatorConstants.ELEVATOR_MANUAL_SHOT_POSITION;
      // set wrist flat
      if(Util.epsilonEquals(m_ElevatorPosition, ElevatorConstants.ELEVATOR_MANUAL_SHOT_POSITION, 0.006))
      {
        m_DesiredShooterWristPosition = ShooterConstants.WRIST_MANUAL_SHOT_POSITION;
      }

      if(Util.epsilonEquals(m_ShooterWristPosition, ShooterConstants.WRIST_MANUAL_SHOT_POSITION, ShooterConstants.WRIST_MOVEMENT_TOLERANCE))
      {

        m_DesiredShooterMotor1Velocity = 45;
        m_DesiredShooterMotor2Velocity = 45;
      }

      if(Util.epsilonEquals(m_ShooterMotor2Velocity, 45, ShooterConstants.SHOOTER_VELOCITY_TOLERANCE))
      {
        m_DesiredShooterFeederVoltage = ShooterConstants.FEEDER_SHOOTING_VOLTAGE;
      }
    }
  }

  public void ShootAmp()
  {
    this.climbing = false;

    this.m_DesiredIntakeWristPosition = IntakeConstants.INTAKE_CLEAR_POSITION;

    // if elevator is above bar
    if (m_ElevatorPosition > ElevatorConstants.ELEVATOR_BAR_POSITION)
    {
      // if elevator is not at the top position and wrist is not up
      if (m_ElevatorPosition < ElevatorConstants.ELEVATOR_BAR_POSITION - 0.02 && !Util.epsilonEquals(m_ShooterWristPosition, ShooterConstants.WRIST_CLEARANCE_POSITION, ShooterConstants.WRIST_MOVEMENT_TOLERANCE))
      {
        m_DesiredElevatorPosition = ElevatorConstants.ELEVATOR_TOP_POSITION;
      }

      // once elevator is up
      if (Util.epsilonEquals(m_ElevatorPosition, ElevatorConstants.ELEVATOR_TOP_POSITION, ElevatorConstants.ELEVATOR_TOP_POSITION))
      {
        // set shooter to clearance position
        m_DesiredShooterWristPosition = ShooterConstants.WRIST_CLEARANCE_POSITION;
      }

      // if shooter is at clearance position
      if (Util.epsilonEquals(m_ShooterWristPosition, ShooterConstants.WRIST_CLEARANCE_POSITION, ShooterConstants.WRIST_MOVEMENT_TOLERANCE))
      {
        m_DesiredElevatorPosition = ElevatorConstants.ELEVATOR_BAR_POSITION - 0.08;
      }
    }
    // if elevator is down
    else
    {
      // set elevator to amp position
      m_DesiredElevatorPosition = SmartDashboard.getNumber("Amp Shot Elevator Position", ElevatorConstants.ELEVATOR_AMP_SHOT_POSITION);
      // set wrist to shooter position
      if(Util.epsilonEquals(m_ElevatorPosition, SmartDashboard.getNumber("Amp Shot Elevator Position", ElevatorConstants.ELEVATOR_AMP_SHOT_POSITION), 0.008))
      {
        m_DesiredShooterWristPosition = SmartDashboard.getNumber("Amp Shot Wrist Position", ShooterConstants.WRIST_AMP_SHOOTING_POSITION);
      }

      if(Util.epsilonEquals(m_ShooterWristPosition, SmartDashboard.getNumber("Amp Shot Wrist Position", ShooterConstants.WRIST_AMP_SHOOTING_POSITION), ShooterConstants.WRIST_MOVEMENT_TOLERANCE))
      {

        m_DesiredShooterMotor2Velocity = SmartDashboard.getNumber("Amp Shot Motor 1 Velocity", 6);
        m_DesiredShooterMotor1Velocity = SmartDashboard.getNumber("Amp Shot Motor 2 Velocity", 17.5);
      }

      if(Util.epsilonEquals(m_ShooterMotor1Velocity, SmartDashboard.getNumber("Amp Shot Motor 2 Velocity", 17.5), ShooterConstants.SHOOTER_VELOCITY_TOLERANCE))
      {
        m_DesiredShooterFeederVoltage = ShooterConstants.FEEDER_AMP_SHOT_VOLTAGE;
      }
    }
  }

  public void BumperShot()
  {
    this.climbing = false;

    // set shooter wheels to bumper shot speed
    this.m_DesiredShooterMotor1Velocity = ShooterConstants.BUMPER_SHOT_VELOCITY;
    this.m_DesiredShooterMotor2Velocity = ShooterConstants.BUMPER_SHOT_VELOCITY;
    this.m_DesiredIntakeRollersVelocity = 0;
    this.m_DesiredShooterFeederVoltage = 0;

    this.m_DesiredIntakeWristPosition = IntakeConstants.INTAKE_BUMPER_SHOT_POSITION;

    // if elevator is above bar
    if (m_ElevatorPosition > ElevatorConstants.ELEVATOR_BAR_POSITION)
    {
      // send elevator up
      m_DesiredElevatorPosition = ElevatorConstants.ELEVATOR_TOP_POSITION;
      // once elevator is up
      if (Util.epsilonEquals(m_ElevatorPosition, ElevatorConstants.ELEVATOR_TOP_POSITION, ElevatorConstants.ELEVATOR_POSITION_TOLERANCE))
      {
        // send elevator down
        m_DesiredElevatorPosition = ElevatorConstants.ELEVATOR_BOTTOM_POSITION;
      }
    }
    // if elevator is down
    else
    {
      // set elevator to bumper shot position
      m_DesiredElevatorPosition = ElevatorConstants.ELEVATOR_BUMPER_SHOT_POSITION;
      
      if(Util.epsilonEquals(m_ElevatorPosition, ElevatorConstants.ELEVATOR_BUMPER_SHOT_POSITION, ElevatorConstants.ELEVATOR_POSITION_TOLERANCE))
      {
        // set wrist to bumper shot position
        m_DesiredShooterWristPosition = ShooterConstants.WRIST_BUMPER_SHOT_POSITION;

        // if wrist is at right position
        if(Util.epsilonEquals(m_ShooterWristPosition, ShooterConstants.WRIST_BUMPER_SHOT_POSITION, ShooterConstants.WRIST_MOVEMENT_TOLERANCE))
        {
          // if shooter motors are at proper velocity
          if (Util.epsilonEquals(m_ShooterMotor1Velocity, ShooterConstants.BUMPER_SHOT_VELOCITY, ShooterConstants.SHOOTER_VELOCITY_TOLERANCE))
          {
            m_DesiredShooterFeederVoltage = ShooterConstants.FEEDER_SHOOTING_VOLTAGE;
          }
        }
      }
    }
  }

  public void Climb()
  {
    // if elevator is below bar
    if (m_ElevatorPosition < ElevatorConstants.ELEVATOR_BAR_POSITION)
    {
      // set intake out of the way
      this.m_DesiredIntakeWristPosition = IntakeConstants.INTAKE_CLEAR_POSITION;
      
      if (Util.epsilonEquals(m_IntakeWristPosition, IntakeConstants.INTAKE_CLEAR_POSITION, IntakeConstants.INTAKE_WRIST_POSITION_TOLERANCE))
      {
        // if intake is out of way send elevator up to first position
        this.m_DesiredElevatorPosition = ElevatorConstants.ELEVATOR_BAR_POSITION - 0.16;
      }
      
      // if elevator is between positions
      if (Util.inRange(m_ElevatorPosition, ElevatorConstants.ELEVATOR_BAR_POSITION - 0.17, ElevatorConstants.ELEVATOR_BAR_POSITION + 0.1))
      {
        // set shooter up
        m_DesiredShooterWristPosition = ShooterConstants.WRIST_CLEARANCE_POSITION;
      }

      if (Util.epsilonEquals(m_ShooterWristPosition, ShooterConstants.WRIST_CLEARANCE_POSITION, ShooterConstants.WRIST_MOVEMENT_TOLERANCE))
      {
        // send elevator to proper position
        m_DesiredElevatorPosition = ElevatorConstants.ELEVATOR_BAR_POSITION + 0.1;

        if (Util.epsilonEquals(m_ElevatorPosition, ElevatorConstants.ELEVATOR_BAR_POSITION + 0.1, ElevatorConstants.ELEVATOR_POSITION_TOLERANCE))
        {
          this.climbing = true;
        }
      }
      
    } 
    // if elevator above bar
    else
    {
      // if wrist is already at position just send elevator down
      if (Util.epsilonEquals(m_ShooterWristPosition, ShooterConstants.WRIST_CLEARANCE_POSITION, ShooterConstants.WRIST_MOVEMENT_TOLERANCE))
      {
        this.m_DesiredElevatorPosition = ElevatorConstants.ELEVATOR_CLIMB_POSITION;
        this.m_DesiredShooterWristPosition = ShooterConstants.WRIST_CLEARANCE_POSITION;

        if (Util.epsilonEquals(m_ElevatorPosition, ElevatorConstants.ELEVATOR_CLIMB_POSITION, ElevatorConstants.ELEVATOR_POSITION_TOLERANCE))
        {
          // climb is ready to be handed off to operator
          this.climbing = true;
        }
      }
      else
      {
        m_DesiredElevatorPosition = ElevatorConstants.ELEVATOR_TOP_POSITION;

        if (Util.epsilonEquals(m_ElevatorPosition, ElevatorConstants.ELEVATOR_TOP_POSITION, ElevatorConstants.ELEVATOR_POSITION_TOLERANCE))
        {
          // if elevator is at top position set shooter to clearance position
          m_DesiredShooterWristPosition = ShooterConstants.WRIST_CLEARANCE_POSITION;
        }
      }
    }
  }

  public void Purge()
  {
    this.climbing = false;
  
    // set shooter wheels to bumper shot speed
    this.m_DesiredShooterMotor1Velocity = ShooterConstants.PURGE_VELOCITY;
    this.m_DesiredShooterMotor2Velocity = ShooterConstants.PURGE_VELOCITY;
    this.m_DesiredIntakeRollersVelocity = IntakeConstants.PURGE_VELOCITY;
    this.m_DesiredShooterFeederVoltage = 0;
  
    this.m_DesiredIntakeWristPosition = IntakeConstants.INTAKE_BUMPER_SHOT_POSITION;
  
    // if elevator is above bar
    if (m_ElevatorPosition > ElevatorConstants.ELEVATOR_BAR_POSITION)
    {
      // send elevator up
      m_DesiredElevatorPosition = ElevatorConstants.ELEVATOR_TOP_POSITION;
      // once elevator is up
      if (Util.epsilonEquals(m_ElevatorPosition, ElevatorConstants.ELEVATOR_TOP_POSITION, ElevatorConstants.ELEVATOR_POSITION_TOLERANCE))
      {
        // send elevator down
        m_DesiredElevatorPosition = ElevatorConstants.ELEVATOR_BOTTOM_POSITION;
      }
    }
    // if elevator is down
    else
    {
      // set elevator to purge position
      m_DesiredElevatorPosition = ElevatorConstants.ELEVATOR_PURGE_POSITION;
        
      if(Util.epsilonEquals(m_ElevatorPosition, ElevatorConstants.ELEVATOR_PURGE_POSITION, ElevatorConstants.ELEVATOR_POSITION_TOLERANCE))
      {
        // set wrist to purge position
        m_DesiredShooterWristPosition = ShooterConstants.WRIST_DEFAULT_POSITION;
  
        // if wrist is at right position
        if(Util.epsilonEquals(m_ShooterWristPosition, ShooterConstants.WRIST_DEFAULT_POSITION, ShooterConstants.WRIST_MOVEMENT_TOLERANCE))
        {
          // if shooter motors are at proper velocity
          if (Util.epsilonEquals(m_ShooterMotor1Velocity, ShooterConstants.PURGE_VELOCITY, ShooterConstants.SHOOTER_VELOCITY_TOLERANCE))
          {
            m_DesiredShooterFeederVoltage = ShooterConstants.FEEDER_SHOOTING_VOLTAGE;
          }
        }
      }
    }
  }

  public void DashboardControl()
  {
    this.climbing = false;

    this.m_DesiredElevatorPosition = SmartDashboard.getNumber("Manual Elevator Position", 0);
    this.m_DesiredShooterWristPosition = SmartDashboard.getNumber("Manual Shooter Position", 0);
    this.m_DesiredIntakeWristPosition = SmartDashboard.getNumber("Manual Intake Position", 0);
  }

  public boolean ManualFire()
  {
    this.climbing = false;

    m_DesiredShooterMotor1Velocity = SmartDashboard.getNumber("Manual Shooter Roller 1 Velocity", 0);
    m_DesiredShooterMotor2Velocity = SmartDashboard.getNumber("Manual Shooter Roller 2 Velocity", 0);
    boolean rollerOneReady = NumberWithinRange(m_DesiredShooterMotor1Velocity, m_ShooterMotor1Velocity, 3);
    boolean rollerTwoReady = NumberWithinRange(m_DesiredShooterMotor2Velocity, m_ShooterMotor2Velocity, 3);
    if(rollerOneReady && rollerTwoReady)
    {
      m_DesiredShooterFeederVoltage = SmartDashboard.getNumber("Manual Feeder Voltage", 0);
    }
    if(!m_Shooter.getBeamBreak1() && !m_Shooter.getBeamBreak2())
    {
      m_DesiredShooterMotor1Velocity = 0;
      m_DesiredShooterMotor2Velocity = 0;
      m_DesiredShooterFeederVoltage = 0;
      return true;
    }
    return false;
  }

  /// Here tolerance is a +- from the expectedValue
  private boolean NumberWithinRange(double expectedValue, double actualValue, double tolerance)
  {
    return (expectedValue - tolerance) < actualValue && actualValue < (expectedValue + tolerance);
  }

  // Useful methods for repeated actions

  public void ElevatorDown()
  {
    if (m_ElevatorPosition > ElevatorConstants.ELEVATOR_INTAKE_CLEAR_POSITION)
    {
      this.m_DesiredIntakeWristPosition = IntakeConstants.INTAKE_CLEAR_POSITION;
    }
    else
    {
      this.m_DesiredIntakeWristPosition = IntakeConstants.INTAKE_UP_POSITION;
    }

    // if elevator is above bar
    if (m_ElevatorPosition > ElevatorConstants.ELEVATOR_BAR_POSITION)
    {
      // if shooter is at clearance position, just send elevator down
      if (Util.epsilonEquals(m_ShooterWristPosition, ShooterConstants.WRIST_CLEARANCE_POSITION, ShooterConstants.WRIST_MOVEMENT_TOLERANCE))
      {
        this.m_DesiredElevatorPosition = ElevatorConstants.ELEVATOR_BAR_POSITION - 0.14;
      }
      // otherwise send elevator up first
      else
      {
        this.m_DesiredElevatorPosition = ElevatorConstants.ELEVATOR_TOP_POSITION;

        // if elevator is up
        if (Util.epsilonEquals(m_ElevatorPosition, ElevatorConstants.ELEVATOR_TOP_POSITION, ElevatorConstants.ELEVATOR_TOP_POSITION))
        {
        // set shooter to clearance position
        this.m_DesiredShooterWristPosition = ShooterConstants.WRIST_CLEARANCE_POSITION;
        }
      }
    }
    // if elevator is below the bar
    else
    {
      this.m_DesiredShooterWristPosition = ShooterConstants.WRIST_DEFAULT_POSITION;
      // set elevator to bottom position
      this.m_DesiredElevatorPosition = ElevatorConstants.ELEVATOR_BOTTOM_POSITION;
      // set wrist flat
    }
  }

  public void ElevatorUp()
  {
    if (m_ElevatorPosition < ElevatorConstants.ELEVATOR_INTAKE_CLEAR_POSITION + 0.03)
    {
      this.m_DesiredIntakeWristPosition = IntakeConstants.INTAKE_CLEAR_POSITION;
    }
    else
    {
      this.m_DesiredIntakeWristPosition = IntakeConstants.INTAKE_UP_POSITION;
    }

    // if elevator is below bar
    if (m_ElevatorPosition < ElevatorConstants.ELEVATOR_BAR_POSITION)
    {
      // if elevator is below position
      if(m_ElevatorPosition < ElevatorConstants.ELEVATOR_BAR_POSITION - 0.12)
      {
        // send elevator up to first position
        m_DesiredElevatorPosition = ElevatorConstants.ELEVATOR_BAR_POSITION - 0.05;
      }
      
      // if elevator is between positions
      if (Util.inRange(m_ElevatorPosition, ElevatorConstants.ELEVATOR_BAR_POSITION - 0.08, ElevatorConstants.ELEVATOR_BAR_POSITION + 0.11))
      {
        // set shooter up
        m_DesiredShooterWristPosition = ShooterConstants.WRIST_CLEARANCE_POSITION;
      }

      if (Util.epsilonEquals(m_ShooterWristPosition, ShooterConstants.WRIST_CLEARANCE_POSITION, ShooterConstants.WRIST_MOVEMENT_TOLERANCE))
      {
        m_DesiredElevatorPosition = ElevatorConstants.ELEVATOR_TOP_POSITION;
      }
    } 
    // if elevator above bar
    else
    {
      m_DesiredElevatorPosition = ElevatorConstants.ELEVATOR_TOP_POSITION;

      if (Util.epsilonEquals(m_ElevatorPosition, ElevatorConstants.ELEVATOR_TOP_POSITION, ElevatorConstants.ELEVATOR_POSITION_TOLERANCE))
      {
        // if elevator is at top position return shooter to default position
        m_DesiredShooterWristPosition = ShooterConstants.WRIST_DEFAULT_POSITION;
      }
    }
  }

  @Override
  public void periodic()
  {
    // V3 PERIODIC:
    // Periodic should be used for updating odometry values from the subsystems.
    // This should be done via the getters in the subsystems and should do it on a (variable) timer for
    // a reduction in the canbus utilization

    SmartDashboard.putData(this);
    SmartDashboard.putNumber("Elevator Desired Position", m_DesiredElevatorPosition);
    SmartDashboard.putNumber("Shooter Wrist Desired Position", m_DesiredShooterWristPosition);
    SmartDashboard.putNumber("Intake Wrist Desired Position", m_DesiredIntakeWristPosition);
    SmartDashboard.putBoolean("Note Grabbed", noteGrabbed);
    SmartDashboard.putBoolean("Note Stowed", noteStowed);

    if (m_Timer.get() > m_odometryUpdateFrequency)
    {
      m_Timer.reset();

      this.m_ElevatorPosition = m_Elevator.getElevatorPosition().getValueAsDouble();

      this.m_ShooterWristPosition = m_Shooter.getWristMotorPosition().getValueAsDouble();
      this.m_ShooterMotor1Velocity = m_Shooter.getShooterMotorVelocity(MOTOR.MOTOR_1).getValueAsDouble();
      this.m_ShooterMotor2Velocity = m_Shooter.getShooterMotorVelocity(MOTOR.MOTOR_2).getValueAsDouble();

      this.m_IntakeWristPosition = m_Intake.getWristPosition().getValueAsDouble();
    }

    // if both beam breaks broken
    if (!m_Shooter.getBeamBreak1() && !m_Shooter.getBeamBreak2())
    {
      // note has been grabbed
      this.noteGrabbed = true;
      this.noteStowed = false;
    }

    // if a note has been grabbed and the front beam break is unbroken and second beam break is broken
    if (noteGrabbed && m_Shooter.getBeamBreak1() && !m_Shooter.getBeamBreak2())
    {
      this.noteStowed = true;
      this.noteGrabbed = false;
    }

    // if back beam break broken and front is not broken
    if (m_Shooter.getBeamBreak1() && !m_Shooter.getBeamBreak2())
    {
      // note has been grabbed
      this.noteStowed = true;
    }

    // if both beam breaks unbroken set both to false
    if (m_Shooter.getBeamBreak1() && m_Shooter.getBeamBreak2())
    {
      this.noteGrabbed = false;
      this.noteStowed = false;
    }
  }

  // keep for now
  /** An enum containing the various states the robot can be asked to go to. */
  public enum RobotState
  {
    STOW,
    SHOOTING_AMP,
    SHOOTING_SPEAKER_DOWN,
    SHOOTING_SPEAKER_UP, // under defense
    INTAKING,
    SWITCHING, // robot is switching between states
    DISABLED,
  }

  private double getElevatorMaxUpPosition()
  {
    NavigableMap<Double, Double> map = Maps.ElevatorToShooter;
    double returnValue;

    // map contains the key
    if (map.containsKey(m_ShooterWristPosition))
    {
      returnValue = map.get(m_ShooterWristPosition);
    }

    Double lowerKey = map.lowerKey(m_ShooterWristPosition);
    Double higherKey = map.higherKey(m_ShooterWristPosition);

    if (lowerKey == null) {
      returnValue = map.get(higherKey);
    } else if (higherKey == null) {
      returnValue = map.get(lowerKey);
    } else {
      // key is between two existing keys, find the closest one
      double lowerDiff = m_ShooterWristPosition - lowerKey;
      double higherDiff = higherKey - m_ShooterWristPosition;

      returnValue = lowerDiff < higherDiff ? map.get(lowerKey) : map.get(higherKey);
    }

    // never go above the clearance position
    if (returnValue >= ShooterConstants.WRIST_CLEARANCE_POSITION)
    {
      return ShooterConstants.WRIST_CLEARANCE_POSITION;
    }
    else
    {
      return returnValue;
    }
  }

  private double getShooterWristMaxUpPosition()
  {
    NavigableMap<Double, Double> map = Maps.ShooterToElevator;
    double returnValue;
    
    // map contains the key
    if (map.containsKey(m_ShooterWristPosition))
    {
      returnValue = map.get(m_ShooterWristPosition);
    }

    Double lowerKey = map.lowerKey(m_ShooterWristPosition);
    Double higherKey = map.higherKey(m_ShooterWristPosition);

    if (lowerKey == null) {
      returnValue = map.get(higherKey);
    } else if (higherKey == null) {
      returnValue = map.get(lowerKey);
    } else {
      // key is between two existing keys, find the closest one
      double lowerDiff = m_ShooterWristPosition - lowerKey;
      double higherDiff = higherKey - m_ShooterWristPosition;

      returnValue = lowerDiff < higherDiff ? map.get(lowerKey) : map.get(higherKey);
    }

    // never go above the clearance position
    if (returnValue >= ShooterConstants.WRIST_CLEARANCE_POSITION)
    {
      return ShooterConstants.WRIST_CLEARANCE_POSITION;
    }
    else
    {
      return returnValue;
    }
  }
}
