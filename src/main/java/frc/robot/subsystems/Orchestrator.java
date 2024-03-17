// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Global;
import frc.robot.constants.Positions;
import frc.robot.constants.elevator.ElevatorConstants;
import frc.robot.constants.intake.IntakeConstants;
import frc.robot.constants.shooter.ShooterConstants;

public class Orchestrator extends SubsystemBase
{
  private final Drivetrain m_Drivetrain; // this will be used later once we have auto aiming/pathplanner
  private final Elevator m_Elevator;
  private final Shooter m_Shooter;
  private final Intake m_Intake;
  private final Timer m_TelemetryTimer = new Timer();
  private final Timer m_ExtraTelemetryTimer = new Timer();

  private double m_ShooterTopRollerVelocity = 0;
  private double m_ShooterBottomRollerVelocity = 0;
  private double m_ShooterFeederVoltage = 0;
  private double m_ShooterAngle = 0;
  private double m_IntakePosition = 0;
  private double m_IntakeRollersVoltage = 0;
  private double m_ElevatorPosition = 0;

  private boolean m_FreshCommand = true;
  private boolean m_ShootAmp = false;

  private StatusSignal<Double> m_ElevatorPositionSignal;

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
    this.setup();
    this.initializeManualControlValues();

    m_ElevatorPositionSignal = m_Elevator.getPositionSignal();
  }

  private void setup()
  {
    if(Global.ENABLE_EXTRA_TELEMETRY)
    {
      m_ExtraTelemetryTimer.start();
    }
    if(Global.ENABLE_TELEMETRY)
    {
      m_TelemetryTimer.start();
    }
    SmartDashboard.putData(this);
  }

  private void initializeManualControlValues()
  {
    SmartDashboard.setDefaultNumber("[Manual] Top Roller Velocity", 0);
    SmartDashboard.setDefaultNumber("[Manual] Bottom Roller Velocity", 0);
    SmartDashboard.setDefaultNumber("[Manual] Feeder Roller Voltage", 0);
    SmartDashboard.setDefaultNumber("[Manual] Shooter Angle", 0.45);
    SmartDashboard.setDefaultNumber("[Manual] Elevator Position", 0.67);
    SmartDashboard.setDefaultNumber("[Manual] Intake Position", 0.5);
    SmartDashboard.setDefaultNumber("[Manual] Intake Voltage", 0);
  }

  @Override
  public void periodic()
  {
    if(Global.ENABLE_TELEMETRY)
    {
      if(m_TelemetryTimer.get() > Global.TELEMETRY_UPDATE_SPEED)
      {
        BaseStatusSignal.refreshAll(m_ElevatorPositionSignal);
        m_TelemetryTimer.reset();
      }
    }
  }

  // subsystem getters
  public double getShooterTopRollerVelocity()
  {
    return this.m_ShooterTopRollerVelocity;
  }

  public double getShooterBottomRollerVelocity()
  {
    return this.m_ShooterBottomRollerVelocity;
  }

  public double getShooterFeederVoltage()
  {
    return this.m_ShooterFeederVoltage;
  }

  public double getShooterAngle()
  {
    return this.m_ShooterAngle;
  }

  public double getElevatorPosition()
  {
    return this.m_ElevatorPosition;
  }

  public double getIntakePosition()
  {
    return this.m_IntakePosition;
  }

  public double getIntakeRollersVoltage()
  {
    return this.m_IntakeRollersVoltage;
  }

  public void freshenOrchestrator()
  {
    this.m_FreshCommand = true;
  }

  // control methods
  public void manualControl()
  {
    m_ShooterTopRollerVelocity = SmartDashboard.getNumber("[Manual] Top Roller Velocity", 0);
    m_ShooterBottomRollerVelocity = SmartDashboard.getNumber("[Manual] Bottom Roller Velocity", 0);
    m_ShooterFeederVoltage = SmartDashboard.getNumber("[Manual] Feeder Roller Voltage", 0);
    m_ShooterAngle = SmartDashboard.getNumber("[Manual] Shooter Angle", 0.6);
    m_ElevatorPosition = SmartDashboard.getNumber("[Manual] Elevator Position", 0.275);
    m_IntakePosition = SmartDashboard.getNumber("[Manual] Intake Position", 2.7);
    m_IntakeRollersVoltage = SmartDashboard.getNumber("[Manual] Intake Voltage", 0);
  }

  // Should include the ability to kill motors once robot is safely stowed.
  public void stow()
  {
    m_IntakeRollersVoltage = Positions.STOW.INTAKE_ROLLER_VOLTAGE;

    if (!m_Shooter.hasNoteCentered())
    {
      this.noteToMiddle();
    }
    else
    {
      m_ShooterTopRollerVelocity = Positions.STOW.SHOOTER_ROLLER_1_VELOCITY;
      m_ShooterBottomRollerVelocity = Positions.STOW.SHOOTER_ROLLER_2_VELOCITY;
      m_ShooterFeederVoltage = Positions.STOW.SHOOTER_FEEDER_VOLTAGE;
    }

    if(elevatorDown())
    {
      m_ElevatorPosition = Positions.STOW.ELEVATOR_POSITION;
      m_ShooterAngle = Positions.STOW.SHOOTER_WRIST_ANGLE;
      if (m_Elevator.isInPosition(m_ElevatorPosition) && m_Shooter.inPosition(m_ShooterAngle))
      {
        m_IntakePosition = Positions.STOW.INTAKE_WRIST_POSITION;
      }
    }
  }

  public void intake()
  {
    if(m_Shooter.hasNoteRearPosition() || m_Shooter.hasNoteCentered())
    {
      m_ShooterTopRollerVelocity = 0;
      m_ShooterBottomRollerVelocity = 0;
      m_ShooterFeederVoltage = 0;
      m_IntakeRollersVoltage = 0;
      this.stow();
      return;
    }
    if(elevatorDown())
    {
      m_ElevatorPosition = Positions.INTAKING.ELEVATOR_POSITION;
      m_ShooterAngle = Positions.INTAKING.SHOOTER_WRIST_ANGLE;
      m_ShooterTopRollerVelocity = Positions.INTAKING.SHOOTER_ROLLER_1_VELOCITY;
      m_ShooterBottomRollerVelocity = Positions.INTAKING.SHOOTER_ROLLER_2_VELOCITY;
      m_ShooterFeederVoltage = Positions.INTAKING.SHOOTER_FEEDER_VOLTAGE;
      m_IntakePosition = Positions.INTAKING.INTAKE_WRIST_POSITION;
      m_IntakeRollersVoltage = Positions.INTAKING.INTAKE_ROLLER_VOLTAGE;
    }
  }

  // If no vision data is available, this will default to bumper shot.
  // If the user is outside of range, this will do nothing.
  // Returns true when no note is detected anymore.
  public boolean shootLow()
  {
    if(m_Shooter.shooterClear())
    {
      m_ShooterTopRollerVelocity = 0;
      m_ShooterBottomRollerVelocity = 0;
      m_ShooterFeederVoltage = 0;
      this.stow();
      return true;
    }

    if(elevatorDown())
    {
      if(m_Drivetrain.getVisionEnabled())
      {
        // LOW SHOT W/ VISION
        if(this.m_FreshCommand)
        {
          if(this.noteBackward())
          {
            m_ShooterTopRollerVelocity = Positions.LOW_SHOT.SHOOTER_ROLLER_1_VELOCITY;
            m_ShooterBottomRollerVelocity = Positions.LOW_SHOT.SHOOTER_ROLLER_2_VELOCITY;
            this.m_FreshCommand = false;
          }
        }
  
        m_ElevatorPosition = Positions.LOW_SHOT.ELEVATOR_POSITION;
        m_IntakePosition = Positions.LOW_SHOT.INTAKE_WRIST_POSITION;
        m_IntakeRollersVoltage = Positions.LOW_SHOT.INTAKE_ROLLER_VOLTAGE;
  
        double i = m_Drivetrain.distanceToSpeaker.get();
          // 0.738 + -0.0567x + 5.12E-03x^2
          double shooterAngle = 0.738 + (-0.0567 * i) + (5.12*Math.pow(10, -3)*Math.pow(i, 2));
          //double shooterAngle = SmartDashboard.getNumber("[Manual] Shooter Angle", 0.6);
          m_ShooterAngle = MathUtil.clamp(shooterAngle, 0.55, Positions.LOW_BUMPER_SHOT.SHOOTER_WRIST_ANGLE);
        if(m_Elevator.isInPosition(m_ElevatorPosition))
        {
          if (this.m_Shooter.readyToShoot(MathUtil.clamp(shooterAngle, 0.55, Positions.LOW_BUMPER_SHOT.SHOOTER_WRIST_ANGLE), 
                                          Positions.LOW_SHOT.SHOOTER_ROLLER_1_VELOCITY,
                                          Positions.LOW_SHOT.SHOOTER_ROLLER_2_VELOCITY)
                                          && m_Drivetrain.getAligned())
          {
            m_ShooterFeederVoltage = Positions.LOW_SHOT.SHOOTER_FEEDER_VOLTAGE;
          }
        }
      }
      else
      {
        // BACKUP BUMPER SHOT
        if(this.m_FreshCommand)
        {
          if(this.noteBackward())
          {
            m_ShooterTopRollerVelocity = Positions.LOW_BUMPER_SHOT.SHOOTER_ROLLER_1_VELOCITY;
            m_ShooterBottomRollerVelocity = Positions.LOW_BUMPER_SHOT.SHOOTER_ROLLER_2_VELOCITY;
            this.m_FreshCommand = false;
          }
        }
  
        m_ElevatorPosition = Positions.LOW_BUMPER_SHOT.ELEVATOR_POSITION;
        m_IntakePosition = Positions.LOW_BUMPER_SHOT.INTAKE_WRIST_POSITION;
        m_IntakeRollersVoltage = Positions.LOW_BUMPER_SHOT.INTAKE_ROLLER_VOLTAGE;
  
        if(m_Elevator.isInPosition(m_ElevatorPosition))
        {
          m_ShooterAngle = Positions.LOW_BUMPER_SHOT.SHOOTER_WRIST_ANGLE;
          if (this.m_Shooter.readyToShoot(Positions.LOW_BUMPER_SHOT.SHOOTER_WRIST_ANGLE, 
                                          Positions.LOW_BUMPER_SHOT.SHOOTER_ROLLER_1_VELOCITY,
                                          Positions.LOW_BUMPER_SHOT.SHOOTER_ROLLER_2_VELOCITY))
          {
            m_ShooterFeederVoltage = Positions.LOW_BUMPER_SHOT.SHOOTER_FEEDER_VOLTAGE;
          }
        }
      }
      
    }
    return false;
  }

  // Returns true when no note is detected anymore.
  public boolean shootHigh()
  {
    if(m_Shooter.shooterClear())
    {
      m_ShooterTopRollerVelocity = 0;
      m_ShooterBottomRollerVelocity = 0;
      m_ShooterFeederVoltage = 0;
      this.stow();
      return true;
    }

    if(elevatorUp())
    {

      m_ElevatorPosition = Positions.DEFENSE_SHOT.ELEVATOR_POSITION;
      m_IntakePosition = Positions.DEFENSE_SHOT.INTAKE_WRIST_POSITION;
      m_IntakeRollersVoltage = Positions.DEFENSE_SHOT.INTAKE_ROLLER_VOLTAGE;
      m_ShooterAngle = Positions.DEFENSE_SHOT.SHOOTER_WRIST_ANGLE;


      if(m_Elevator.isInPosition(m_ElevatorPosition))
      {
        if(this.m_FreshCommand)
        {
          if(this.noteBackward())
          {
            m_ShooterTopRollerVelocity = Positions.DEFENSE_SHOT.SHOOTER_ROLLER_1_VELOCITY;
            m_ShooterBottomRollerVelocity = Positions.DEFENSE_SHOT.SHOOTER_ROLLER_2_VELOCITY;
            this.m_FreshCommand = false;
          }
        }

        if (this.m_Shooter.readyToShoot(Positions.DEFENSE_SHOT.SHOOTER_WRIST_ANGLE, 
                                        Positions.DEFENSE_SHOT.SHOOTER_ROLLER_1_VELOCITY,
                                        Positions.DEFENSE_SHOT.SHOOTER_ROLLER_2_VELOCITY))
        {
          m_ShooterFeederVoltage = Positions.DEFENSE_SHOT.SHOOTER_FEEDER_VOLTAGE;
        }
      }
    }
    return false;
  }

  // If no vision data is available, this will revert to a basic command that shoots amp when all positions are met.
  // Returns true when no note is detected anymore.
  // Need a way to track....
  public boolean shootAmp()
  {
    if(m_Shooter.shooterClear())
    {
      m_ShooterTopRollerVelocity = 0;
      m_ShooterBottomRollerVelocity = 0;
      m_ShooterFeederVoltage = 0;
      this.stow();
      return true;
    }

    if(elevatorUp())
    {
      if(this.m_FreshCommand)
      {
        if(this.noteForward())
        {
          this.m_FreshCommand = false;
        }
      }
      else
      {
        m_ShooterFeederVoltage = Positions.AMP.SHOOTER_FEEDER_VOLTAGE;
      }

      m_ElevatorPosition = Positions.AMP.ELEVATOR_POSITION;
      m_IntakePosition = Positions.AMP.INTAKE_WRIST_POSITION;
      m_IntakeRollersVoltage = Positions.AMP.INTAKE_ROLLER_VOLTAGE;
      m_ShooterAngle = Positions.AMP.SHOOTER_WRIST_ANGLE;

      if(m_Elevator.isInPosition(m_ElevatorPosition))
      {
        if (this.m_Shooter.inPosition(Positions.AMP.SHOOTER_WRIST_ANGLE))
        {
          m_ShooterTopRollerVelocity = Positions.AMP.SHOOTER_ROLLER_1_VELOCITY;
          m_ShooterBottomRollerVelocity = Positions.AMP.SHOOTER_ROLLER_2_VELOCITY;

        }
      }
    }

    // if note is forward
    if (m_Shooter.hasNoteForwardPosition() && m_FreshCommand)
    {
      // command is no longer fresh
      this.m_FreshCommand = false;
      m_ShooterFeederVoltage = 0;
      m_ShooterBottomRollerVelocity = 0;
      m_ShooterTopRollerVelocity = 0;
    }

    // if command is fresh (just started)
    if (this.m_FreshCommand && (m_Elevator.isAboveBar() || m_Shooter.inPosition(ShooterConstants.WRIST_CLEARANCE_POSITION)))
    {
      // move note forward
      noteForward();
    }

    return false;
  }

  // Prepares the amp shot, returning true when amp shot is ready to be taken
  public boolean prepareAmp()
  {
    if(m_Shooter.shooterClear())
    {
      m_ShooterTopRollerVelocity = 0;
      m_ShooterBottomRollerVelocity = 0;
      m_ShooterFeederVoltage = 0;
      this.stow();
      return true;
    }

    if(elevatorUp())
    {
      if(this.m_FreshCommand)
      {
        if(this.noteForward())
        {
          this.m_FreshCommand = false;
        }
      }
      else
      {
        m_ShooterFeederVoltage = Positions.AMP.SHOOTER_FEEDER_VOLTAGE;
      }

      m_ElevatorPosition = Positions.AMP.ELEVATOR_POSITION;
      m_IntakePosition = Positions.AMP.INTAKE_WRIST_POSITION;
      m_IntakeRollersVoltage = Positions.AMP.INTAKE_ROLLER_VOLTAGE;
      m_ShooterAngle = Positions.AMP.SHOOTER_WRIST_ANGLE;

      if(m_Elevator.isInPosition(m_ElevatorPosition))
      {
        if (this.m_Shooter.inPosition(Positions.AMP.SHOOTER_WRIST_ANGLE))
        {
          if (this.m_ShootAmp)
          {
            m_ShooterTopRollerVelocity = Positions.AMP.SHOOTER_ROLLER_1_VELOCITY;
            m_ShooterBottomRollerVelocity = Positions.AMP.SHOOTER_ROLLER_2_VELOCITY;
          }
        }
      }
    }

    // if note is forward
    if (m_Shooter.hasNoteForwardPosition() && m_FreshCommand)
    {
      // command is no longer fresh
      this.m_FreshCommand = false;
      m_ShooterFeederVoltage = 0;
      m_ShooterBottomRollerVelocity = 0;
      m_ShooterTopRollerVelocity = 0;
    }

    // if command is fresh (just started)
    if (this.m_FreshCommand && (m_Elevator.isAboveBar() || m_Shooter.inPosition(ShooterConstants.WRIST_CLEARANCE_POSITION)))
    {
      // move note forward
      noteForward();
    }

    return false;
  }

  // just runs motors
  public void shootAmpWithPrep()
  {
    m_ShooterTopRollerVelocity = Positions.AMP.SHOOTER_ROLLER_1_VELOCITY;
    m_ShooterBottomRollerVelocity = Positions.AMP.SHOOTER_ROLLER_2_VELOCITY;
  }

  public void setAmpShoot(boolean shoot)
  {
    this.m_ShootAmp = shoot;
  }

  // sends note out the back and runs intake forward
  public void purge()
  {
    if(elevatorDown())
    {
      m_ElevatorPosition = Positions.PURGE.ELEVATOR_POSITION;
      m_IntakePosition = Positions.PURGE.INTAKE_WRIST_POSITION;
      m_IntakeRollersVoltage = Positions.PURGE.INTAKE_ROLLER_VOLTAGE;
      m_ShooterAngle = Positions.PURGE.SHOOTER_WRIST_ANGLE;

      if(m_Elevator.isInPosition(Positions.PURGE.ELEVATOR_POSITION))
      {
        if (m_Shooter.inPosition(Positions.PURGE.SHOOTER_WRIST_ANGLE))
        {
          m_ShooterTopRollerVelocity = Positions.PURGE.SHOOTER_ROLLER_1_VELOCITY;
          m_ShooterBottomRollerVelocity = Positions.PURGE.SHOOTER_ROLLER_2_VELOCITY;
          m_ShooterFeederVoltage = Positions.PURGE.SHOOTER_FEEDER_VOLTAGE;
        }
      }
    }
  }

  public boolean climb()
  {
    m_ShooterFeederVoltage = Positions.CLIMB.SHOOTER_FEEDER_VOLTAGE;
    m_ShooterTopRollerVelocity = Positions.CLIMB.SHOOTER_ROLLER_1_VELOCITY;
    m_ShooterBottomRollerVelocity = Positions.CLIMB.SHOOTER_ROLLER_2_VELOCITY;
    m_IntakePosition = Positions.CLIMB.INTAKE_WRIST_POSITION;
    m_IntakeRollersVoltage = Positions.CLIMB.INTAKE_ROLLER_VOLTAGE;

    // if elevator is up all the way
    if(elevatorUp())
    {
      // set to clearance
      m_ShooterAngle = ShooterConstants.WRIST_CLEARANCE_POSITION;
      m_ElevatorPosition = Positions.CLIMB.ELEVATOR_POSITION;

      // if in position return true to end the command
      if(m_Shooter.inPosition(ShooterConstants.WRIST_CLEARANCE_POSITION) && m_Elevator.isInPosition(Positions.CLIMB.ELEVATOR_POSITION))
      {
        return true;
      }
    }
    return false;
  }

  public boolean elevatorUp()
  {
    if(m_Elevator.isAboveBar())
    {
      return true;
    }

    if(m_Elevator.isBelowBar())
    {
      m_IntakePosition = IntakeConstants.INTAKE_CLEAR_POSITION;
      if(m_ShooterAngle != ShooterConstants.WRIST_CLEARANCE_POSITION)
      {
        m_ElevatorPosition = ElevatorConstants.BAR_BOTTOM_CLEAR - 0.01;
        m_ShooterAngle = ShooterConstants.WRIST_CLEARANCE_POSITION;
      }
      if(m_Shooter.inPosition(ShooterConstants.WRIST_CLEARANCE_POSITION))
      {
        m_ElevatorPosition = ElevatorConstants.BAR_TOP_CLEAR + 0.005;
      }
    }
    else
    {
      if(m_ShooterAngle != ShooterConstants.WRIST_CLEARANCE_POSITION)
      {
        if(m_ElevatorPositionSignal.getValueAsDouble() <= ElevatorConstants.BAR)
        {
          m_ElevatorPosition = ElevatorConstants.BAR_BOTTOM_CLEAR - 0.01;
          m_ShooterAngle = ShooterConstants.WRIST_CLEARANCE_POSITION;
        }
        else
        {
          m_ElevatorPosition = ElevatorConstants.BAR_TOP_CLEAR + 0.005;
        }
      }
    }
    return false;
  }

  public boolean elevatorDown()
  {
    if(m_Elevator.isBelowBar())
    {
      return true;
    }
    
    if(m_Elevator.isAboveBar())
    {
      m_IntakePosition = IntakeConstants.INTAKE_CLEAR_POSITION;
      if(m_ShooterAngle != ShooterConstants.WRIST_CLEARANCE_POSITION)
      {
        m_ElevatorPosition = ElevatorConstants.BAR_TOP_CLEAR + 0.005;
        m_ShooterAngle = ShooterConstants.WRIST_CLEARANCE_POSITION;
      }
      if(m_Shooter.inPosition(this.m_ShooterAngle))
      {
        m_ElevatorPosition = ElevatorConstants.BAR_BOTTOM_CLEAR - 0.01;
      }
    }
    else
    {
      if(m_ShooterAngle != ShooterConstants.WRIST_CLEARANCE_POSITION)
      {
        if(m_ElevatorPositionSignal.getValueAsDouble() >= ElevatorConstants.BAR)
        {
          m_ElevatorPosition = ElevatorConstants.BAR_TOP_CLEAR + 0.005;
        }
        else
        {
          m_ElevatorPosition = ElevatorConstants.BAR_BOTTOM_CLEAR - 0.01;
        }
      }
      else
      {
        if(!m_Shooter.inPosition(ShooterConstants.WRIST_CLEARANCE_POSITION))
        {
          m_ElevatorPosition = ElevatorConstants.BAR_TOP_CLEAR + 0.05;
        }
      }
    }
    return false;
  }

  // Note movements
  private boolean noteForward()
  {
    if(!this.m_Shooter.hasNoteForwardPosition())
    {
      m_ShooterFeederVoltage = 3;
      m_ShooterBottomRollerVelocity = 10;
      m_ShooterTopRollerVelocity = 10;
      return false;
    }
    m_ShooterFeederVoltage = 0;
    m_ShooterBottomRollerVelocity = 0;
    m_ShooterTopRollerVelocity = 0;
    return true;
  }

  // Add to positions
  private boolean noteBackward()
  {
    if(!this.m_Shooter.hasNoteRearPosition())
    {
      m_ShooterFeederVoltage = -3;
      m_ShooterBottomRollerVelocity = -10;
      m_ShooterTopRollerVelocity = -10;
      return false;
    }
    m_ShooterFeederVoltage = 0;
    m_ShooterBottomRollerVelocity = 0;
    m_ShooterTopRollerVelocity = 0;
    return true;
  }

  private boolean noteToMiddle()
  {
    if(this.m_Shooter.hasNoteCentered())
    {
      m_ShooterFeederVoltage = 0;
      m_ShooterBottomRollerVelocity = 0;
      m_ShooterTopRollerVelocity = 0;
      return true;
    }
    if(m_Shooter.hasNoteForwardPosition())
    {
      m_ShooterFeederVoltage = -2.5;
      m_ShooterBottomRollerVelocity = -5;
      m_ShooterTopRollerVelocity = -5;
    }
    if(m_Shooter.hasNoteRearPosition())
    {
      m_ShooterFeederVoltage = 1.5;
      m_ShooterBottomRollerVelocity = 5;
      m_ShooterTopRollerVelocity = 5;
    }
    if(m_Shooter.shooterClear())
    {
      m_ShooterFeederVoltage = 0;
      m_ShooterBottomRollerVelocity = 0;
      m_ShooterTopRollerVelocity = 0;
    }
    return false;
  }
}
