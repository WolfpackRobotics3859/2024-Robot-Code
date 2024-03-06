// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Global;
import frc.robot.constants.Positions;
import frc.robot.constants.elevator.ElevatorConstants;
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

  private boolean m_IgnoreNotePosition = false;

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
    this.Setup();

    m_ElevatorPositionSignal = m_Elevator.getPositionSignal();

    SmartDashboard.setDefaultNumber("[Manual] Top Roller Velocity", 0);
    SmartDashboard.setDefaultNumber("[Manual] Bottom Roller Velocity", 0);
    SmartDashboard.setDefaultNumber("[Manual] Feeder Roller Voltage", 0);
    SmartDashboard.setDefaultNumber("[Manual] Shooter Angle", 0.45);
    SmartDashboard.setDefaultNumber("[Manual] Elevator Position", 0.67);
    SmartDashboard.setDefaultNumber("[Manual] Intake Position", 0.5);
    SmartDashboard.setDefaultNumber("[Manual] Intake Voltage", 0);
  }

  private void Setup()
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

  @Override
  public void periodic()
  {
    // if(Global.ENABLE_TELEMETRY)
    // {
    //   if(m_TelemetryTimer.get() > Global.TELEMETRY_UPDATE_SPEED)
    //   {
    //     // Intentionally Empty
    //     m_TelemetryTimer.reset();
    //   }
    // }

    // if(Global.ENABLE_EXTRA_TELEMETRY)
    // {
    //   if(m_ExtraTelemetryTimer.get() > Global.EXTRA_TELEMETRY_UPDATE_SPEED)
    //   {
    //     // Intentioally Empty
    //     m_ExtraTelemetryTimer.reset();
    //   }
    // }
  }

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

  public void manualControl()
  {
    m_ShooterTopRollerVelocity = SmartDashboard.getNumber("[Manual] Top Roller Velocity", 0);
    m_ShooterBottomRollerVelocity = SmartDashboard.getNumber("[Manual] Top Roller Velocity", 0);
    // m_ShooterBottomRollerVelocity = SmartDashboard.getNumber("[Manual] Bottom Roller Velocity", 0);
    m_ShooterFeederVoltage = SmartDashboard.getNumber("[Manual] Feeder Roller Voltage", 0);
    m_ShooterAngle = SmartDashboard.getNumber("[Manual] Shooter Angle", 0);
    m_ElevatorPosition = SmartDashboard.getNumber("[Manual] Elevator Position", 0.65);
    m_IntakePosition = SmartDashboard.getNumber("[Manual] Intake Position", 0.5);
    m_IntakeRollersVoltage = SmartDashboard.getNumber("[Manual] Intake Voltage", 0);
  }

  // Should include the ability to kill motors once robot is safely stowed.
  public void stow()
  {
    if(elevatorDown())
    {
      m_ElevatorPosition = Positions.STOW.ELEVATOR_POSITION;
      m_ShooterAngle = Positions.STOW.SHOOTER_WRIST_ANGLE;
      m_ShooterTopRollerVelocity = Positions.INTAKING.SHOOTER_ROLLER_1_VELOCITY;
      m_ShooterBottomRollerVelocity = Positions.INTAKING.SHOOTER_ROLLER_2_VELOCITY;
      m_ShooterFeederVoltage = Positions.INTAKING.SHOOTER_FEEDER_VOLTAGE;
      m_IntakePosition = Positions.INTAKING.INTAKE_WRIST_POSITION;
      m_IntakeRollersVoltage = Positions.INTAKING.INTAKE_ROLLER_VOLTAGE;
    }
  }

  public void intake()
  {
    if(m_Shooter.hasNoteRearPosition())
    {
      m_ShooterTopRollerVelocity = 0;
      m_ShooterBottomRollerVelocity = 0;
      m_ShooterFeederVoltage = 0;
      m_IntakeRollersVoltage = 0;
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
    // Intentionally Empty
    if(elevatorDown())
    {
      // Empty for now.
    }
    return false;
  }

  // If no vision data is available, this will default to bumper shot.
  // If the user is outside of range, this will do nothing
  // Returns true when no note is detected anymore.
  public boolean shootHigh()
  {
    // Empty for now.
    if(elevatorUp())
    {
      // Empty for now.
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
      return true;
    }
    if(m_Shooter.hasNoteRearPosition())
    {

    }
    return false;
  }

  public void purge()
  {
    // Empty for now.
  }

  public boolean elevatorUp()
  {
    if(m_Elevator.isAboveBar())
    {
      return true;
    }
    else if(m_Elevator.isBelowBar())
    {
      if(m_ShooterAngle != ShooterConstants.WRIST_CLEARANCE_POSITION)
      {
        m_ElevatorPosition = ElevatorConstants.BAR_BOTTOM_CLEAR - 0.01;
        m_ShooterAngle = ShooterConstants.WRIST_CLEARANCE_POSITION;
      }
      if(m_Shooter.inPosition())
      {
        m_ElevatorPosition = ElevatorConstants.BAR_TOP_CLEAR + 0.01;
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
      }
      m_ElevatorPosition = ElevatorConstants.BAR_TOP_CLEAR + 0.01;
    }
    return false;
  }

  public boolean elevatorDown()
  {
    if(m_Elevator.isBelowBar())
    {
      return true;
    }
    else if(m_Elevator.isAboveBar())
    {
      if(m_ShooterAngle != ShooterConstants.WRIST_CLEARANCE_POSITION)
      {
        m_ElevatorPosition = ElevatorConstants.BAR_TOP_CLEAR + 0.01;
        m_ShooterAngle = ShooterConstants.WRIST_CLEARANCE_POSITION;
      }
      if(m_Shooter.inPosition())
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
          m_ElevatorPosition = ElevatorConstants.BAR_TOP_CLEAR + 0.01;
        }
      }
      m_ElevatorPosition = ElevatorConstants.BAR_BOTTOM_CLEAR - 0.01;
    }
    return false;
  }
}
