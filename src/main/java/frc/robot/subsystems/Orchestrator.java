// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.elevator.ElevatorConstants;
import frc.robot.constants.intake.IntakeConstants;
import frc.robot.constants.shooter.ShooterConstants;
import frc.robot.constants.shooter.ShooterConstants.MOTOR;
import frc.robot.utils.Util;

public class Orchestrator extends SubsystemBase
{
  /**
   * FOR V3 ORCHESTRATOR:
   * V3 should be a completely new framework. All logic is to be done within the orchestrator, with
   * logic for each state (RobotState) done within it's own function in the orchestrator.
   * Then, the orchestrator should be updating public numbers based on this logic that correspond to motor
   * positions and velocities in order to make all the subsystems work together.
   * 
   * This data is fed to the subsystems via the PlayAlong commands, which simply set motor positions and velocities
   * in their execute functions, while performing minimal logic that checks the amount of times we are sending motor
   * requests over the CAN bus to prevent over-utilization.
   * 
   * Finally, each of the methods pertaining to each state are run within the corresponding state commands,
   * which simply run the method in their execute function in order to update logic checks and get desired motor
   * positions and velocities to send to the subsystems over the playalong commands.
   */

  private final Drivetrain m_Drivetrain; // this will be used later once we have auto aiming/pathplanner
  private final Elevator m_Elevator;
  private final Shooter m_Shooter;
  private final Intake m_Intake;

  private final Timer m_Timer;
  private double m_odometryUpdateFrequency = 0.1;

  public RobotState m_CurrentState = RobotState.DISABLED; // set robot to be disabled first
  public boolean noteGrabbed = false;
  public boolean noteStowed = false;
  public boolean shotReady = false;
  public boolean shotRequested = false; // if the robot is being told to actually fire the note once the shot is ready

  /**
   * 
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
  public double m_DesiredShooterFeederVelocity;

  public double m_DesiredIntakeWristPosition;
  public double m_DesiredIntakeRollersVelocity;

  // Getters/Setters (to be deleted)
  public void setOrchestratorState(RobotState state)
  {
    this.m_CurrentState = state;
  }

  public void setShotReady(boolean ready)
  {
    this.shotReady = ready;
  }

  // High Level methods

  public void Stow()
  {
    // this should contain logic to decide the positions and velocities of all the components of the 
    // individual subsystems, handling all conflicts

    /**
     * check if shooter is above intake -> if so, set desired intake position to be clearing
     * 
     * check if elevator is above bar:
     * - if above bar, set elevator up to top position, then set shooter to clearance, then move down once shooter is cleared
     * - if elevator is below bar, just send elevator down
     * - if shooter is below a certain position set back to flat
    */

    // set all wheels to off
    this.m_DesiredShooterMotor1Velocity = 0;
    this.m_DesiredShooterMotor2Velocity = 0;
    this.m_DesiredIntakeRollersVelocity = 0;
    this.m_DesiredShooterFeederVelocity = 0;

    // if elevator is above the position where the intake needs to move
    if (m_ElevatorPosition > ElevatorConstants.ELEVATOR_INTAKE_CLEAR_POSITION)
    {
      // set intake position to the clearance position
      this.m_DesiredIntakeWristPosition = IntakeConstants.INTAKE_CLEAR_POSITION;
    }
    else
    {
      this.m_DesiredIntakeWristPosition = IntakeConstants.INTAKE_UP_POSITION;
    }

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
      m_DesiredShooterWristPosition = ShooterConstants.WRIST_DEFAULT_POSITION;
      // set elevator to bottom position
      m_DesiredElevatorPosition = ElevatorConstants.ELEVATOR_BOTTOM_POSITION;
      // set wrist flat
    }
  }

  // intake logic
  public void Intake()
  {
    // set intake down
    m_DesiredIntakeWristPosition = IntakeConstants.INTAKE_DOWN_POSITION;

    // if elevator is above bar
    if (m_ElevatorPosition > ElevatorConstants.ELEVATOR_BAR_POSITION)
    {
      // send elevator up
      m_DesiredElevatorPosition = ElevatorConstants.ELEVATOR_TOP_POSITION;
      // once elevator is up
      if (Util.epsilonEquals(m_ElevatorPosition, ElevatorConstants.ELEVATOR_TOP_POSITION, ElevatorConstants.ELEVATOR_TOP_POSITION))
      {
        // send elevator down
        m_DesiredElevatorPosition = ElevatorConstants.ELEVATOR_BOTTOM_POSITION;
      }
    }
    // if elevator is down
    else
    {
      // set elevator to bottom position
      m_DesiredElevatorPosition = ElevatorConstants.ELEVATOR_BOTTOM_POSITION;
      // set wrist flat
      m_DesiredShooterWristPosition = ShooterConstants.WRIST_DEFAULT_POSITION;
      
      if (Util.epsilonEquals(m_DesiredShooterWristPosition, ShooterConstants.WRIST_DEFAULT_POSITION, ShooterConstants.WRIST_MOVEMENT_TOLERANCE))
      {
        m_DesiredIntakeRollersVelocity = -30;
        m_DesiredShooterMotor1Velocity = -30;
        m_DesiredShooterMotor2Velocity = -30;
        m_DesiredShooterFeederVelocity = -20;
      }
    }
  }

  public void ShootSpeakerUp()
  {
    // Empty for now
  }

  public void ShootSpeakerDown()
  {
    // Empty for now
  }

  public void ShootAmp()
  {
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
      m_DesiredElevatorPosition = ElevatorConstants.ELEVATOR_AMP_SHOT_POSITION;
      // set wrist flat
      if(Util.epsilonEquals(m_ElevatorPosition, ElevatorConstants.ELEVATOR_AMP_SHOT_POSITION, 0.006))
      {
        m_DesiredShooterWristPosition = ShooterConstants.WRIST_AMP_SHOOTING_POSITION;
      }

      if(Util.epsilonEquals(m_ShooterWristPosition, ShooterConstants.WRIST_AMP_SHOOTING_POSITION, ShooterConstants.WRIST_MOVEMENT_TOLERANCE))
      {

        m_DesiredShooterMotor1Velocity = 12;
        m_DesiredShooterMotor2Velocity = 19;
      }

      if(Util.epsilonEquals(m_ShooterMotor2Velocity, 13, ShooterConstants.SHOOTER_VELOCITY_TOLERANCE))
      {
        m_DesiredShooterFeederVelocity = 30;
      }
      
    }
  }

  public void BumberShot()
  {
    // set shooter wheels to bumper shot speed
    this.m_DesiredShooterMotor1Velocity = ShooterConstants.BUMPER_SHOT_VELOCITY;
    this.m_DesiredShooterMotor2Velocity = ShooterConstants.BUMPER_SHOT_VELOCITY;
    this.m_DesiredIntakeRollersVelocity = 0;
    this.m_DesiredShooterFeederVelocity = 0;

    this.m_DesiredIntakeWristPosition = IntakeConstants.INTAKE_BUMPER_SHOT_POSITION;

    // if elevator is above bar
    if (m_ElevatorPosition > ElevatorConstants.ELEVATOR_BAR_POSITION)
    {
      // send elevator up
      m_DesiredElevatorPosition = ElevatorConstants.ELEVATOR_TOP_POSITION;
      // once elevator is up
      if (Util.epsilonEquals(m_ElevatorPosition, ElevatorConstants.ELEVATOR_TOP_POSITION, ElevatorConstants.ELEVATOR_TOP_POSITION))
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
            m_DesiredShooterFeederVelocity = 30;
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
      m_DesiredIntakeWristPosition = IntakeConstants.INTAKE_CLEAR_POSITION;
      
      // if elevator is below position
      if(m_ElevatorPosition < ElevatorConstants.ELEVATOR_BAR_POSITION - 0.06)
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
    // periodic function should be used for updating intake status (whether robot has note) and some logging
    // periodic should also check if robot is within range of the speaker to determine if the robot should begin aiming at the speaker

    // V3 PERIODIC:
    // Periodic should be used for updating odometry values from the subsystems.
    // This should be done via the getters in the subsystems and should do it on a (variable) timer for
    // a reduction in the canbus utilization

    if (m_Timer.get() > m_odometryUpdateFrequency)
    {
      m_Timer.reset();

      this.m_ElevatorPosition = m_Elevator.getElevatorPosition().getValueAsDouble();

      this.m_ShooterWristPosition = m_Shooter.getWristMotorPosition().getValueAsDouble();
      this.m_ShooterMotor1Velocity = m_Shooter.getShooterMotorVelocity(MOTOR.MOTOR_1).getValueAsDouble();
      this.m_ShooterMotor2Velocity = m_Shooter.getShooterMotorVelocity(MOTOR.MOTOR_2).getValueAsDouble();

      this.m_IntakeWristPosition = m_Intake.getWristPosition().getValueAsDouble();
    }

    SmartDashboard.putData(this);
    SmartDashboard.putNumber("Elevator Desired Position", m_DesiredElevatorPosition);
    SmartDashboard.putNumber("Shooter Wrist Desired Position", m_DesiredShooterWristPosition);
    SmartDashboard.putNumber("Intake Wrist Desired Position", m_DesiredIntakeWristPosition);

    // minor logging only

    if (!m_Shooter.getBeamBreak2())
    {
      this.noteStowed = true;
    }

    if (m_Shooter.getBeamBreak2())
    {
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
}
