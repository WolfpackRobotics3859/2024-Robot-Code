// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.elevator.ElevatorConstants;
import frc.robot.constants.intake.IntakeConstants;
import frc.robot.constants.shooter.ShooterConstants;
import frc.robot.constants.shooter.ShooterConstants.MOTOR;
import frc.robot.statemachines.OrchestratorStateMachine;
import frc.robot.statemachines.OrchestratorStateMachine.DesiredAction;
import frc.robot.statemachines.OrchestratorStateMachine.SystemState;
import frc.robot.statemachines.OrchestratorStateMachine.SystemState.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Orchestrator extends SubsystemBase
{
  // Subsystems
  private Drivetrain m_Drivetrain;
  private Shooter m_Shooter;
  private Elevator m_Elevator;
  private Intake m_Intake;

  // Timer
  private final Timer m_Timer;

  // Drivetrain
  private Pose2d m_RobotPose;
  
  // Shooter
  private double m_ShooterMotor1Velocity;
  private double m_ShooterMotor2Velocity;
  private double m_ShooterFeederVelocity;
  private double m_ShooterWristPosition;

  // Elevator
  private double m_ElevatorAbsolutePosition;

  // Intake
  private double m_IntakeWristPosition;

  // Orchestrator State Machine
  private OrchestratorStateMachine m_StateMachine;

  /**
   * Creates a new Orchestrator subsystem
   * @param drivetrain The drivetrain subsystem
   * @param shooter The shooter subsystem
   * @param elevator The elevator subsytem
   * @param intake The intake subsystem
   */
  public Orchestrator(Drivetrain drivetrain, Shooter shooter, Elevator elevator, Intake intake)
  {
    this.m_Drivetrain = drivetrain;
    this.m_Shooter = shooter;
    this.m_Elevator = elevator;
    this.m_Intake = intake;

    this.m_StateMachine = new OrchestratorStateMachine(m_Drivetrain, m_Elevator, m_Intake, m_Shooter);
    this.m_Timer = new Timer();
  }
  
  // Odometry Fun(ctions) REMOVE LATER

  /**
   * Provides the Orchestrator with drivetrain odometry
   * @param robotPose The current position of the robot on the field
   */
  public void updateDriveOdometry(Pose2d robotPose)
  {
    this.m_RobotPose = robotPose;
  }

  /**
   * Provides the Orchestrator with odometry for the elevator
   * @param elevatorGoalPosition Whether the elevator is currently up or down (or is trying to go up or down)
   * @param elevatorAbsolutePosition The absolute position of the elevator
   * @param movementError The distance between where the elevator wants to be and where it currently is
   */
  public void updateElevatorOdometry(double elevatorAbsolutePosition)
  {
    this.m_ElevatorAbsolutePosition = elevatorAbsolutePosition;
  }

  /**
   * Provides the Orchestrator with intake odometry
   * @param intakeWristPosition The intake wrist's current position
   */
  public void updateIntakeOdometry(double intakeWristPosition)
  {
    this.m_IntakeWristPosition = intakeWristPosition;
  }

  /**
   * Provides the Orchestrator with shooter odometry
   * @param motor1Velocity The current velocity of the first shooter motor
   * @param motor2Velocity The current velocity of the second shooter motor
   * @param feederVelocity The current velocity of the feeder motor
   * @param shooterWristPosition The current absolute position of the shooter wrist
   * @param beamBreak1 The status of the first beam break sensor
   * @param beamBreak2 The status of the second beam break sensor
   */
  public void updateShooterOdometry(double motor1Velocity, double motor2Velocity, double feederVelocity, double shooterWristPosition, boolean beamBreak1, boolean beamBreak2)
  {
    this.m_ShooterMotor1Velocity = motor1Velocity;
    this.m_ShooterMotor2Velocity = motor2Velocity;
    this.m_ShooterFeederVelocity = feederVelocity;
    this.m_ShooterWristPosition = shooterWristPosition;
  }

  // Util
  public boolean isAtPosition(double currentPosition, double goalPosition, double tolerance)
  {
    return (currentPosition - tolerance <= goalPosition) && (currentPosition + tolerance >= goalPosition);
  }

  public void setDesiredAction(DesiredAction action)
  {
    this.m_StateMachine.m_DesiredAction = action;
  }

  @Override
  public void periodic()
  {
    // update odometry
    // ELEVATOR
    m_ElevatorAbsolutePosition = m_Elevator.getElevatorPosition().getValueAsDouble();

    // SHOOTER
    m_ShooterWristPosition = m_Shooter.getWristMotorPosition().getValueAsDouble();
    m_ShooterMotor1Velocity = m_Shooter.getShooterMotorVelocity(MOTOR.MOTOR_1).getValueAsDouble();
    m_ShooterMotor2Velocity = m_Shooter.getShooterMotorVelocity(MOTOR.MOTOR_2).getValueAsDouble();
    m_ShooterFeederVelocity = m_Shooter.getShooterMotorVelocity(MOTOR.FEEDER_MOTOR).getValueAsDouble();

    // INTAKE
    m_IntakeWristPosition = m_Intake.getWristPosition().getValueAsDouble();

    // ROBOT
    m_RobotPose = m_Drivetrain.getOdometry().getEstimatedPosition();
    
    // update the robot state
    m_StateMachine.update();

    SystemState m_State = m_StateMachine.getCurrentState();

    // put data to smart dashboard
    SmartDashboard.putString("Desired Action", m_StateMachine.m_DesiredAction.toString());
    SmartDashboard.putString("Robot State", m_State.state.toString());
    SmartDashboard.putBoolean("Robot has note", m_State.hasNote);
    SmartDashboard.putBoolean("Shot ready", m_State.shotReady);
    SmartDashboard.putString("Elevator Goal Position", m_State.elevatorGoalPosition.toString());

    SmartDashboard.putBoolean("balls", !isAtPosition(m_ShooterWristPosition, ShooterConstants.WRIST_DEFAULT_POSITION, ShooterConstants.WRIST_MOVEMENT_TOLERANCE));

    // have distance variable
    double distance = 10;

    double shooterPosition = 0;

    // determine what to do with robot based on the current robot state
    switch (m_State.state)
    {
      // if robot is set to be idle make sure intake is up and off
      case IDLE:
        // if intake is down
        if(!isAtPosition(m_IntakeWristPosition, IntakeConstants.INTAKE_UP_POS, 2))
        {
          // set intake up and turn intake motor off
          // m_Intake.setWristPosition(IntakeConstants.INTAKE_UP_POS);
          m_Intake.setRollerPercent(0);
        }

        // if shooter motors are spinning set them to be off
        if (Math.abs(m_ShooterFeederVelocity) > 0 || Math.abs(m_ShooterMotor1Velocity) > 0 || Math.abs(m_ShooterMotor2Velocity) > 0)
        {
          m_Shooter.setShooterMotorPercent(MOTOR.MOTOR_1, 0);
          m_Shooter.setShooterMotorPercent(MOTOR.MOTOR_2, 0);
        }

        // if shooter wrist is not at default position set it to default position
        if (!isAtPosition(m_ShooterWristPosition, ShooterConstants.WRIST_DEFAULT_POSITION, ShooterConstants.WRIST_MOVEMENT_TOLERANCE))
        {
          m_Shooter.setWristPosition(ShooterConstants.WRIST_DEFAULT_POSITION);
        }

        if(m_State.hasNote)
        {
          m_Shooter.setMotorVelocity(MOTOR.FEEDER_MOTOR, 0);
        }
        
        break;
      
      // if elevator is going to the down position
      case ELEVATOR_DOWN:
        // if elevator is not already down
        if (!isAtPosition(m_ElevatorAbsolutePosition, ElevatorConstants.ELEVATOR_BOTTOM_POSITION, ElevatorConstants.ELEVATOR_POSITION_TOLERANCE)) 
        {
          // check if elevator is above bar
          if (m_ElevatorAbsolutePosition >= 0.045)
          {
            // set elevator to top position to be 100% sure that the wrist will clear
            m_Elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_TOP_POSITION);
            // if elevator at top position
            if (isAtPosition(m_ElevatorAbsolutePosition, ElevatorConstants.ELEVATOR_TOP_POSITION, ElevatorConstants.ELEVATOR_POSITION_TOLERANCE))
            {
              // send wrist to clearance position
              m_Shooter.setWristPosition(ShooterConstants.WRIST_CLEARANCE_POSITION);
            }
          
            // if wrist is at clearance position
            if (isAtPosition(m_ShooterWristPosition, ShooterConstants.WRIST_CLEARANCE_POSITION, ShooterConstants.WRIST_MOVEMENT_TOLERANCE))
            {
              // send elevator down
              m_Elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_BOTTOM_POSITION);
            }
          }
          // if elevator is below bar
          else 
          {
            // set elevator to bottom position and set shooter to flat
            m_Elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_BOTTOM_POSITION);
            m_Shooter.setWristPosition(ShooterConstants.WRIST_DEFAULT_POSITION);
          }
        }
        // if elevator is at proper position
        else
        {
          // if intake is queued after elevator movement
          if (m_State.intakeQueued)
          {
            // set desired action to intake and remove intaking from queue
            m_StateMachine.m_DesiredAction = DesiredAction.INTAKE;
            m_State.intakeQueued = false;
          }
          // otherwise set back to idle
          else
          {
            m_StateMachine.m_DesiredAction = DesiredAction.IDLE;
          }
        }
        break;

      // if elevator is being sent up
      case ELEVATOR_UP:
        // if elevator is not already up
        if (!isAtPosition(m_ElevatorAbsolutePosition, ElevatorConstants.ELEVATOR_TOP_POSITION, ElevatorConstants.ELEVATOR_POSITION_TOLERANCE)) 
        {
          // check if elevator is below bar
          if (m_ElevatorAbsolutePosition <= 0.5)
          {
            // send elevator up
            m_Elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_TOP_POSITION);
            // if elevator at or above clearance position
            if (m_ElevatorAbsolutePosition >= ElevatorConstants.ELEVATOR_BOTTOM_CLEARANCE_POSITION)
            {
              // send wrist to clearance position
              m_Shooter.setWristPosition(ShooterConstants.WRIST_CLEARANCE_POSITION);
            }
          }
          // if elevator is above bar
          else if (m_ElevatorAbsolutePosition >= 1.1)
          {
            // set elevator to top position and set shooter to flat
            m_Elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_TOP_POSITION);
            m_Shooter.setWristPosition(ShooterConstants.WRIST_DEFAULT_POSITION);
          }
        }
        // if elevator is at proper position set back to idle
        else
        {
          m_StateMachine.m_DesiredAction = DesiredAction.IDLE;
        }
        break;

      // if robot is being requested to intake
      case INTAKING:
        // if elevator is not already at the proper position
        if (!isAtPosition(m_ElevatorAbsolutePosition, ElevatorConstants.ELEVATOR_BOTTOM_POSITION, ElevatorConstants.ELEVATOR_POSITION_TOLERANCE))
        {
          // set desired action to bring the elevator down and tell robot to queue an intake after elevator is down 
          m_StateMachine.m_DesiredAction = DesiredAction.ELEVATOR_DOWN;
          m_State.intakeQueued = true;
        }
        // if elevator is where it is supposed to be
        else
        {
          // if shooter motors not up to speed
          if (!isAtPosition(m_ShooterMotor1Velocity, -15, 2))
          {
            // set shooter motors + feeder to a slowish velocity inwards
            m_Shooter.setMotorVelocity(MOTOR.MOTOR_1, -15);
            m_Shooter.setMotorVelocity(MOTOR.MOTOR_2, -15);
            m_Shooter.setShooterMotorPercent(MOTOR.FEEDER_MOTOR, -0.3);
          }

          // if intake is not out and motors aren't on
          if (!isAtPosition(m_IntakeWristPosition, IntakeConstants.INTAKE_DOWN_POS, 2) || m_ShooterFeederVelocity < 1)
          {
            // set intake down and turn intake on
            m_Intake.setWristPosition(IntakeConstants.INTAKE_DOWN_POS);
            m_Intake.setRollerPercent(-0.4);
          } 
          // if all is well
          else
          {
            // if beam break 2 is unbroken (true) and beam break 1 is broken (false)
            if(m_Shooter.getBeamBreak2() && !m_Shooter.getBeamBreak1())
            {
              // set robot to idle and tell robot we have note
              m_State.state = RobotState.IDLE;
              m_StateMachine.m_DesiredAction = DesiredAction.IDLE;
              m_State.hasNote = true;
            }
          }
        }
        break;
      case SHOOTING:

        /**
         * distance(unit) - position on shooter
         * 
         */
        if(distance == 10)
        {
          shooterPosition = 0.7; // replace number
        } // add statements for each distance
        
        // if elevator is at shooting position
        if(isAtPosition(m_ElevatorAbsolutePosition, ElevatorConstants.ELEVATOR_SHOOTING_POSITION, ElevatorConstants.ELEVATOR_POSITION_TOLERANCE))
        {
          // if shooter wrist is not at proper position
          if(!isAtPosition(m_ShooterWristPosition, shooterPosition , 0.05))
          {
            // set to shooting position
            m_Shooter.setWristPosition(shooterPosition);
          }
          // shooter is at proper position
          else
          {
            // if shooter motors are not at velocity
            if(!isAtPosition(m_ShooterMotor1Velocity, 10, 3) || !isAtPosition(m_ShooterMotor2Velocity, 30, 3))
            {
              // set to velocity
              m_Shooter.setMotorVelocity(MOTOR.MOTOR_1, 10);
              m_Shooter.setMotorVelocity(MOTOR.MOTOR_2, 10);
            }
          }
        } 
        // if elevator not at shooting position
        else
        {
          // set to shooting position
          m_Elevator.setElevatorPosition(ElevatorConstants.ELEVATOR_SHOOTING_POSITION);
        }

        // if shooter is at velocity and position
        if(isAtPosition(m_ShooterWristPosition, shooterPosition, ShooterConstants.WRIST_MOVEMENT_TOLERANCE) && isAtPosition(m_ShooterMotor1Velocity, 10, 3) && isAtPosition(m_ShooterMotor2Velocity, 10, 3) && isAtPosition(m_ElevatorAbsolutePosition, ElevatorConstants.ELEVATOR_SHOOTING_POSITION, ElevatorConstants.ELEVATOR_POSITION_TOLERANCE))
        {
          // shoot
          m_Shooter.setShooterMotorPercent(MOTOR.FEEDER_MOTOR, 0.2);
        }
        break; // break my skull ahahah
      
      default:
        break;
    }
    
  }
}

