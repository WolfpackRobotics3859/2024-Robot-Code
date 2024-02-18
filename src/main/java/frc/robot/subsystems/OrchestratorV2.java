// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.elevator.SetElevatorPosition;
import frc.robot.commands.intake.SetIntakeWristPosition;
import frc.robot.commands.shooter.SetFeederMotorPercent;
import frc.robot.commands.shooter.SetFeederMotorVelocity;
import frc.robot.commands.shooter.SetShooterMotorsVelocity;
import frc.robot.commands.shooter.SetWristMotorPercent;
import frc.robot.commands.shooter.SetWristMotorPosition;
import frc.robot.constants.elevator.ElevatorConstants;
import frc.robot.constants.shooter.ShooterConstants;
import frc.robot.constants.shooter.ShooterConstants.MOTOR;
import frc.robot.statemachines.OrchestratorStateMachine.DesiredAction;
import frc.robot.states.ElevatorState;
import frc.robot.states.IntakeState;
import frc.robot.states.ShooterState;
import frc.robot.utils.Util;

public class OrchestratorV2 extends SubsystemBase {
  
  // Subsystem declaration
  private final Drivetrain m_Drivetrain;
  private final Elevator m_Elevator;
  private final Shooter m_Shooter;
  private final Intake m_Intake;

  // an enum containing the actions we can tell orchestrator to perform
  // can be expanded upon
  public enum OrchestratorAction
  {
    NONE,
    AMP_SHOT,
    SPEAKER_SHOT,
    INTAKE,
    CLIMB,
  }

  // create the local desired action and set it to be none at the beginning
  private OrchestratorAction m_RequestedAction = OrchestratorAction.NONE;
  private OrchestratorAction m_CurrentAction;

  // State classes
  private ElevatorState m_ElevatorState = new ElevatorState();
  private ShooterState m_ShooterState = new ShooterState();
  private IntakeState m_IntakeState = new IntakeState();

  // Suppliers
  private final BooleanSupplier elevatorIsAtClearancePosition;

  /** Creates a new OrchestratorV2. */
  public OrchestratorV2(Drivetrain drivetrain, Elevator elevator, Shooter shooter, Intake intake) 
  {
    this.m_Drivetrain = drivetrain;
    this.m_Elevator = elevator;
    this.m_Shooter = shooter;
    this.m_Intake = intake;

    this.elevatorIsAtClearancePosition = () -> Util.inRange(
      m_Elevator.getElevatorPosition().getValueAsDouble(), 
      ElevatorConstants.ELEVATOR_BOTTOM_CLEARANCE_POSITION, 
      ElevatorConstants.ELEVATOR_BAR_POSITION
    );
  }

  /**
   * Sets the action we want the orchestrator to perform
   * @param action The orchestrator action to perform
   */
  public void setOrchestratorAction(OrchestratorAction action)
  {
    this.m_RequestedAction = action;
  }

  @Override
  public void periodic() {
    // Shooter Odometry
    double m_ShooterMotor1Velocity = m_Shooter.getShooterMotorVelocity(MOTOR.MOTOR_1).getValueAsDouble();
    double m_ShooterMotor2Velocity = m_Shooter.getShooterMotorVelocity(MOTOR.MOTOR_2).getValueAsDouble();
    double m_FeederMotorVelocity = m_Shooter.getShooterMotorVelocity(MOTOR.FEEDER_MOTOR).getValueAsDouble();
    double m_ShooterWristPosition = m_Shooter.getWristMotorPosition().getValueAsDouble();
    boolean m_BeamBreak1Status = m_Shooter.getBeamBreak1();
    boolean m_BeamBreak2Status = m_Shooter.getBeamBreak2();

    // Elevator Odometry
    double m_ElevatorPosition = m_Elevator.getElevatorPosition().getValueAsDouble();
    
    // Intake Odometry
    double m_IntakeWristPosition = m_Intake.getWristPosition().getValueAsDouble();

    // if the requested action is different from the current action
    //if (m_CurrentAction != m_RequestedAction)
    // {
      // switch case based on the action the orchestrator is being asked to perform
      switch (m_RequestedAction) {
        // amp shot is requested
        case AMP_SHOT:

          // assuming elevator is down already

          // set elevator to top position and once that is finished set wrist to go down and spin motors
          new SequentialCommandGroup
          (
            new ParallelCommandGroup(
              // set elevator to top position and set wrist up once it is at the right position
              new SetElevatorPosition(m_Elevator, ElevatorConstants.ELEVATOR_TOP_POSITION),
              new SetWristMotorPosition(m_Shooter, ShooterConstants.WRIST_CLEARANCE_POSITION).onlyIf(elevatorIsAtClearancePosition)
            ),
            // once elevator is at up position and wrist is at clearance position
            new ParallelCommandGroup
            (
              // set wrist to be down and set shooter motors to be up
              new SetWristMotorPosition(m_Shooter, ShooterConstants.WRIST_AMP_SHOOTING_POSITION),
              new SetShooterMotorsVelocity(m_Shooter, 30, 30)
            ),
            // once wrist is down and shooters are up to speed shoot piece
            new SetFeederMotorVelocity(m_Shooter, 20)
          );
          
          
            if(new SetWristMotorPosition(m_Shooter, ShooterConstants.WRIST_CLEARANCE_POSITION).isScheduled())
            {
              // schedule command
              new SetWristMotorPosition(m_Shooter, ShooterConstants.WRIST_CLEARANCE_POSITION);
            }

          break;
        case NONE:

          break;
        default:
          // if switch case breaks somehow or something is unaccounted for, set back to idle
          this.m_RequestedAction = OrchestratorAction.NONE;
          break;
      }
    //}

    this.m_CurrentAction = m_RequestedAction;

    // nightmare nightmare nightmare nightmare nightmare nightmare
  }
}
