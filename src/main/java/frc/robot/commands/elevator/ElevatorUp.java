// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.shooter.SetWristMotorPosition;
import frc.robot.constants.elevator.ElevatorConstants;
import frc.robot.constants.shooter.ShooterConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;

public class ElevatorUp extends SequentialCommandGroup
{
  /**
   * A general command for sending the elevator + shooter up. This is put into a single command here for ease
   * of reuse across the robot, as it is a very common routine for the robot to be performing.
   * @param elevator The elevator subsystem
   * @param shooter The shooter subsystem
   * @param endBehaviour The desired end behaviour of the robot when this command is over.
   * This is important for setting what the end position of the shooter wrist should be.
   */
  public ElevatorUp(Elevator elevator, Shooter shooter, EndBehaviour endBehaviour)
  {
    addRequirements(elevator, shooter);

    addCommands
    (
      new ConditionalCommand
      (
        // if elevator is below bar
        new SequentialCommandGroup
        (
          new PrintCommand("Elevator is below the bar, moving up"),
          new ParallelCommandGroup
          (
            new SetElevatorPosition(elevator, ElevatorConstants.ELEVATOR_TOP_POSITION),
            // send wrist to clearance position once elevator is high enough
            new WaitUntilCommand(elevator.elevatorIsAboveClearancePosition).andThen(
              new PrintCommand("Moving wrist to clearance position"),
              new SetWristMotorPosition(shooter, ShooterConstants.WRIST_CLEARANCE_POSITION)
            ),
            // if elevator is almost at bar stop elevator and wait for shooter wrist to continue unless shooter wrist is already at the proper location
            new WaitUntilCommand(() -> elevator.getElevatorPosition().getValueAsDouble() > ElevatorConstants.ELEVATOR_UP_CHECK_POSITION).andThen(
              // stop elevator, set wrist to clearance position and wait until it's at the position to continue moving elevator
              new SequentialCommandGroup
              (
                new PrintCommand("Elevator too high for current clearance position!"),
                new SetElevatorPosition(elevator, ElevatorConstants.ELEVATOR_UP_CHECK_POSITION),
                new SetWristMotorPosition(shooter, ShooterConstants.WRIST_CLEARANCE_POSITION),
                new SetElevatorPosition(elevator, ElevatorConstants.ELEVATOR_TOP_POSITION)
              )
            ).unless(shooter.wristIsAtClearancePosition)
          ),
          // once elevator is up, set wrist back to flat
          new PrintCommand("Setting wrist back to flat"),
          new SetWristMotorPosition(shooter, ShooterConstants.WRIST_DEFAULT_POSITION)
        ),
        // if elevator is above bar 
        new SequentialCommandGroup
        (
          // set wrist flat, then set elevator up
          new PrintCommand("Elevator above bar, setting up and flat"),
          new SetWristMotorPosition(shooter, ShooterConstants.WRIST_DEFAULT_POSITION),
          new SetElevatorPosition(elevator, ElevatorConstants.ELEVATOR_TOP_POSITION)
        ),
        elevator.elevatorIsBelowBar
      )
    );
  }

  public enum EndBehaviour
  {
    SHOOTING,
    AMP,
    IDLE,
    INTAKING
  }
}
