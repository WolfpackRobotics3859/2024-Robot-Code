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

// the grasses are quiet because the trees are listening
// every day i stray further from sanity
public class ElevatorDown extends SequentialCommandGroup
{
  /**
   * A general command for sending the elevator + shooter down. This is put into a single command here for ease
   * of reuse across the robot, as it is a very common routine for the robot to be performing.
   * @param elevator The elevator subsystem
   * @param shooter The shooter subsystem
   * @param endBehaviour The desired end behaviour of the robot when this command is over.
   * This is important for setting what the end position of the shooter wrist should be.
   */
  public ElevatorDown(Elevator elevator, Shooter shooter, ElevatorDownEndBehaviour endBehaviour)
  {
    // add subsystem requirements to prevent interrupts
    addRequirements(elevator, shooter);

    // spaghetti code time :3
    addCommands
    (
      new ConditionalCommand
      (
        // if elevator is above bar
        new SequentialCommandGroup
        (
          new PrintCommand("Elevator is above bar"),
          // send elevator up if it's not high enough
          new SetElevatorPosition(elevator, ElevatorConstants.ELEVATOR_TOP_POSITION).unless(elevator.elevatorIsAtTop),
          // next, send wrist to clearance position
          new SetWristMotorPosition(shooter, ShooterConstants.WRIST_CLEARANCE_POSITION),
          // then, send elevator down while waiting for elevator to be below bar
          new ParallelCommandGroup
          (
            new PrintCommand("Sending elevator down"),
            new SetElevatorPosition(elevator, ElevatorConstants.ELEVATOR_BOTTOM_POSITION),
            // wait until elevator is below the bar and then set the shooter wrist to flat
            new WaitUntilCommand(elevator.elevatorIsBelowBar).andThen(new SetWristMotorPosition(shooter, ShooterConstants.WRIST_DEFAULT_POSITION)),
            new WaitUntilCommand(() -> elevator.getElevatorPosition().getValueAsDouble() < ElevatorConstants.ELEVATOR_DOWN_CHECK_POSITION).andThen
            (
              new PrintCommand("Elevator too low for wrist position, stopping elevator movement"),
              new SequentialCommandGroup
              (
                new WaitUntilCommand(() -> elevator.getElevatorPosition().getValueAsDouble() < ElevatorConstants.ELEVATOR_DOWN_CHECK_POSITION),
                // set elevator to down check position
                new SetElevatorPosition(elevator, ElevatorConstants.ELEVATOR_DOWN_CHECK_POSITION),
                // then wait for shooter wrist to be at proper position before setting elevator down
                new WaitUntilCommand(shooter.wristIsAtDefaultPosition),
                new SetElevatorPosition(elevator, ElevatorConstants.ELEVATOR_BOTTOM_POSITION)
              )
            ).unless(shooter.wristIsAtClearancePosition)
          )
        ),
        // if elevator is below bar 
        new SequentialCommandGroup
        (
          new PrintCommand("Elevator is below bar, setting flat and sending down"),
          // set wrist to flat and set elevator down
          new SetWristMotorPosition(shooter, ShooterConstants.WRIST_DEFAULT_POSITION),
          new SetElevatorPosition(elevator, ElevatorConstants.ELEVATOR_BOTTOM_POSITION)
        ),
        elevator.elevatorIsAboveBar
      )
    );
  }

  public enum ElevatorDownEndBehaviour
  {
    SHOOTING,
    AMP,
    IDLE,
    INTAKING
  }
}
