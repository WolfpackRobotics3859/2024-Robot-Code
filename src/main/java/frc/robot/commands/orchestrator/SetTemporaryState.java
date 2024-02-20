// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.orchestrator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Orchestrator;
import frc.robot.subsystems.Orchestrator.RobotState;;

public class SetTemporaryState extends Command {
  private final Orchestrator m_Orchestrator;
  private final RobotState m_desiredState;

  /** Tells the orchestrator to move the robot to a given state. The robot will return to it's idle state once the controller's button
   * is released. For example, if you want the orchestrator to shoot, pressing the button will get the robot ready to 
   * shoot, but once the button is release, the robot will stop preparing to shoot and go back to it's idle state.
   * @param orchestrator The orchestrator subsystem
   * @param action The state you want the robot to be in while the button is being pressed
   */
  public SetTemporaryState(Orchestrator orchestrator, RobotState desiredState)
  {
    this.m_Orchestrator = orchestrator;
    this.m_desiredState = desiredState;

    // add requirements to make it so that multiple commands cannot be requested at once
    addRequirements(m_Orchestrator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    // Intentionally Empty
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    // Set orchestrator's action
    m_Orchestrator.setOrchestratorState(m_desiredState);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    // Remove the set action when button is released
    m_Orchestrator.setOrchestratorState(RobotState.IDLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }
}
