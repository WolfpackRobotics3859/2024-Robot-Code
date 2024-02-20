// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.orchestrator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Orchestrator;
import frc.robot.subsystems.Orchestrator.RobotState;

public class SetPersistentState extends InstantCommand {
  private final Orchestrator m_Orchestrator;
  private final RobotState m_DesiredState;

  /**
   * Tells the orchestrator to move the robot to a certain state. This request for an state will persist even after the controller's button is released.
   * For example, if you want to tell the robot to begin intaking, using this command you would only press the button once, but the
   * robot will continue intaking.
   * @param orchestrator The orchestrator subsystem.
   * @param action The state you want the robot to be in.
   */
  public SetPersistentState(Orchestrator orchestrator, RobotState desiredState)
  {
    this.m_Orchestrator = orchestrator;
    this.m_DesiredState = desiredState;

    // add requirements to make it so that multiple commands cannot be requested at once
    addRequirements(m_Orchestrator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_Orchestrator.setOrchestratorState(m_DesiredState);;
  }
}
