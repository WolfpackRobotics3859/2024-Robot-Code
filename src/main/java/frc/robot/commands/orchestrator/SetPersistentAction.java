// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.orchestrator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.OrchestratorV2;
import frc.robot.subsystems.OrchestratorV2.OrchestratorAction;

public class SetPersistentAction extends InstantCommand {
  private final OrchestratorV2 m_Orchestrator;
  private final OrchestratorAction m_DesiredAction;

  /**
   * Tells the orchestrator to perform an action. This request for an action will persist even after the controller's button is released.
   * For example, if you want to tell the orchestrator to begin intaking, using this command you would only press the button once, but the
   * robot will continue intaking.
   * @param orchestrator The orchestrator subsystem.
   * @param action The action you want the orchestrator to perform.
   */
  public SetPersistentAction(OrchestratorV2 orchestrator, OrchestratorAction desiredAction)
  {
    this.m_Orchestrator = orchestrator;
    this.m_DesiredAction = desiredAction;
    
    // add requirements to make it so that multiple commands cannot be requested at once
    addRequirements(m_Orchestrator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_Orchestrator.setOrchestratorAction(this.m_DesiredAction);
  }
}
