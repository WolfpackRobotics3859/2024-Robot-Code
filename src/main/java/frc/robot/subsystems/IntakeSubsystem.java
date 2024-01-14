// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.intake.IntakeConstants;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;


public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private TalonFX intakeMotor = new TalonFX(17);


  public IntakeSubsystem()
  {
    intakeMotor.getConfigurator().apply(IntakeConstants.INTAKE_GAINS);
  }

  @Override
  public void periodic()
  {
    // Intentionally Empty
  }

  public void setIntakePosition(double position)
  {
    MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(position, 40, false, 0, 0, false, false, false);
    intakeMotor.setControl(request);
  }
}
