/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intakeSubsystem;

public class intakeDeployCommand extends CommandBase {

  intakeSubsystem m_intake;

  public intakeDeployCommand(intakeSubsystem intake) {
    addRequirements(intake);
    m_intake = intake;
  }

  @Override
  public void initialize() {
    m_intake.deployIntake();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.retractIntake();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
