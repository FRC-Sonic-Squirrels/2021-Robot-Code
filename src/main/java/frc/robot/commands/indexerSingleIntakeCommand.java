/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.indexerSubsystem;

public class indexerSingleIntakeCommand extends CommandBase {

  private indexerSubsystem m_indexer;
  private boolean clearedSensor2 = false;
  private XboxController opController = RobotContainer.m_operatorController;

  public indexerSingleIntakeCommand(indexerSubsystem indexer) {
    addRequirements(indexer);
    m_indexer = indexer;
  }

  @Override
  public void initialize() {
    clearedSensor2 = false;
    m_indexer.runOnlyIntake();
  }

  @Override
  public void execute() {

    // TODO: tune the speeds of these to not destory balls
    if (opController.getTriggerAxis(Hand.kRight) >= 0.1) {
      m_indexer.setIntakePercentOutput(opController.getTriggerAxis(Hand.kRight));
      m_indexer.setBeltsPercentOutput(opController.getTriggerAxis(Hand.kRight));
      m_indexer.setKickerPercentOutput(opController.getTriggerAxis(Hand.kRight));
    }

    // TODO: tune the speeds of these to not destory balls
    else if (opController.getTriggerAxis(Hand.kLeft) >= 0.1) {
      m_indexer.setIntakePercentOutput(-opController.getTriggerAxis(Hand.kLeft));
      m_indexer.setBeltsPercentOutput(-opController.getTriggerAxis(Hand.kLeft));
      m_indexer.setKickerPercentOutput(-opController.getTriggerAxis(Hand.kLeft));
    }

    else {

      if (m_indexer.ballStaged() == false) {
        clearedSensor2 = true;
      }

      if (m_indexer.ballExiting() == true) {
        // Always stop indexer if a ball is at the exit point.
        // Don't eject a ball unless we're shooting.
        m_indexer.stopIndexer();
      }
      else if (m_indexer.ballReadyForIndexer() == true) {
        // here we know ballExiting() == false
        // OK to pull in more balls and continue to fill indexer
        m_indexer.runIndexer();
      }
      if (clearedSensor2 && m_indexer.ballStaged()) {
        m_indexer.runOnlyIntake();
        clearedSensor2 = false;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
