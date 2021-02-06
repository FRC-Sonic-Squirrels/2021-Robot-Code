/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexerSubsystem;

public class indexerRestageCommand extends CommandBase {
  
  indexerSubsystem m_indexer;
  private int restageEndBallCount;

  public indexerRestageCommand(indexerSubsystem indexer) {
    addRequirements(indexer);
    m_indexer = indexer; 
  }

  @Override
  public void initialize() {
    m_indexer.setRestageState(0);
    restageEndBallCount = m_indexer.getBallCount();
  }

  @Override
  public void execute() {
    
    if (m_indexer.ballReadyForIndexer() == false && m_indexer.getRestageState() == 0) {
      m_indexer.reverseIndexer();
    }

    if (m_indexer.ballReadyForIndexer() == true && m_indexer.ballStaged() == true) {
      m_indexer.runIndexer();
    }

    if (m_indexer.ballReadyForIndexer() == true && m_indexer.getRestageState() == 0) {
      m_indexer.runIndexer();
      m_indexer.setRestageState(1);
    }

    if (m_indexer.ballStaged() == false && m_indexer.getRestageState() == 1) {
      m_indexer.runIndexer();
    }

    if (m_indexer.ballStaged() == true && m_indexer.getRestageState() == 1) {
      m_indexer.stopIndexer();
      m_indexer.setRestageState(2);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_indexer.setBallCount(restageEndBallCount);
    m_indexer.stopIndexer();
  }

  @Override
  public boolean isFinished() {
    if (m_indexer.getRestageState() == 2) {
      return true;
    }
    return false;
  }
}
