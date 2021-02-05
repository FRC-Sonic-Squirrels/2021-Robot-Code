/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexerSubsystem;

public class indexerSingleFeedCommand extends CommandBase {

  private indexerSubsystem m_indexer;
  private boolean clearedSensor3 = false;
  private int startBallCount;

  public indexerSingleFeedCommand(indexerSubsystem indexer) {
    addRequirements(indexer);
    m_indexer = indexer;
  }

  @Override
  public void initialize() {
    clearedSensor3 = false;
    startBallCount = m_indexer.getBallCount();
    m_indexer.setFinishedSingleFeed(false);
  }

  @Override
  public void execute() {
    
    if (m_indexer.ballExiting() == false && startBallCount == m_indexer.getBallCount() + 1) {
      clearedSensor3 = true;
    }

    if (m_indexer.ballExiting() == false && startBallCount == m_indexer.getBallCount()) {
      m_indexer.ejectIndexer();
    }

    if (m_indexer.ballExiting() == true) {
      m_indexer.ejectIndexer();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_indexer.stopIndexer();
    m_indexer.setFinishedSingleFeed(true);
  }

  @Override
  public boolean isFinished() {
    if (clearedSensor3 == true && startBallCount == m_indexer.getBallCount() + 1) {
      return true;
    }
    
    return false;
  }
}
