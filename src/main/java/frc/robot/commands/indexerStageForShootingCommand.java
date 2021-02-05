/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexerSubsystem;

public class indexerStageForShootingCommand extends CommandBase {

  indexerSubsystem m_indexer;

  public indexerStageForShootingCommand(indexerSubsystem indexer) {
    addRequirements(indexer);
    m_indexer = indexer;
  }

  @Override
  public void initialize() {
    m_indexer.runIndexer();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    m_indexer.stopIndexer();
  }

  @Override
  public boolean isFinished() {
    if (m_indexer.ballExiting() == true) {
      return true;
    }
    
    return false;
  }
}
