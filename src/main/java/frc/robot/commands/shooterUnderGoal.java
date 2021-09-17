/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.hoodSubsystem;
import frc.robot.subsystems.indexerSubsystem;
import frc.robot.subsystems.shooterSubsystem;
import frc.robot.subsystems.turretSubsystem;

public class shooterUnderGoal extends CommandBase {

  private indexerSubsystem m_indexer;
  private turretSubsystem m_turret;
  private shooterSubsystem m_shooter;
  private hoodSubsystem m_hood;

  /**
   * shooterUnderGoal class constructor
   * 
   * @param indexer,    indexer subsystem
   * @param turret,     turret subsystem
   * @param shooter,    shooter subsystem
   * @param hood,       hood subsystem
   */
  public shooterUnderGoal(indexerSubsystem indexer, turretSubsystem turret, shooterSubsystem shooter, hoodSubsystem hood) {
    addRequirements(indexer);
    addRequirements(turret);
    addRequirements(shooter);
    m_indexer = indexer;
    m_turret = turret;
    m_shooter = shooter;
    m_hood = hood;
  }


  @Override
  public void initialize() {
    m_shooter.setShooterRPMforDistanceFeet(4.0);  // for 4 feet away
    m_turret.setAngleDegrees(0);
    m_hood.retractHood();
  }

  @Override
  public void execute() {

    // shoot!
    if (m_shooter.isAtSpeed() == true) {
      m_indexer.ejectOneBall();
    }
  }
  
  @Override
  public void end(boolean interrupted) {
    m_indexer.stopIndexer();
    m_shooter.setShooterRPM(0);
    m_turret.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
