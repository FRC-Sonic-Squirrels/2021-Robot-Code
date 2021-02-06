/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexerSubsystem;
import frc.robot.subsystems.shooterSubsystem;
import frc.robot.subsystems.turretSubsystem;

public class hoodDownNoLimelight extends CommandBase {

  private indexerSubsystem m_indexer;
  private turretSubsystem m_turret;
  private shooterSubsystem m_shooter;

  /**
   * Creates a new hoodDownNoLimelight.
   */
  public hoodDownNoLimelight(indexerSubsystem indexer, turretSubsystem turret, shooterSubsystem shooter) {
    addRequirements(indexer);
    addRequirements(turret);
    addRequirements(shooter);
    m_indexer = indexer;
    m_turret = turret;
    m_shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turret.setAngleDegrees(0);
    m_shooter.setShooterRPM(2800);  // RPM for shooting from 4 feet
    m_shooter.retractHood();
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_shooter.isAtSpeed() == true && (Math.abs(m_turret.getAngleDegrees()) < 0.5 )) {
      m_indexer.ejectOneBall();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexer.stopIndexer();
    m_shooter.idle();
    m_turret.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
