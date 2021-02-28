/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.fearxzombie.limelight;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.hoodSubsystem;
import frc.robot.subsystems.indexerSubsystem;
import frc.robot.subsystems.shooterSubsystem;
import frc.robot.subsystems.turretSubsystem;

public class shooterAutoCommand extends CommandBase {

  private indexerSubsystem m_indexer;
  private turretSubsystem m_turret;
  private shooterSubsystem m_shooter;
  private hoodSubsystem m_hood;
  private limelight m_limelight;
  private boolean m_stationary;
  private double steer_k = 0.075;
  private double tv;
  private double tx;
  private double limelightSteerCommand = 0;

  /**
   * shooterAutoCommand class constructor
   * 
   * @param indexer,    indexer subsystem
   * @param turret,     turret subsystem
   * @param shooter,    shooter subsystem
   * @param hood,       hood subsystem
   * @param ll_util,    limelight class
   * @param stationary, boolean: true if robot is stationary
   */
  public shooterAutoCommand(indexerSubsystem indexer, turretSubsystem turret, shooterSubsystem shooter, hoodSubsystem hood, limelight ll_util, boolean stationary) {
    addRequirements(indexer);
    addRequirements(turret);
    addRequirements(shooter);
    addRequirements(hood);
    m_indexer = indexer;
    m_turret = turret;
    m_shooter = shooter;
    m_hood = hood;
    m_limelight = ll_util;
    m_stationary = stationary;
  }

  public shooterAutoCommand(indexerSubsystem indexer, turretSubsystem turret, shooterSubsystem shooter, hoodSubsystem hood, limelight ll_util) {
    // call main constructor, w/ stationary false,
     this(indexer, turret, shooter, hood, ll_util, false);
  }

  @Override
  public void initialize() {
    m_limelight.setLEDMode(0);
  }

  @Override
  public void execute() {
    tv = m_limelight.getTV();
    tx = m_limelight.getTX();

    if (m_stationary && RobotContainer.limelightOnTarget) {
      // we're stationary and we saw the target
      // Trust that the target didn't move
      m_turret.setPercentOutput(0);
    } else {

      if (tv != 1) {
        // no target seen, use manual turret input

        RobotContainer.limelightOnTarget = false;
        limelightSteerCommand = 0;
        var manualInput = RobotContainer.m_operatorController.getX(Hand.kLeft);
        if (Math.abs(manualInput) > 0.05) {
          m_turret.setPercentOutput(manualInput * 0.5);
        } else {
          // if we don't see a target stop the turret
          m_turret.setPercentOutput(0);
        }
        return;
      }

      // m_shooter.setShooterRPM(m_shooter.getRPMforTY(m_limelight.getTY()));
      m_shooter.setShooterRPM(m_shooter.getRPMforDistanceMeter(Robot.distance_meters));
      limelightSteerCommand = tx * steer_k;
      m_turret.setPercentOutput(limelightSteerCommand);

      if (Math.abs(m_limelight.getTX()) < 0.75) {
        RobotContainer.limelightOnTarget = true;
      } else {
        RobotContainer.limelightOnTarget = false;
      }
    }

    // shoot!
    // TODO: make sure hood is in correct position as well
    if (m_shooter.isAtSpeed() == true && RobotContainer.limelightOnTarget == true) {
      m_indexer.ejectOneBall();
    }
  }
  
  @Override
  public void end(boolean interrupted) {
    m_indexer.stopIndexer();
    m_shooter.setShooterRPM(0);
    m_turret.stop();
    m_hood.retractHood();
    RobotContainer.limelightOnTarget = false;
    //m_limelight.setLEDMode(1);
    m_stationary = false;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
