/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.fearxzombie.limelight;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private double steer_kp = 0.03;
  private double steer_ki = 0.0001;
  private double limelightSteerCommand = 0;
  private long startTimeNS = 0;
  private long shooterReadyTimeNS = 0;
  private double m_Integral = 0;

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
    // m_stationary = stationary;
    SmartDashboard.putNumber("Time to Shoot", 0.0);
  }

  public shooterAutoCommand(indexerSubsystem indexer, turretSubsystem turret, shooterSubsystem shooter, hoodSubsystem hood, limelight ll_util) {
    // call main constructor, w/ stationary false,
     this(indexer, turret, shooter, hood, ll_util, false);
  }

  @Override
  public void initialize() {
    m_limelight.setLEDMode(0);
    m_Integral = 0;
  }

  @Override
  public void execute() {
    if (startTimeNS == 0) {
      startTimeNS = System.nanoTime();
    }

    m_shooter.setShooterRPM(m_shooter.getRPMforDistanceMeter(Robot.distance_meters));
    m_hood.setPositionRotations(m_hood.angleToRotations(m_hood.getAngleforDistanceMeter(Robot.distance_meters)));
    double tx_angleError = m_limelight.getTX();
    m_Integral += -tx_angleError * (0.02);
    limelightSteerCommand = (tx_angleError * steer_kp) + (m_Integral * steer_ki);
    m_turret.setPercentOutput(limelightSteerCommand);

    // shoot!
    if ( m_shooter.isAtSpeed() && RobotContainer.limelightOnTarget && m_hood.isAtPos() ) {
      m_indexer.ejectOneBall();
      if (shooterReadyTimeNS == 0)
        shooterReadyTimeNS = System.nanoTime();
        SmartDashboard.putNumber("Time to Shoot", (double) (shooterReadyTimeNS - startTimeNS) / 1_000_000_000);
    }
  }
  
  @Override
  public void end(boolean interrupted) {
    m_indexer.stopIndexer();
    m_shooter.setShooterRPM(0);
    m_turret.stop();
    m_hood.retractHood();
    //m_limelight.setLEDMode(1);
    //m_stationary = false;
    startTimeNS = 0;
    shooterReadyTimeNS = 0;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
