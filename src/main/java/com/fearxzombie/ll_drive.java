package com.fearxzombie;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * An object that can drive a robot to a target based off of desired target area value.
 */
public class ll_drive {
    public boolean m_calculationsComplete = false;
    public boolean m_LimelightHasValidTarget = false;
    private double m_LimelightDriveCommand = 0.0;
    private double m_LimelightSteerCommand = 0.0;
    limelight m_limelight;

    /**
     * calculate - This calcuates the data for the program. 
     * @param STEER Sensitivity of horizontal target lock.
     * @param DRIVE Sensitivity of travelling to a given target.
     * @param desiredTA How close you want the robot to get to a target.
     * @param MAX Maximum speed limit of robot.
     * @param name name of the limelight used to track the target
     */
    public void calculate(double STEER, double DRIVE, double desiredTA, double MAX, limelight m_limelight){
    double tv = m_limelight.getTV();
    double tx = m_limelight.getTX();
    double ta = m_limelight.getTA();

    if (tv < 1.0)
    {
      m_LimelightHasValidTarget = false;
      m_calculationsComplete = false;
      m_LimelightDriveCommand = 0.0;
      m_LimelightSteerCommand = 0.0;
      System.out.println("ll_drive.calculate(): No target");
      return;
    }

    m_LimelightHasValidTarget = true;
    
    // Start with proportional steering
    double steer_cmd = tx * STEER;
    m_LimelightSteerCommand = steer_cmd;

    // try to drive forward until the target area reaches our desired area
    double drive_cmd = (desiredTA - ta) * DRIVE;

    // don't let the robot drive too fast into the goal
    if (drive_cmd > MAX)
    {
      drive_cmd = MAX;
    }
    m_LimelightDriveCommand = drive_cmd;
    m_calculationsComplete = true;
    System.out.println("ll_drive.calculate(): Target Calculation complete, ready to start");
  }
  /**
   * start - runs calculated path, must run concurrent with calculate to allow TA detection.
   * The program will only start if the calculations are present.
   * @param drive Differental drive object for the robot's drivetrain.
   * @param isSteerReversed
   * @param isDriveReversed
   */
  public void start(DifferentialDrive drive, boolean isSteerReversed, boolean isDriveReversed){
    if (m_calculationsComplete = true){
    System.out.println("ll_drive.start(): Running program.");
    if (isSteerReversed == true && isDriveReversed == true){
    drive.arcadeDrive(-m_LimelightDriveCommand, -m_LimelightSteerCommand);
    } else if (isSteerReversed == false && isDriveReversed == true) {
      drive.arcadeDrive(-m_LimelightDriveCommand, m_LimelightSteerCommand);
    } else if (isSteerReversed == true && isDriveReversed == false){
      drive.arcadeDrive(m_LimelightDriveCommand, -m_LimelightSteerCommand);
    } else {
      drive.arcadeDrive(m_LimelightDriveCommand, m_LimelightSteerCommand);
    }
  } else {
    System.out.println("ll_drive.start(): No program to run, awaiting calculations.");
  }
}
}