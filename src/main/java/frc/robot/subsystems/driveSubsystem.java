/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.driveConstants.kaVoltSecondsSquaredPerMeter;
import static frc.robot.Constants.driveConstants.ksVolts;
import static frc.robot.Constants.driveConstants.kvVoltSecondsPerMeter;
import static frc.robot.Constants.driveConstants.kPDriveVel;
import static frc.robot.Constants.driveConstants.kDDriveVel;
import static frc.robot.Constants.driveConstants.kEncoderCPR;
import static frc.robot.Constants.driveConstants.kDistancePerWheelRevolutionMeters;
import static frc.robot.Constants.driveConstants.kGearReduction;
import static frc.robot.Constants.driveConstants.kGyroReversed;
import static frc.robot.Constants.driveConstants.kDriveKinematics;
import static frc.robot.Constants.driveConstants.driveTimeout;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.driveConstants;
import static frc.robot.Constants.canId;

public class driveSubsystem extends SubsystemBase {

  private WPI_TalonFX falcon1_leftLead    = new WPI_TalonFX(driveConstants.falcon1_leftLead);
  private WPI_TalonFX falcon2_leftFollow  = new WPI_TalonFX(driveConstants.falcon2_leftFollow);
  private WPI_TalonFX falcon3_rightLead   = new WPI_TalonFX(driveConstants.falcon3_rightLead);
  private WPI_TalonFX falcon4_rightFollow = new WPI_TalonFX(driveConstants.falcon4_rightFollow);

  // teleop driver control fine tuning
  private boolean driveInvert = true;
  private boolean forzaModeEnabled = true;
  private boolean squaredInputs = false;


  // New Gyro, pigeon IMU on the CAN bus
  private PigeonIMU m_gyro = new PigeonIMU(canId.canId20_pigeon_imu);

  private final DifferentialDrive m_drive;
  private final SimpleMotorFeedforward  m_feedforward = 
      new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter);

  private PIDController left_PIDController = new PIDController(kPDriveVel, 0.0, kDDriveVel);
  private PIDController right_PIDController =  new PIDController(kPDriveVel, 0.0, kDDriveVel);

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;
 
  // robot drives opposite of built in motor encoders
  private double invertEncoders = -1.0;

  private boolean m_debug = false;

  // http://www.ctr-electronics.com/downloads/pdf/Falcon%20500%20User%20Guide.pdf
  // Peak power: 140A
  // Stall:      257A  (more than the battery can supply)
  // Battery can at best supply around 250A
  private SupplyCurrentLimitConfiguration m_limit =
      new SupplyCurrentLimitConfiguration(true, 30, 20, 0.5);
  
  public driveSubsystem() {

    m_gyro.configFactoryDefault();

    falcon1_leftLead.configFactoryDefault();
    falcon2_leftFollow.configFactoryDefault();
    falcon3_rightLead.configFactoryDefault();
    falcon4_rightFollow.configFactoryDefault();

    // Current limiting
    setCurrentLimit(m_limit);

    // Voltage limits - WARNING: this does not play nice with autonomous driving
    //setVoltageLimit(11);

    // set Ramp up speed, time in seconds (smaller is more responsive, 0 disables)
    // configOpenLoopRampRate(0.25);
    
    // set brake mode
    falcon1_leftLead.setNeutralMode(NeutralMode.Brake);
    falcon2_leftFollow.setNeutralMode(NeutralMode.Brake);
    falcon3_rightLead.setNeutralMode(NeutralMode.Brake);
    falcon4_rightFollow.setNeutralMode(NeutralMode.Brake);
    
    // No need to invert Follow Motors
    falcon1_leftLead.setInverted(false);
    falcon3_rightLead.setInverted(true);
    falcon2_leftFollow.setInverted(InvertType.FollowMaster);
    falcon4_rightFollow.setInverted(InvertType.FollowMaster);

    // set Lead/Follow 
    falcon2_leftFollow.follow(falcon1_leftLead);
    falcon4_rightFollow.follow(falcon3_rightLead);

    // NOTE: setSensorPhase() does nothing on TalonFX motors as the encoders 
    // are integrated, and can cannot be out of phase with the motor. 
    falcon1_leftLead.setSensorPhase(true);
    falcon3_rightLead.setSensorPhase(true);

    // default feedback sensor
    falcon1_leftLead.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, driveTimeout);
    falcon3_rightLead.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, driveTimeout);

    m_drive = new DifferentialDrive(falcon1_leftLead, falcon3_rightLead);
    m_drive.setRightSideInverted(false);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

  }
  
  @Override
  public void periodic() {
    // Note: periodic() is run by the scheduler, always. No matter what.
    // Update the odometry in the periodic block
    double leftDist = getLeftPosition(); 
    double rightDist = getRightPosition();
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), leftDist, rightDist);

    if (m_debug) {
      // log drive train and data to SmartDashboard
      /* Display 9-axis Heading (requires magnetometer calibration to be useful) */
      SmartDashboard.putNumber("IMU_FusedHeading", m_gyro.getFusedHeading());
      // NOTE: call getFusedHeading(FusionStatus) to detect gyro errors

      // report the wheel speed, position, and pose
      SmartDashboard.putNumber("left_wheel_Velocity", getLeftVelocity());
      SmartDashboard.putNumber("right_wheel_Velocity", getRightVelocity());
      SmartDashboard.putNumber("left_wheel_Distance", leftDist);
      SmartDashboard.putNumber("right_wheel_Distance", rightDist);
      SmartDashboard.putNumber("left volts", falcon1_leftLead.getMotorOutputVoltage());
      SmartDashboard.putNumber("right volts", falcon3_rightLead.getMotorOutputVoltage());

      Pose2d currentPose = m_odometry.getPoseMeters();
      SmartDashboard.putNumber("pose_x", currentPose.getTranslation().getX());
      SmartDashboard.putNumber("pose_y", currentPose.getTranslation().getY());
      SmartDashboard.putNumber("pose_theta", currentPose.getRotation().getDegrees());

      // from
      // https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20General/DriveStraight_Pigeon/src/main/java/frc/robot/Robot.java
      boolean angleIsGood = (m_gyro.getState() == PigeonIMU.PigeonState.Ready) ? true : false;
      SmartDashboard.putBoolean("DEBUG pigeon angle is good", angleIsGood);
    }
  }
  
  /**
   * Returns the distance in Meters the left wheel has travelled
   *
   * @return distance in meters
   */
  double getLeftPosition() {
     // Native units are encoder ticks (2048 ticks per revolution)
    return invertEncoders * falcon1_leftLead.getSelectedSensorPosition() * kDistancePerWheelRevolutionMeters * kGearReduction / kEncoderCPR;
  }

  /**
   * Returns the distance in Meters the right wheel has travelled
   *
   * @return distance in meters
   */
  double getRightPosition() {
    // Native units are encoder ticks (2048 ticks per revolution)
    return invertEncoders * falcon3_rightLead.getSelectedSensorPosition() * kDistancePerWheelRevolutionMeters * kGearReduction / kEncoderCPR;
  }

  /**
   * Returns the velocity of the left wheel in meters per second
   *
   * @return velocity in meters/second
   */
  double getLeftVelocity() {
    // Native units are encoder ticks per 100ms
    return invertEncoders * falcon1_leftLead.getSelectedSensorVelocity() * kDistancePerWheelRevolutionMeters * kGearReduction * 10.0 / kEncoderCPR ;
  }

  /**
   * Returns the velocity of the right wheel in meters per second
   *
   * @return velocity in meters/second
   */
  double getRightVelocity() {
    // Native units are encoder ticks per 100ms
    return invertEncoders * falcon3_rightLead.getSelectedSensorVelocity() * kDistancePerWheelRevolutionMeters * kGearReduction * 10.0 / kEncoderCPR ;
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * getFuturePose() - predict robot pose t_sec in the future based on the current robot wheel
   * speed.
   * 
   * This is only an estimate and the larger the value t_sec, the less accurate the estimate will
   * be.
   * 
   * @param t_sec
   * @return Estimated pose t_sec in the future.
   */
  public Pose2d getFuturePose(double t_sec) {
    Pose2d current_pose = m_odometry.getPoseMeters();
    if (t_sec <= 0) {
      System.out.println("Error: getFuturePose needs a positive time value.");
      return current_pose;
    }

    // new odometry class starting from current position
    Rotation2d current_heading = Rotation2d.fromDegrees(getHeading());
    DifferentialDriveOdometry future_odometry =
        new DifferentialDriveOdometry(current_heading, current_pose);

    // predict where we will be t_sec in the future based on our current wheel speeds
    future_odometry.update(current_heading, getLeftVelocity() * t_sec,
        getRightVelocity() * t_sec);

    return future_odometry.getPoseMeters();
  }

  /**
   * Returns the Feedforward settings for the drivetrain.
   * 
   * @return Feedforward
   */
  public SimpleMotorFeedforward getFeedforward() {
    return m_feedforward;
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        getLeftVelocity(),
        getRightVelocity());
  }

  /**
   * getChassisSpeeds() - return the robot velocity in meters/second in robot centric X and Y
   * direction, and the rotation of the robot in radians/second.
   * 
   * Note: Vy should always be zero, because the robot cannot drive sideways.
   * 
   * @return ChassisSpeeds: (vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond)
   */
  public ChassisSpeeds getChassisSpeeds() {
    return kDriveKinematics.toChassisSpeeds(getWheelSpeeds());
  }

  /**
   * Returns the left PIDController object
   *
   * @return PIDController
   */
  public PIDController getLeftPidController() {
    return left_PIDController;
  }

  public void setCoastMode(){
    falcon1_leftLead.setNeutralMode(NeutralMode.Coast);
    falcon2_leftFollow.setNeutralMode(NeutralMode.Coast);
    falcon3_rightLead.setNeutralMode(NeutralMode.Coast);
    falcon4_rightFollow.setNeutralMode(NeutralMode.Coast);
  }

  /**
   * Returns the right PIDController object
   *
   * @return PIDController
   */
  public PIDController getRightPidController() {
    return right_PIDController;
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    // use slew rate filters to implement ramp up/down of speed and rotation
    //m_drive.arcadeDrive(speedFilter.calculate(fwd), rotationFilter.calculate(rot));
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    falcon1_leftLead.setVoltage(invertEncoders * leftVolts);
    falcon3_rightLead.setVoltage(invertEncoders * rightVolts);
    m_drive.feed();
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    falcon1_leftLead.setSelectedSensorPosition(0);
    falcon3_rightLead.setSelectedSensorPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return ((getLeftPosition() + getRightPosition()) / 2.0);
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

    /**
   * Set the heading of the robot. In degrees.
   */
  public void setHeadingRadians(double headingRadians) {
    setHeadingDegrees(Math.toDegrees(headingRadians));
  }

  /**
   * Set the heading of the robot. In degrees.
   */
  public void setHeadingDegrees(double headingDegrees) {
    m_gyro.setAccumZAngle(headingDegrees);
    m_gyro.setFusedHeading(headingDegrees);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    setHeadingDegrees(0.0);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    // m_gyro.getFusedHeading() returns degrees
    return Math.IEEEremainder(m_gyro.getFusedHeading(), 360) * (driveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    double [] xyz_dps = new double [3];
    // getRawGyro returns in degrees/second
    m_gyro.getRawGyro(xyz_dps);
    return xyz_dps[2] * (kGyroReversed ? -1.0 : 1.0);
  }


  /**
   * Enable current limiting.
   *
   * @param current limit
   */
  public void setVoltageLimit(double maxV) {
    if (maxV > 12.0) {
      maxV = 12.0;
    }
    if (maxV < 2.0) {
      maxV = 2.0;
    }
    falcon1_leftLead.configVoltageCompSaturation(maxV);
    falcon1_leftLead.enableVoltageCompensation(true);
    falcon2_leftFollow.configVoltageCompSaturation(maxV);
    falcon2_leftFollow.enableVoltageCompensation(true);
    falcon3_rightLead.configVoltageCompSaturation(maxV);
    falcon3_rightLead.enableVoltageCompensation(true);
    falcon4_rightFollow.configVoltageCompSaturation(maxV);
    falcon4_rightFollow.enableVoltageCompensation(true);
  }

  /**
   * configOpenLoopRampRate() - Set minimum desired time to go from neutral to full throttle. 
   *      A value of '0' will disable the ramp.
   *  
   * @param secondsFromNeutralToFull
   */
  public void configOpenLoopRampRate(double secondsFromNeutralToFull) {
    falcon1_leftLead.configOpenloopRamp(secondsFromNeutralToFull, driveTimeout);
    falcon2_leftFollow.configOpenloopRamp(secondsFromNeutralToFull, driveTimeout);
    falcon3_rightLead.configOpenloopRamp(secondsFromNeutralToFull, driveTimeout);
    falcon4_rightFollow.configOpenloopRamp(secondsFromNeutralToFull, driveTimeout);
  }

  /**
   * Enable current limiting.
   *
   * @param current limit
   */
  public void setCurrentLimit(SupplyCurrentLimitConfiguration limit) {
    falcon1_leftLead.configSupplyCurrentLimit(limit);
    falcon2_leftFollow.configSupplyCurrentLimit(limit);
    falcon2_leftFollow.configSupplyCurrentLimit(limit);
    falcon4_rightFollow.configSupplyCurrentLimit(limit);
  }

  /**
   * Enable default current limiting for drivetrain.
   */
  public void enableCurrentLimit() {
    setCurrentLimit(m_limit);
  }

  /**
   * Disable current limiting for drivetrain.
   */
  public void disableCurrentLimit() {
    // not completely disabled, 4x80 amps is 240Amps, which is almost 100% of the battery output
    setCurrentLimit(new SupplyCurrentLimitConfiguration(true, 80, 60, 1));
  }

  public boolean getDriveInvert() {
    return driveInvert;
  }

  public void setDriveInvert(boolean invert) {
    driveInvert = invert;
  }

  public void toggleDriveInverted() {
    driveInvert = ! driveInvert;
  }

  public boolean getForzaModeEnabled() {
    return forzaModeEnabled;
  }

  public void setForzaModeEnabled(boolean forzaMode) {
    forzaModeEnabled = forzaMode;
  }

  public void toggleForzaMode() {
    forzaModeEnabled = ! forzaModeEnabled;
  }

  public boolean getSquaredInputs() {
    return squaredInputs;
  }

  public void toggleSquaredInputs() {
    squaredInputs = ! squaredInputs;
  }

}
