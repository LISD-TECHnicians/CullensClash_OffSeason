package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.*;
import com.pathplanner.lib.config.RobotConfig; 
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPLTVController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro; 
import edu.wpi.first.wpilibj.SPI; 
import com.pathplanner.lib.config.ModuleConfig; 
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;


public class CANDriveSubsystem extends SubsystemBase {
  private final SparkMax leftLeader;
  private final SparkMax leftFollower;
  private final SparkMax rightLeader;
  private final SparkMax rightFollower;

  private final ADXRS450_Gyro gyro; 

  private final DifferentialDrive drive;
  private final DifferentialDriveOdometry driveOdometry; 
  private final DifferentialDriveKinematics kinematics; 

  public CANDriveSubsystem() {
    leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushless);
    leftFollower = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushless);
    rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushless);
    rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushless);

    gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

    drive = new DifferentialDrive(leftLeader, rightLeader);
    driveOdometry = new DifferentialDriveOdometry(new Rotation2d(), 
      0, 0, new Pose2d());
    
    kinematics = new DifferentialDriveKinematics(TRACK_WIDTH);

    leftLeader.setCANTimeout(250);
    rightLeader.setCANTimeout(250);
    leftFollower.setCANTimeout(250);
    rightFollower.setCANTimeout(250);


    SparkMaxConfig config = new SparkMaxConfig();
    config.voltageCompensation(12);
    config.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
    config.encoder.positionConversionFactor(ROTATIONS_METER);

    config.follow(leftLeader);
    leftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.follow(rightLeader);
    rightFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config.disableFollowerMode();
    rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config.inverted(true);
    leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    
    RobotConfig robotConfig = new RobotConfig(WEIGHT_KG,
                                          1.5,
                                          new ModuleConfig(.05, 
                                          3.5, 
                                          .60,
                                          DCMotor.getNEO(2).withReduction(8.45),
                                          40,
                                          1),
                                          TRACK_WIDTH
                                              ); // Placeholder Confige to Appease Compiler :)
    
    try {
      robotConfig = RobotConfig.fromGUISettings();
    }
    catch (Exception exp) {
      exp.printStackTrace();
    }

    // Autobuilder configuration must be last!
    AutoBuilder.configure(
        this::getPose, 
        this::resetPose, 
        this::getRobotRelativeSpeeds, 
        (speeds, feedforwards) -> driveRobotRelative(speeds), 
        new PPLTVController(0.02), // Ensure 'V' is included
           robotConfig, 
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this
    );
  }
    
  
  @Override
  public void periodic() {
    
    driveOdometry.update(getGyro(), getLeftEncoderMeters(), getRightEncoderMeters());
  }

  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

  private double getLeftEncoderMeters() {return leftFollower.getEncoder().getPosition();}
  private double getRightEncoderMeters() {return rightFollower.getEncoder().getPosition();} 
  private Rotation2d getGyro() {return gyro.getRotation2d();}
  private Pose2d getPose() {return driveOdometry.getPoseMeters();}  // init dummy pose.

  private void resetPose(Pose2d resetPose) {
    leftFollower.getEncoder().setPosition(0);
    rightFollower.getEncoder().setPosition(0);

    driveOdometry.resetPosition(
      resetPose.getRotation(),
      0.0,
      0.0,
      resetPose
    );
  } 

  private ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(
        new DifferentialDriveWheelSpeeds(
            leftFollower.getEncoder().getVelocity(), 
            rightFollower.getEncoder().getVelocity()
        )
    );
  }
  
  private void driveRobotRelative(ChassisSpeeds chassisSpeed) {
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeed);
    System.out.println("drive: Wheelspeeds L --> " + wheelSpeeds.leftMetersPerSecond);
    System.out.println("drive: Wheelspeeds R --> " + wheelSpeeds.rightMetersPerSecond);
    drive.tankDrive(
        wheelSpeeds.leftMetersPerSecond, 
        wheelSpeeds.rightMetersPerSecond,
        false
    );
  }
}

