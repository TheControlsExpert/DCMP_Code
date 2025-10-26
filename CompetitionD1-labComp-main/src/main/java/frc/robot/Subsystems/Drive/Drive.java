package frc.robot.Subsystems.Drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.Constants.Mode;
import frc.robot.Constants.SwerveConstants;
import java.util.Arrays;
import java.util.HashMap;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {

  //pose estimation
  static final double ODOMETRY_FREQUENCY = 200;
  public LinearFilter filter = LinearFilter.movingAverage(10);
  public Pose2d estimatedPose = new Pose2d(0, 0, new Rotation2d());
  public Pose2d odometryPose = new Pose2d();
  public Pose2d lastodometrypose = new Pose2d();

  private double[] sampleTimestamps;
  private int sampleCount;
  private SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
  private SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
  private Twist2d twist = new Twist2d();

  public boolean resettingLocalization = false;
  
  SwerveModuleState[] mods = new SwerveModuleState[] {
    new SwerveModuleState(),
    new SwerveModuleState(),
    new SwerveModuleState(),
    new SwerveModuleState()
  };
  

  static final Lock odometryLock = new ReentrantLock();
  
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  //private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics = SwerveConstants.swerveKinematics;
  private Rotation2d rawGyroRotation = new Rotation2d();
  public SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator SwervePoseEstimator = new SwerveDrivePoseEstimator(kinematics, getRotation(), lastModulePositions, new Pose2d(0, 0, new Rotation2d()), VecBuilder.fill(0.005,0.005, Radians.convertFrom(5, Degrees)), VecBuilder.fill(0.05, 0.05, 999999999) );
  
  
  private int numTimes = 0;
  private double lastgyro = 0.0;
      
      
  public Drive(
    GyroIO gyroIO,
    ModuleIO flModuleIO,
    ModuleIO frModuleIO,
    ModuleIO blModuleIO,
    ModuleIO brModuleIO) {
         
    this.gyroIO = gyroIO;
          
    modules[0] = new Module(flModuleIO,0, SwerveConstants.Mod0.constants);
    modules[1] = new Module(frModuleIO, 1, SwerveConstants.Mod1.constants);
    modules[2] = new Module(blModuleIO, 2, SwerveConstants.Mod2.constants);
    modules[3] = new Module(brModuleIO, 3,SwerveConstants.Mod3.constants);
      
    SmartDashboard.putData("Field", m_field);
    PhoenixOdometryThread.getInstance().start();

 }
     
        @Override
  public void periodic() {
        
    m_field.setRobotPose(getEstimatedPosition());
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    
    for (var module : modules) {
      module.periodic();
    }
    
    odometryLock.unlock();

    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }  
          // Update odometry
    double[] sampleTimestamps =  modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      modulePositions = new SwerveModulePosition[4];
      moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] = new SwerveModulePosition(
          modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
          modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      } 
      twist = kinematics.toTwist2d(moduleDeltas);
      if (gyroInputs.connected) {
              // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
            
      } 
      else {
              // Use the angle delta from the kinematics and module deltas
          Twist2d twist = kinematics.toTwist2d(moduleDeltas);
          rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }
          SwervePoseEstimator.updateWithTime(sampleTimestamps[i],  rawGyroRotation, modulePositions);
    }
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
  }

  public double[] getWheelRadiusCharacterizationPosition() {
    return Arrays.stream(modules).mapToDouble(Module::getPositionRadians).toArray();
  }

  public void resetGyro() {
    int addOn = DriverStation.getAlliance().get().equals(Alliance.Red) ? 180 : 0;
    gyroIO.resetGyro(Rotation2d.fromDegrees(addOn));
  }

  public void resetGyro(double degrees) {
    gyroIO.resetGyro(Rotation2d.fromDegrees(degrees));
  }

  public void runVelocity(ChassisSpeeds speeds) {
      // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
     // ChassisSpeeds heightLimit = getNewTargetVelocity(discreteSpeeds);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, getMaxLinearSpeedMetersPerSec());
  
      // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
        modules[i].runSetpoint(setpointStates[i]);
    }
  }
  
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }


 
  //limits max acceleration
  public ChassisSpeeds getNewTargetVelocity(ChassisSpeeds vel) {
    Vector<N2> accel =  VecBuilder.fill(getRobotRelativeSpeeds().vxMetersPerSecond, getRobotRelativeSpeeds().vyMetersPerSecond).minus(VecBuilder.fill(vel.vxMetersPerSecond, vel.vyMetersPerSecond));
    ChassisSpeeds newvel = vel;
    Vector<N2> velFixed = VecBuilder.fill(getRobotRelativeSpeeds().vxMetersPerSecond, getRobotRelativeSpeeds().vyMetersPerSecond);
    double maxAccel = (-0.08558 * Superstructure.encoderElevator + 4.426);
     //SmartDashboard.putNumber("max Accel", maxAccel);
     if (accel.norm() > maxAccel) {
       accel = accel.times( maxAccel/ accel.norm());
       velFixed = velFixed.plus(accel);
       SmartDashboard.putNumber("resulting velocity", velFixed.norm());
       SmartDashboard.putNumber("wanted velocity", Math.hypot(vel.vxMetersPerSecond, vel.vyMetersPerSecond));
       return new ChassisSpeeds(velFixed.get(0), velFixed.get(1), vel.omegaRadiansPerSecond);
     }
      return vel;
    }
  
    /** Stops the drive. */
    public void stop() {
      runVelocity(new ChassisSpeeds());
    }
  

    private SwerveModuleState[] getModuleStates() {
      SwerveModuleState[] states = new SwerveModuleState[4];
      for (int i = 0; i < 4; i++) {
        states[i] = modules[i].getState();
      }
      return states;
    }


    public ChassisSpeeds getRobotRelativeSpeeds() {
      return kinematics.toChassisSpeeds(getModuleStates());
    }

    private static Translation2d convertSwerveStateToVelocityVector(SwerveModuleState swerveModuleState) {
      return new Translation2d(swerveModuleState.speedMetersPerSecond, swerveModuleState.angle);
  }

    public static double getSkiddingRatio(SwerveModuleState[] swerveStatesMeasured, SwerveDriveKinematics swerveDriveKinematics) {
      final double angularVelocityOmegaMeasured = swerveDriveKinematics.toChassisSpeeds(swerveStatesMeasured).omegaRadiansPerSecond;
      final SwerveModuleState[] swerveStatesRotationalPart = swerveDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, angularVelocityOmegaMeasured));
      final double[] swerveStatesTranslationalPartMagnitudes = new double[swerveStatesMeasured.length];

      for (int i =0; i < swerveStatesMeasured.length; i++) {
          final Translation2d swerveStateMeasuredAsVector = convertSwerveStateToVelocityVector(swerveStatesMeasured[i]),
                  swerveStatesRotationalPartAsVector = convertSwerveStateToVelocityVector(swerveStatesRotationalPart[i]),
                  swerveStatesTranslationalPartAsVector = swerveStateMeasuredAsVector.minus(swerveStatesRotationalPartAsVector);
          swerveStatesTranslationalPartMagnitudes[i] = swerveStatesTranslationalPartAsVector.getNorm();
      }

      double maximumTranslationalSpeed = 0, minimumTranslationalSpeed = Double.POSITIVE_INFINITY;
      for (double translationalSpeed:swerveStatesTranslationalPartMagnitudes) {
          maximumTranslationalSpeed = Math.max(maximumTranslationalSpeed, translationalSpeed);
          minimumTranslationalSpeed = Math.min(minimumTranslationalSpeed, translationalSpeed);
      }

      return maximumTranslationalSpeed / minimumTranslationalSpeed;
  }
  
  
    /** Returns the module positions (turn angles and drive positions) for all of the modules. */
    private SwerveModulePosition[] getModulePositions() {
      SwerveModulePosition[] states = new SwerveModulePosition[4];
      for (int i = 0; i < 4; i++) {
        states[i] = modules[i].getPosition();
      }
      return states;
    }
  
    /** Returns the measured chassis speeds of the robot. */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    private ChassisSpeeds getChassisSpeeds() {
      return kinematics.toChassisSpeeds(getModuleStates());
    }
  
    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
      double[] values = new double[4];
      for (int i = 0; i < 4; i++) {
        values[i] = modules[i].getWheelRadiusCharacterizationPosition();
      }
      return values;
    }
  

    public void addVision(Pose2d pose, double timestamp, double[] visionstds) {
      SmartDashboard.putBoolean("what the sigma", true);
      Vector<N3> stds = VecBuilder.fill(visionstds[0], visionstds[1], 9999999);
      SwervePoseEstimator.addVisionMeasurement(new Pose2d(pose.getTranslation(), SwervePoseEstimator.getEstimatedPosition().getRotation()), timestamp, stds);
      
    }

    public Pose2d getEstimatedPosition() {
      return new Pose2d(SwervePoseEstimator.getEstimatedPosition().getTranslation(), gyroInputs.yawPosition);
    }


    public void resetPosition(Pose2d pose) {
    }
  
//  
  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  // @AutoLogOutput(key = "Odometry/Robot")
  // public Pose2d getPose() {
  //   return poseEstimator.getEstimatedPosition();
  // }

  /** Returns the current odometry rotation. */
  public double getRotationLL() {
    return gyroInputs.yawPosition.getDegrees()+180;
  }

  public Rotation2d getRotation() {
    return gyroInputs.yawPosition;
  }

 
  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    if (DriverStation.isAutonomous()) {
      return 5.5;
    }
    else {
      return 4;
    }
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() * 1.5  / SwerveConstants.DRIVE_BASE_RADIUS;
  }


  public Command getPathFollowingCommand(String pathName) {
    try {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
     return AutoBuilder.followPath(path);
    }

    catch (Exception e) {
      DriverStation.reportError("oops, couldnt find path:" + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
    
  }


  public double getGyroSpeed() {
    return gyroIO.getRate();
  }


 
}
