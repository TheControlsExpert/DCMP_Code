package frc.robot.Commands.DriveCommands.AutoAlignReef;


import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.RobotState;
import frc.robot.Commands.DriveCommands.AutoDriveCommand2;
import frc.robot.Commands.ElevatorArmCommands.EjectCommand;
import frc.robot.RobotState.AlgaeLevel;
import frc.robot.RobotState.DirectionREEF;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Superstructure.Superstructure;
import frc.robot.Subsystems.Superstructure.Superstructure.SuperstructureState;
import frc.robot.Subsystems.Vision.VisionSubsystem;

public class ThirdPartAutoAlign extends Command {
    Drive drive;
    VisionSubsystem vision;
    AutoDriveCommand2 autodriver = new AutoDriveCommand2(2, 0.03);
    double offsetX;
    double offsetY;
    boolean hasResetAtLeastOnce = false;
    Superstructure superstructure;
    CommandXboxController controller;

    DoubleSupplier xSupplier;
    DoubleSupplier ySupplier;
    DoubleSupplier rotationSupplier;

  



    //SwerveDriveOdometry odometryRelative;

    public ThirdPartAutoAlign(Drive drive, VisionSubsystem vision, Superstructure superstructure, CommandXboxController controller) { //SwerveDriveOdometry odometryRelative) {
        this.drive = drive;
        this.vision = vision;
        this.controller = controller;
        this.superstructure = superstructure;

        xSupplier =   () -> -controller.getLeftY();
        ySupplier =   () -> -controller.getLeftX();
        rotationSupplier = () -> -controller.getRightX();
       
        //this.odometryRelative = odometryRelative;
        addRequirements(drive, superstructure.intake, superstructure.wrist, superstructure.elevator, superstructure.pivot);
        
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {

        if (vision.reeftransformX != 0 && vision.reeftransformY != 0 && !superstructure.current_state.equals(SuperstructureState.L1_EJECTED)) { //&& (DriverStation.isTeleop() || )) {

        

        double multiplierY = RobotState.getInstance().getScoringDirection().equals(DirectionREEF.RIGHT) ? -1 : 1;
        if (DriverStation.isAutonomous()) {
            offsetY = - (vision.reeftransformY - FieldConstants.REEF_Y_OFFSET_STEPBACK * multiplierY);
            offsetX = (vision.reeftransformX - FieldConstants.REEF_X_OFFSET_STEPBACK/2);

        }

        else {
         offsetY = - (vision.reeftransformY - FieldConstants.REEF_Y_OFFSET_STEPBACK * multiplierY);
         offsetX = (vision.reeftransformX - FieldConstants.REEF_X_OFFSET_STEPBACK);
        }
         //odometryRelative = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, drive.getRotation(), drive.lastModulePositions, new Pose);
         //hasResetAtLeastOnce = true;


        drive.runVelocity(autodriver.getTargetSpeeds2(offsetX, offsetY, drive.getEstimatedPosition()));
        }


        else if (superstructure.current_state.equals(SuperstructureState.L1_EJECTED)) {


        

      
    


        Translation2d linearVelocity;

        if (controller.rightStick().getAsBoolean()) {
          linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble() / 12, ySupplier.getAsDouble() / 12);
        }

        else {
            linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

        }

              // Calculate angular speed
              double omega = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), 0.1);

         if (controller.rightStick().getAsBoolean()) {
             omega = omega / 12;
         }

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega * drive.getMaxAngularSpeedRadPerSec());
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
    }
}


    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), 0.1);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }


  





    @Override
    public boolean isFinished() {
        //stop aligning when we are less than 1 cm off or segev starts driving
        return  superstructure.current_state.equals(SuperstructureState.L1_EJECTED) && !controller.rightTrigger().getAsBoolean() || Math.hypot(offsetX, offsetY) < 0.02 && !superstructure.current_state.equals(SuperstructureState.L1_EJECTED) || (Math.hypot(controller.getLeftX(), controller.getLeftY()) > 0.15) && (!superstructure.current_state.equals(SuperstructureState.L1_EJECTED) && !superstructure.current_state.equals(SuperstructureState.L2_EJECTED) && !superstructure.current_state.equals(SuperstructureState.L3_EJECTED) && !superstructure.current_state.equals(SuperstructureState.L4_EJECTED));
    }
    

    @Override
    public void end(boolean interrupted) {
       superstructure.setDesiredState(SuperstructureState.HOME_UP);
}



    
}
