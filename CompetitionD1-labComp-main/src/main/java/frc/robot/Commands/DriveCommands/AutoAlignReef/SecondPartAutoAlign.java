package frc.robot.Commands.DriveCommands.AutoAlignReef;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Robot.levelscore;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.Commands.DriveCommands.AutoDriveCommand2;
import frc.robot.Commands.ElevatorArmCommands.EjectCommand;
import frc.robot.RobotState.DirectionREEF;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Superstructure.Superstructure;
import frc.robot.Subsystems.Superstructure.Superstructure.SuperstructureState;
import frc.robot.Subsystems.Vision.VisionSubsystem;

public class SecondPartAutoAlign extends Command {
    Drive drive;
    VisionSubsystem vision;
    AutoDriveCommand2 autodriver = new AutoDriveCommand2(2.35, 0.03);
    double offsetX;
    double offsetY;
    boolean isOnSecondPart = false;
    Superstructure superstructure;
    boolean hasResetGyro = false;
    //SwerveDriveOdometry odometryRelative;

    public SecondPartAutoAlign(Drive drive, VisionSubsystem vision, Superstructure superstructure) { //SwerveDriveOdometry odometryRelative) {
        this.drive = drive;
        this.vision = vision;
        this.superstructure = superstructure;
        //this.odometryRelative = odometryRelative;
        addRequirements(drive, superstructure.intake, superstructure.wrist, superstructure.elevator, superstructure.pivot);
        
    }

    @Override
    public void initialize() {
        isOnSecondPart = false;
        hasResetGyro = false;
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("hasResetGyro", hasResetGyro);

        if (vision.reeftransformX != 0.0 && vision.reeftransformY != 0.0 && vision.currentRotation != 0.0 && !hasResetGyro && Math.hypot(vision.reeftransformX, vision.reeftransformY) < 0.50) {
            drive.resetGyro(vision.currentRotation);
           



        }








        if (vision.reeftransformX != 0.0 && vision.reeftransformY != 0.0 && !isOnSecondPart) {

        

        double multiplierY = RobotState.getInstance().getScoringDirection().equals(DirectionREEF.RIGHT) ? -1 : 1;
        if (Robot.CurrnetLevelPosition.equals(levelscore.Level1)) {
            offsetY = - (vision.reeftransformY);
            //add a bigger offset for the xvalue so we dont hit the reef first
            offsetX = (vision.reeftransformX - (0.40+ 0.1));
        }

        else {
            offsetY = - (vision.reeftransformY - FieldConstants.REEF_Y_OFFSET * multiplierY);
            offsetX = (vision.reeftransformX - (FieldConstants.REEF_X_OFFSET + 0.1));

        }
//changed from 0.03
        if (Math.abs(offsetY) < 0.05) {
            isOnSecondPart = true;
        }
         
         

         //odometryRelative = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, drive.getRotation(), drive.lastModulePositions, new Pose);
        


        drive.runVelocity(autodriver.getTargetSpeeds2(offsetX, offsetY, drive.getEstimatedPosition()));
        }

        else if (isOnSecondPart && vision.reeftransformX != 0 && vision.reeftransformY != 0) {
            double multiplierY = RobotState.getInstance().getScoringDirection().equals(DirectionREEF.RIGHT) ? -1 : 1;
        if (Robot.CurrnetLevelPosition.equals(levelscore.Level1)) {
            offsetY = - (vision.reeftransformY);
            //remove offset for the xvalue so we dont hit the reef first
            offsetX = (vision.reeftransformX - (0.40));
        }

        else if (Robot.CurrnetLevelPosition.equals(levelscore.Level4)) {
            offsetY = - (vision.reeftransformY - FieldConstants.REEF_Y_OFFSET * multiplierY);
            offsetX = (vision.reeftransformX - (0.385));
           // offsetX = (vision.reeftransformX - (0.37));
          // offsetX = vision.reeftransformX - 0.37;
         // offsetX = vision.reeftransformX - 0.4;

        }

        else {
            offsetY = - (vision.reeftransformY - FieldConstants.REEF_Y_OFFSET * multiplierY);
            offsetX = (vision.reeftransformX - (FieldConstants.REEF_X_OFFSET));

        }

         //odometryRelative = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, drive.getRotation(), drive.lastModulePositions, new Pose);

        drive.runVelocity(autodriver.getTargetSpeeds2(offsetX, offsetY, drive.getEstimatedPosition()));
        }

        }


      

    

//changed tolerances 
    @Override
    public boolean isFinished() {
        SmartDashboard.putNumber("offsX", Math.abs(offsetX));
        SmartDashboard.putNumber("offsetY", Math.abs(offsetY));
        
        return  ((Math.abs(offsetX) < 0.0075 && Math.abs(offsetY) < 0.0075 && (Robot.CurrnetLevelPosition.equals(levelscore.Level4) || Robot.CurrnetLevelPosition.equals(levelscore.Level1))) || (Math.hypot(offsetX, offsetY) < 0.03 && (Robot.CurrnetLevelPosition.equals(levelscore.Level2) || Robot.CurrnetLevelPosition.equals(levelscore.Level3)))) && isOnSecondPart &&  Math.abs(Units.radiansToDegrees(MathUtil.angleModulus(RobotState.getInstance().getScoringPose().getRotation().minus((drive.getEstimatedPosition()).getRotation()).getRadians()))) < 1;
    }
    

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            superstructure.setDesiredState(SuperstructureState.HOME_UP);
        }

        drive.runVelocity(new ChassisSpeeds());

       
    }
}
