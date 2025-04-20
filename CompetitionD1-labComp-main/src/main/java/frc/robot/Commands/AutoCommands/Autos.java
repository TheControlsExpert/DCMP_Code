package frc.robot.Commands.AutoCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer.ScoringPosition;
import frc.robot.Commands.DriveCommands.AutoAlignReef.AutoAlignReef;
import frc.robot.Commands.DriveCommands.AutoAlignReef.FirstPartAutoAlign;
import frc.robot.Commands.DriveCommands.AutoAlignReef.SecondPartAutoAlign;
import frc.robot.Commands.DriveCommands.AutoAlignSource.FirstPartAutoAlignSource;
import frc.robot.Commands.DriveCommands.AutoAlignSource.SecondPartAutoAlignSource;
import frc.robot.Commands.DriveCommands.AutoAlignSource.ThirdPartAutoAlignSource;
import frc.robot.Commands.ElevatorArmCommands.EjectCommand;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Superstructure.Superstructure;
import frc.robot.Subsystems.Superstructure.Superstructure.SuperstructureState;
import frc.robot.Subsystems.Vision.VisionSubsystem;

public class Autos {


    public static Command getTopAuto(Drive drive, Superstructure superstructure, VisionSubsystem vision) {

        return new FirstPartAutoAlign(drive,  superstructure, ScoringPosition.E).andThen(new SecondPartAutoAlign(drive, vision, superstructure)).andThen(new EjectCommand(superstructure, drive, vision)).andThen(new InstantCommand(() -> {drive.runVelocity(new ChassisSpeeds(-0.5, 2.5, 0));})).andThen(new WaitCommand(0.5)).andThen(new FirstPartAutoAlignSource(superstructure, drive)).andThen(new SecondPartAutoAlignSource(drive, superstructure)).andThen(new ThirdPartAutoAlignSource(drive, superstructure)).andThen(new FirstPartAutoAlign(drive, superstructure, ScoringPosition.D)).andThen(new SecondPartAutoAlign(drive, vision, superstructure)).andThen(new EjectCommand(superstructure, drive ,vision)).andThen(new InstantCommand(() -> {drive.runVelocity(new ChassisSpeeds(-0.5, 0, 0));})).andThen(new WaitUntilCommand(() -> (superstructure.current_state.equals(SuperstructureState.HOME_UP)))).andThen(new FirstPartAutoAlignSource(superstructure, drive)).andThen(new SecondPartAutoAlignSource(drive, superstructure));
      
    }

    public static Command getBottomAuto(Drive drive, Superstructure superstructure, VisionSubsystem vision) {
        
            return new FirstPartAutoAlign(drive,  superstructure, ScoringPosition.I).andThen(new SecondPartAutoAlign(drive, vision, superstructure)).andThen(new EjectCommand(superstructure, drive, vision)).andThen(new InstantCommand(() -> {drive.runVelocity(new ChassisSpeeds(-0.5, -2.5, 0));})).andThen(new WaitCommand(0.5)).andThen(new FirstPartAutoAlignSource(superstructure, drive)).andThen(new SecondPartAutoAlignSource(drive, superstructure)).andThen(new ThirdPartAutoAlignSource(drive, superstructure)).andThen(new FirstPartAutoAlign(drive, superstructure, ScoringPosition.K)).andThen(new SecondPartAutoAlign(drive, vision, superstructure)).andThen(new EjectCommand(superstructure, drive ,vision)).andThen(new InstantCommand(() -> {drive.runVelocity(new ChassisSpeeds(-0.5, 0, 0));})).andThen(new WaitUntilCommand(() -> (superstructure.current_state.equals(SuperstructureState.HOME_UP)))).andThen(new FirstPartAutoAlignSource(superstructure, drive)).andThen(new SecondPartAutoAlignSource(drive, superstructure));
           
 
    }


    public static Command getCenterCommand(Drive drive, Superstructure superstructure, VisionSubsystem vision) {
        return new WaitCommand(7.5).andThen(new FirstPartAutoAlign(drive, superstructure, ScoringPosition.G)).andThen(new SecondPartAutoAlign(drive, vision, superstructure)).andThen(new EjectCommand(superstructure, drive, vision));
    }
    
}
