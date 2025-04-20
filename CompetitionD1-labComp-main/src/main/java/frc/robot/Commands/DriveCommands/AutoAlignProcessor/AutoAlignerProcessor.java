package frc.robot.Commands.DriveCommands.AutoAlignProcessor;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.DriveCommands.AutoDriveCommand;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Superstructure.Superstructure;
import frc.robot.Subsystems.Superstructure.Superstructure.SuperstructureState;

public class AutoAlignerProcessor extends Command {
    AutoDriveCommand autoDriver = new AutoDriveCommand(2.1, 0.05, 0, 1);

    Superstructure superstructure;
    CommandXboxController controller;
    //Drive drive;


    public AutoAlignerProcessor(Superstructure superstructure, CommandXboxController controller) {
        this.superstructure = superstructure;
        this.controller = controller;
        //this.drive = drive;
        addRequirements(superstructure.intake, superstructure.pivot, superstructure.wrist, superstructure.elevator);
    }





    @Override
    public void initialize() {
        superstructure.setDesiredState(SuperstructureState.ALGAE_EJECT);
    }


    @Override
    public void execute() {

        //Pose2d targetPose = 
       // autoDriver.getTargetSpeeds(drive.getEstimatedPosition(), null)
        
    }


    @Override
    public void end(boolean interrupted) {
        if (!controller.rightTrigger().getAsBoolean()) {
            superstructure.setDesiredState(SuperstructureState.HOME_ALGAE);
        }
    }

    


    
}
