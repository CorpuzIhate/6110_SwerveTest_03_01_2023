package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSub;
import frc.robot.subsystems.LimeLightSub;

public class LockOnCMD extends Command {

    


    private final LimeLightSub limeLightSub;
    private final SwerveSub swerveSub;

    public LockOnCMD(SwerveSub swerveSub ,LimeLightSub limeLightSub){
        this.swerveSub = swerveSub;
        this.limeLightSub = limeLightSub; 
        addRequirements(swerveSub);
        addRequirements(limeLightSub);
         
    }

    @Override
    public void execute(){
        double adjust = limeLightSub.steerMotorToTarget(LimelightHelpers.getTX("limelight"));
        double turningSpeed = adjust;
        double ySpeed = 0;
        double xSpeed = 0;
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed,-ySpeed,-turningSpeed);
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        System.out.println(limeLightSub.calculateDistanceFromTarget());
        swerveSub.setModuleStates(moduleStates);
        //swerveSub.setMotors( adjust, -adjust);

    }





    
}