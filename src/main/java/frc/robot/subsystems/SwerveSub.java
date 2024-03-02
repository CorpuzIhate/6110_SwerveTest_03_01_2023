package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.proto.SwerveModulePositionProto;
import edu.wpi.first.math.proto.Kinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import com.kauailabs.navx.frc.AHRS;

import java.io.FilePermission;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;


import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.DriverStation;





public class SwerveSub extends SubsystemBase {
    public final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    public final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);




    public final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    public final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);
    private final SwerveModuleState[] mySwerveStates = new SwerveModuleState[]{ // used for debugging to Adavantage Scope
        frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()
    };


    private final SwerveModule swerveModules[] = new SwerveModule[]{
        frontLeft,frontRight,
        backLeft, backRight
    };

    private Field2d field = new Field2d();


    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, 
    new Rotation2d(0), getModulePositionsAuto() );


    
    private final AHRS gyro = new AHRS(SerialPort.Port.kUSB1); // change later to usb port and get kMXP library





    public SwerveSub(){



 new Thread(() -> {  /// try catch function is a fancy if else statement
        try{              // it tries to run a thread of resseting the gryo but if it exception e happens it stops 
            Thread.sleep(1000);
        }catch (Exception e){
        }
        }).start();

        zeroHeading();  
        
          AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );

    
    }
    @Override
    public void periodic(){

        odometer.update(getRotation2d(),  getModulePositionsAuto()
        );

        SmartDashboard.putNumber("robot Heading", getHeading());
        SmartDashboard.putString("robot location", getPose().getTranslation().toString());


        SwerveModulePosition[] debugModulePosition = getModulePositionsAuto();
        for(int i = 0; i <= 3; ++i){
            SmartDashboard.putString("SwerveModulePostions [" + i + "]" , "distance : " + debugModulePosition[i].distanceMeters
            + "Speeds : " + debugModulePosition[i].angle);
}
        Logger.recordOutput("heading",getHeading());


        frontLeft.sendToDashboard();
        frontRight.sendToDashboard();
        backLeft.sendToDashboard();
        backRight.sendToDashboard();
    }
    public Pose2d getPose(){
        return odometer.getPoseMeters();
    } 
    public void resetPose(Pose2d pose){
        odometer.resetPosition(gyro.getRotation2d(), getModulePositionsAuto() , pose);
    }
     public ChassisSpeeds getSpeeds() {
         return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates()); // error here
    }


    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds){
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
    }




    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    // proportaionally decreases the change the speeds so driver always had control of robot
        frontRight.setDesiredState(desiredStates[0]);        
        frontLeft.setDesiredState(desiredStates[1]); //1 
        backRight.setDesiredState(desiredStates[2]); //2                     
        backLeft.setDesiredState(desiredStates[3]); // 3



        //ouputs to Adavantage Log

        // log desired states is an array that orders the desired states in the order 
        // Advantage Log wants ( FL,FR, BL, BR )
        SwerveModuleState[] LogDesiredStates = new SwerveModuleState[]{desiredStates[1], desiredStates[0],
         desiredStates[3], desiredStates[2]};


        Logger.recordOutput("CurrentStates", mySwerveStates);
        Logger.recordOutput("DesiredStates",LogDesiredStates);
    
    }

// get positions
public SwerveModulePosition[] getModulePositionsAuto() {
    SwerveModulePosition[] positions = new SwerveModulePosition[swerveModules.length];
    for (int i = 0; i < swerveModules.length; i++) {
      positions[i] = swerveModules[i].getSwerveModulePosition();
    }
    return positions;
  }


    public void zeroHeading(){
        gyro.reset();
    }

    public double getHeading(){
        return Math.IEEEremainder(-gyro.getAngle(), 360); //puts the value between 0 and 360 because gryo is naturally continous
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    } // converts into Rotation2d








    


      public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[swerveModules.length];
        for (int i = 0; i < swerveModules.length; i++) {
          states[i] = swerveModules[i].getState();
        }
        return states;
      }



    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();

    }






}