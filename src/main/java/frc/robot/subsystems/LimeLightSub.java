

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimeLightConsants;

public class LimeLightSub extends SubsystemBase {
  //limelight values
   //= LimelightHelpers.getTX("limeLight");

   
 
   public double steerMotorToTarget(double tx){
 
   double headingError = -tx;
   double steeringAdjust = headingError * LimeLightConsants.Kp;
   if(Math.abs(headingError) > 1.0){
     if(headingError < 0)
         return LimeLightConsants.Kp * headingError + LimeLightConsants.minCommand;
     }
       else{
         return LimeLightConsants.Kp * headingError - LimeLightConsants.minCommand;
     }
     return 0.0;
     
   }

   
   public double calculateDistanceFromTarget(){


     // what is the offset angle of the Limelight to the target
     double targetOffsetAngle_Vertical = LimelightHelpers.getTY(getName());


      // get in degrees and converts into radians
     double angleToGoalDegrees = LimeLightConsants.limelightMountAngleDegrees + targetOffsetAngle_Vertical;
     double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0); // a2
    //d = (h2-h1) / tan(a1+a2)
    // conduct distance formulas using the values from constants and the recent ty value

     double distanceFromLimelightToGoalInches = (LimeLightConsants.goalHeightInches - LimeLightConsants.limelightLensHeightInches) / Math.tan(angleToGoalRadians);

     return distanceFromLimelightToGoalInches;

   }
   
    
}