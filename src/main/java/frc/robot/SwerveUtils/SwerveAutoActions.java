package frc.robot.SwerveUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class SwerveAutoActions {
    public void goToPosition(SwerveWheel[] wheels, Pose2d currentPose, Pose2d desiredPose){
        double catX = (desiredPose.getX() - currentPose.getX());
        double catY = (desiredPose.getY() - currentPose.getY());
        double distance = Math.hypot(catX,catY);
        double distanceReached = 0;
        double angle = Units.radiansToRotations(Math.atan2(catX,catY));
        
        for(int i = 0; i < wheels.length; i++){
            if(catX != 0 && catY != 0){ 
            wheels[i].setDirection(angle);
            wheels[i].setSpeed(Constants.MAX_SPEED);
            }
        }
    }
}
