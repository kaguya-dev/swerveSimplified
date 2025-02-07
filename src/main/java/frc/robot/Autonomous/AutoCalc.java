package frc.robot.Autonomous;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.SwerveUtils.SwerveWheel;

public class AutoCalc {

    public void getMeters(SwerveWheel[] wheels){

        for (int i = 0; i < wheels.length; i++) {
            double meters = wheels[i].getPosition() * Constants.kWheelCircuferenceMeters;
            SmartDashboard.putNumber("metros" + (i + 1), meters);
        }

    }

    
}
    

