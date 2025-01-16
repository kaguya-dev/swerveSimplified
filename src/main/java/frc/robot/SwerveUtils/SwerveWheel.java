package frc.robot.SwerveUtils;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class SwerveWheel {
    private final SparkMax motorDirection;
    private final SparkMax motorSpeed;
    private final PIDController pids;

    private final CANcoder cancoder;

    public SwerveWheel(SparkMax motorDirection, SparkMax motorSpeed, CANcoder cancoder) {
        this.motorDirection = motorDirection;
        this.motorSpeed = motorSpeed;
        this.cancoder = cancoder;
        pids = new PIDController(Constants.KP_Swerve_ANGLE, Constants.KI_Swerve_ANGLE, Constants.KD_Swerve_ANGLE);
    }

    public void setDirection(double setpoint){
        Math.tan
        pids.setSetpoint(setpoint);

        motorDirection.set(pids.calculate(cancoder.getAbsolutePosition().getValueAsDouble()));
    }
}
