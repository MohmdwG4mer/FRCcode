package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopSwerve extends CommandBase {

        private double rotation;
        private Translation2d translation;
        private boolean fieldRelative;
        private boolean openLoop;

        private Swerve s_Swerve;
        private Joystick controller;
        private int translationAxis;
        private int strafeAxis;
        private int rotationAxis;
        private double speedLimit;
        private final SlewRateLimiter xLimiter, yLimiter, rLimiter;

        /**
         * Driver control
         */
        public TeleopSwerve(Swerve s_Swerve, Joystick controller, int translationAxis, int strafeAxis, int rotationAxis,
                        boolean openLoop) {
                this.s_Swerve = s_Swerve;
                addRequirements(s_Swerve);

                this.controller = controller;
                this.translationAxis = translationAxis;
                this.strafeAxis = strafeAxis;
                this.rotationAxis = rotationAxis;
                this.openLoop = openLoop;
                this.xLimiter = new SlewRateLimiter(7);
                this.yLimiter = new SlewRateLimiter(7);
                this.rLimiter = new SlewRateLimiter(1);

        }

        @Override
        public void execute() {
                fieldRelative = !controller.getRawButton(XboxController.Button.kA.value);
                SmartDashboard.putNumber("SpeedLimit", speedLimit);
                double yAxis = -controller.getRawAxis(translationAxis);
                double xAxis = -controller.getRawAxis(strafeAxis);
                double rAxis = -controller.getRawAxis(rotationAxis);

                /* Deadbands */
                /*
                 * yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis;
                 * xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
                 * rAxis = (Math.abs(rAxis) < Constants.stickDeadband) ? 0 : rAxis;
                 */
                // yAxis = (Math.pow(yAxis, 2) > 0.9 ? 0.9 * Math.signum(yAxis) :
                // Math.pow(yAxis, 2) * Math.signum(yAxis));
                // xAxis = (Math.pow(xAxis, 2) > 0.9 ? 0.9 * Math.signum(xAxis) :
                // Math.pow(xAxis, 2) * Math.signum(xAxis));
                // rAxis = (Math.pow(rAxis, 2) > 0.9 ? 0.9 * Math.signum(rAxis) :
                // Math.pow(xAxis, 2) * Math.signum(rAxis));

                yAxis = (Math.pow(yAxis, 2) * 0.9 > speedLimit) && (Math.abs(yAxis) < Constants.stickDeadband) ? 0
                                : Math.pow(yAxis, 2) * Math.signum(yAxis) * 0.9;
                xAxis = (Math.pow(xAxis, 2) * 0.9 > speedLimit) && (Math.abs(xAxis) < Constants.stickDeadband) ? 0
                                : Math.pow(xAxis, 2) * Math.signum(xAxis) * 0.9;
                rAxis = (Math.pow(rAxis, 2) * 0.9 > speedLimit) && (Math.abs(rAxis) < Constants.stickDeadband) ? 0
                                : Math.pow(rAxis, 2) * Math.signum(rAxis) * 0.9;

                xAxis = xLimiter.calculate(xAxis) * 5 / 4;
                yAxis = yLimiter.calculate(yAxis) * 5 / 4;
                // rAxis = rLimiter.calculate(rAxis) * 2 * 2 * Math.PI;

                translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
                rotation = rAxis * Constants.Swerve.maxAngularVelocity;
                s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
        }
}
