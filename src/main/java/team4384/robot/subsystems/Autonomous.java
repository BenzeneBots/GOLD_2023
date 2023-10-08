package team4384.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team4384.robot.constants.SwerveMap;

public class Autonomous {
    private Swerve s_Swerve;
    private AHRS gyro;
    public Autonomous(Swerve m_swerve, AHRS gyro) {
        this.s_Swerve = m_swerve;
        this.gyro = gyro;
    }
    private static double microTime(double seconds) {
        return seconds * 1000000;
    }
    public void basic() {
        long initTime = RobotController.getFPGATime();

        while (RobotController.getFPGATime() - initTime < microTime(1.3)) {
            s_Swerve.drive(
                    new Translation2d(.5, 0).times(SwerveMap.maxSpeed),
                    0 * SwerveMap.maxAngularVelocity,
                    false,
                    false
            );
        }

        s_Swerve.drive(
                new Translation2d(.0, 0).times(SwerveMap.maxSpeed),
                0 * SwerveMap.maxAngularVelocity,
                false,
                false
        );

        initTime = RobotController.getFPGATime();
        while (RobotController.getFPGATime() - initTime < microTime(3.5)) {
            s_Swerve.drive(
                    new Translation2d(.3, 0).times(SwerveMap.maxSpeed),
                    0 * SwerveMap.maxAngularVelocity,
                    false,
                    true
            );
        }

        s_Swerve.drive(
                new Translation2d(.0, 0).times(SwerveMap.maxSpeed),
                0 * SwerveMap.maxAngularVelocity,
                false,
                true
        );
    }

    public void cone() {
        long initTime = RobotController.getFPGATime();

        while (RobotController.getFPGATime() - initTime < microTime(1.3)) {
            s_Swerve.drive(
                    new Translation2d(.5, 0).times(SwerveMap.maxSpeed),
                    0 * SwerveMap.maxAngularVelocity,
                    false,
                    false
            );
        }

        s_Swerve.drive(
                new Translation2d(.0, 0).times(SwerveMap.maxSpeed),
                0 * SwerveMap.maxAngularVelocity,
                false,
                false
        );
    }

    public void chargingStation() {
        long initTime = RobotController.getFPGATime();
        long initTime1 = RobotController.getFPGATime();
        double roll = this.gyro.getRoll();
        boolean isFirst = false;

        while (RobotController.getFPGATime() - initTime < microTime(10)) {
            if (this.gyro.getRoll() == roll) continue;
            System.out.println(this.gyro.getRoll()+"|"+roll);
            if (RobotController.getFPGATime() - initTime > microTime(1.9)) {
                if (Math.floor(this.gyro.getRoll()) < Math.floor(roll)) {
                    System.out.println("Blaance");
                    s_Swerve.drive(
                            new Translation2d(0, .1).times(SwerveMap.maxSpeed),
                            0 * SwerveMap.maxAngularVelocity,
                            false,
                            true
                    );
                    isFirst = true;
                    break;
                }
            }
            else {
                s_Swerve.drive(
                        new Translation2d(-.7, 0).times(SwerveMap.maxSpeed),
                        0 * SwerveMap.maxAngularVelocity,
                        false,
                        true
                );
                roll = gyro.getRoll();
                continue;
            }

           initTime1 = RobotController.getFPGATime();

            s_Swerve.drive(
                    new Translation2d(-.2, 0).times(SwerveMap.maxSpeed),
                    0 * SwerveMap.maxAngularVelocity,
                    false,
                    true
            );

            roll = gyro.getRoll();
        }
    }
}
