package frc.robot.subsystems.vision;

import com.gos.lib.properties.GosDoubleProperty;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

import java.awt.*;

// A camera pose object that can be updated on the fly using
// the Gos properties library
public class CameraPose {
    GosDoubleProperty m_x;
    GosDoubleProperty m_y;
    GosDoubleProperty m_z;

    GosDoubleProperty m_pitch;
    GosDoubleProperty m_yaw;
    GosDoubleProperty m_roll;

    double m_lastx;
    double m_lasty;
    double m_lastz;

    double m_lastpitch;
    double m_lastyaw;
    double m_lastroll;

    double m_originalx;
    double m_originaly;
    double m_originalz;

    double m_originalpitch;
    double m_originalyaw;
    double m_originalroll;

    String m_camName;

    Mechanism2d topMech;
    MechanismRoot2d topRoot;
    MechanismLigament2d topCamera;

    Mechanism2d leftMech;
    MechanismRoot2d leftRoot;
    MechanismLigament2d leftCamera;

    Mechanism2d frontMech;
    MechanismRoot2d frontRoot;
    MechanismLigament2d frontCamera;

    public CameraPose(String camName, double x, double y, double z,
                      double pitch, double yaw, double roll, boolean isConstant) {
        m_x = new GosDoubleProperty(isConstant, camName + "/x", x);
        m_y = new GosDoubleProperty(isConstant, camName + "/y", y);
        m_z = new GosDoubleProperty(isConstant, camName + "/z", z);

        m_pitch = new GosDoubleProperty(isConstant, camName + "/pitch", pitch);
        m_yaw = new GosDoubleProperty(isConstant, camName + "/yaw", yaw);
        m_roll = new GosDoubleProperty(isConstant, camName + "/roll", roll);

        m_lastx = x;
        m_lasty = y;
        m_lastz = z;

        m_lastpitch = pitch;
        m_lastyaw = yaw;
        m_lastroll = roll;

        m_originalx = x;
        m_originaly = y;
        m_originalz = z;

        m_originalpitch = pitch;
        m_originalyaw = yaw;
        m_originalroll = roll;

        m_camName = camName;

        Preferences.setDouble(camName + "/x", x);

        // IntelliJ got mad :'(
        // Mechanism2d visualization of angles
        topMech = new Mechanism2d(4, 4);
        topRoot = topMech.getRoot(m_camName + "topRoot", 2, 2);
        topCamera = topRoot.append(
                new MechanismLigament2d(m_camName + "topCamera", 3, m_yaw.getValue() + 90, 1.0, new Color8Bit(Color.kFirstRed)));

        SmartDashboard.putData(m_camName + "/Top Visualization", topMech);

        // Front view
        frontMech = new Mechanism2d(4, 4);
        frontRoot = frontMech.getRoot(m_camName + "frontRoot", 2, 2);
        frontCamera = frontRoot.append(
                new MechanismLigament2d(m_camName + "frontCamera", 3, m_pitch.getValue() + 90, 1.0, new Color8Bit(Color.kFirstRed)));

        SmartDashboard.putData(m_camName + "/Front Visualization", frontMech);

        // Side View (left)
        leftMech = new Mechanism2d(4, 4);
        leftRoot = leftMech.getRoot(m_camName + "leftRoot", 2, 2);
        leftCamera = leftRoot.append(
                new MechanismLigament2d(m_camName + "rollCamera", 3, m_roll.getValue() + 90, 1.0, new Color8Bit(Color.kFirstRed)));

        SmartDashboard.putData(m_camName + "/Left Visualization", leftMech);
    }

    public CameraPose(String camName, double x, double y, double z,
                      double pitch, double yaw, double roll) {
        this(camName, x, y, z, pitch, yaw, roll, false);
    }

    public CameraPose(String camName, Translation3d trans, Rotation3d rot, boolean isConstant) {
        this(camName, Units.metersToInches(trans.getX()), Units.metersToInches(trans.getY()), Units.metersToInches(trans.getZ()),
                Units.radiansToDegrees(rot.getX()), Units.radiansToDegrees(rot.getY()), Units.radiansToDegrees(rot.getZ()), isConstant);
    }

    public CameraPose(String camName, Translation3d trans, Rotation3d rot) {
        this(camName, trans, rot, false);
    }

    public boolean hasChanged() {
        Logger.getInstance().recordOutput(m_camName + "/transform", new Pose3d(getTranslation(), getRotation()));

        if (m_lastx != m_x.getValue() ||
                m_lasty != m_y.getValue() ||
                m_lastz != m_z.getValue() ||
                m_lastpitch != m_pitch.getValue() ||
                m_lastyaw != m_yaw.getValue() ||
                m_lastroll != m_roll.getValue()
        ) {
            m_lastx = m_x.getValue();
            m_lasty = m_y.getValue();
            m_lastz = m_z.getValue();
            m_lastpitch = m_pitch.getValue();
            m_lastyaw = m_yaw.getValue();
            m_lastroll = m_roll.getValue();
            return true;
        }
        return false;
    }

    public Translation3d getTranslation() {
        return new Translation3d(m_x.getValue(), m_y.getValue(), m_z.getValue());
    }

    public Rotation3d getRotation() {
        return new Rotation3d(m_pitch.getValue(), m_yaw.getValue(), m_roll.getValue());
    }

    public void updateVisualization() {
        topCamera.setAngle(m_yaw.getValue() + 90);
        leftCamera.setAngle(m_pitch.getValue());
        frontCamera.setAngle(m_roll.getValue());
    }


    public void setOriginal() {
        Preferences.setDouble(m_camName + "/x", m_originalx);
        Preferences.setDouble(m_camName + "/y", m_originaly);
        Preferences.setDouble(m_camName + "/z", m_originalz);

        Preferences.setDouble(m_camName + "/pitch", m_originalpitch);
        Preferences.setDouble(m_camName + "/yaw", m_originalyaw);
        Preferences.setDouble(m_camName + "/roll", m_originalroll);
    }
}
