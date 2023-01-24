package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class vision extends SubsystemBase {

    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

    public NetworkTable limelight;

    public vision() {
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
    }
    
    public boolean getV(){
        if(tv == 1) return true;
        else return false;
    }

    public double getX() {
        return tx;
    }

    public double getY() {
        return ty;
    }

    public double getArea() {
        return ta;
    }

}
