package frc.robot.subsystems;

import edu.wpi.first.networktables.IntegerArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionTablesListener {
    private static VisionTablesListener instance;

    private NetworkTableInstance networkTable;
    private NetworkTable visionTable;
    private IntegerArraySubscriber tagIDSub;
    private IntegerArraySubscriber xCoordsSub;
    private IntegerArraySubscriber yCoordsSub;
    private IntegerArraySubscriber xEulerSub;
    private IntegerArraySubscriber yEulerSub;
    private IntegerArraySubscriber zEulerSub;

    public VisionTablesListener() {
        networkTable = NetworkTableInstance.getDefault();
        visionTable = networkTable.getTable("Vision");
        tagIDSub = visionTable.getIntegerArrayTopic("IDs").subscribe(new long[] {});
        xCoordsSub = visionTable.getIntegerArrayTopic("X Coords").subscribe(new long[] {});
        yCoordsSub = visionTable.getIntegerArrayTopic("Y Coords").subscribe(new long[] {});
        xEulerSub = visionTable.getIntegerArrayTopic("X Euler Angles").subscribe(new long[] {});
        yEulerSub = visionTable.getIntegerArrayTopic("Y Euler Angles").subscribe(new long[] {});
        zEulerSub = visionTable.getIntegerArrayTopic("Z Euler Angles").subscribe(new long[] {});
    }

    public void putInfoOnDashboard() {
        SmartDashboard.putNumberArray("Tag IDs", convertArray(tagIDSub.get()));
        SmartDashboard.putNumberArray("X Coords", convertArray(xCoordsSub.get()));
        SmartDashboard.putNumberArray("Y Coords", convertArray(yCoordsSub.get()));
        SmartDashboard.putNumberArray("X Euler Angles", convertArray(xEulerSub.get()));
        SmartDashboard.putNumberArray("Y Euler Angles", convertArray(yEulerSub.get()));
        SmartDashboard.putNumberArray("Z Euler Angles", convertArray(zEulerSub.get()));
        SmartDashboard.putNumber("Ansh", 0);
    }

    // need to convert each value to double individually, can't typecast entire
    // array
    private double[] convertArray(long[] arr) {
        double[] newArr = new double[arr.length];

        for (int i = 0; i < arr.length; i++) {
            newArr[i] = (double) arr[i];
        }
        return newArr;
    }

    public static VisionTablesListener getInstance() {
        if (instance == null)
            instance = new VisionTablesListener();
        return instance;
    }

}
