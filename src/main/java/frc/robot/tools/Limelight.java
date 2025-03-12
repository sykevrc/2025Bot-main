package frc.robot.tools;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawDetection;
import frc.robot.LimelightHelpers.RawFiducial;

public class Limelight {
    //private PoseEstimate limelightMeasurement = null;
    //private boolean isSim = false;
	//private String name = "";
	private RawDetection[] rawDetections;
	private RawFiducial[] rawFiducials;
	private int[] detections;
	private int counter = 0;
	//private boolean rejectUpdate = false;
	private LimelightHelpers.PoseEstimate mt2;
	private Pose2d robotPose2d = null;
	private double[] poseArray = new double[3];

    public Limelight() {
		LimelightHelpers.setCameraPose_RobotSpace(
			"limelight", 0,0,0,0,0,0
		);
	}

	public PoseEstimate getPose2d(Pose2d robotPose2d) {

		LimelightHelpers.SetRobotOrientation(
									"limelight",
									robotPose2d.getRotation().getDegrees(),
									0,
									0,
									0,
									0,
									0);

		mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

							if (mt2 != null) {
								poseArray[0] = mt2.pose.getX();
								poseArray[1] = mt2.pose.getY();
								poseArray[2] = mt2.pose.getRotation().getDegrees();
							}

		return mt2;
	}

	public double[] getPoseArray() {
		return poseArray;
	}

	public boolean hasTarget() {
		return LimelightHelpers.getTV(Constants.LimelightConstants.name);
	}

	public void setPipelineIndex(int index) {
		LimelightHelpers.setPipelineIndex(Constants.LimelightConstants.name, index);
	}

	public double getDistancToTargetFromRobot(int target) {
		rawFiducials = LimelightHelpers.getRawFiducials(Constants.LimelightConstants.name);

		return rawFiducials[target].distToRobot;
	}

	public int[] getDetections() {
		rawDetections = LimelightHelpers.getRawDetections(Constants.LimelightConstants.name);

		counter = 0;
		detections = new int[rawDetections.length];
		for (RawDetection detection : rawDetections) {
			detections[counter] = detection.classId;
			counter++;
		}

		return detections;
	}
}
