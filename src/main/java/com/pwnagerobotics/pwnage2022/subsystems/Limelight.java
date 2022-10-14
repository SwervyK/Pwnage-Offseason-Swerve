package com.pwnagerobotics.pwnage2022.subsystems;

import com.pwnagerobotics.pwnage2022.Constants;
import com.pwnagerobotics.pwnage2022.RobotState;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.subsystems.Subsystem;
import com.team254.lib.vision.TargetInfo;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;

/**
 * Subsystem for interacting with the Limelight 2
 */
public class Limelight extends Subsystem {

    public static class LimelightConstants {
        public String kName = "";
        public String kTableName = "";
        public double kHeight = 0.0;
        public Pose2d kTurretToLens = Pose2d.identity();
        public Rotation2d kHorizontalPlaneToLens = Rotation2d.identity();
    }

    private NetworkTable mNetworkTable;

    public Limelight(LimelightConstants constants) {
        mConstants = constants;
        mNetworkTable = NetworkTableInstance.getDefault().getTable(constants.kTableName);
    }

    public static class PeriodicIO {
        // INPUTS
        public double latency;
        public int givenLedMode;
        public int givenPipeline;
        public double xAngle;
        public double yAngle;
        public double area;

        // OUTPUTS
        public int ledMode = 0; // 0 - use pipeline mode, 1 - off, 2 - blink, 3 - on
        public int camMode = 0; // 0 - vision processing, 1 - driver camera
        public int pipeline = 0; // 0 - 9
        public int stream = 2; // sets stream layout if another webcam is attached
        public int snapshot = 0; // 0 - stop snapshots, 1 - 2 Hz
    }

    private LimelightConstants mConstants = null;
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private boolean mOutputsHaveChanged = true;
    // private double[] mZeroArray = new double[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    private boolean mSeesTarget = false;

    public String getName() {
        return mConstants.kName;
    }

    public Pose2d getTurretToLens() {
        return mConstants.kTurretToLens;
    }

    public double getLensHeight() {
        return mConstants.kHeight;
    }

    public Rotation2d getHorizontalPlaneToLens() {
        return mConstants.kHorizontalPlaneToLens;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.latency = mNetworkTable.getEntry("tl").getDouble(0) / 1000.0 + Constants.kImageCaptureLatency;
        mPeriodicIO.givenLedMode = (int) mNetworkTable.getEntry("ledMode").getDouble(1.0);
        mPeriodicIO.givenPipeline = (int) mNetworkTable.getEntry("pipeline").getDouble(0);
        mPeriodicIO.xAngle = mNetworkTable.getEntry("tx").getDouble(0.0);
        mPeriodicIO.yAngle = mNetworkTable.getEntry("ty").getDouble(0.0);
        mPeriodicIO.area = mNetworkTable.getEntry("ta").getDouble(0.0);
        mSeesTarget = mNetworkTable.getEntry("tv").getDouble(0) == 1.0;
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mPeriodicIO.givenLedMode != mPeriodicIO.ledMode || mPeriodicIO.givenPipeline != mPeriodicIO.pipeline) {
            mOutputsHaveChanged = true;
        }
        if (mOutputsHaveChanged) {
            mNetworkTable.getEntry("ledMode").setNumber(mPeriodicIO.ledMode);
            mNetworkTable.getEntry("camMode").setNumber(mPeriodicIO.camMode);
            mNetworkTable.getEntry("pipeline").setNumber(mPeriodicIO.pipeline);
            mNetworkTable.getEntry("stream").setNumber(mPeriodicIO.stream);
            mNetworkTable.getEntry("snapshot").setNumber(mPeriodicIO.snapshot);

            mOutputsHaveChanged = false;
        }
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putBoolean(mConstants.kName + ": Has Target", mSeesTarget);
        // SmartDashboard.putNumber(mConstants.kName + ": Pipeline Latency (ms)", mPeriodicIO.latency);
    }

    public enum LedMode {
        PIPELINE, OFF, BLINK, ON
    }

    public synchronized void setLed(LedMode mode) {
        if (mode.ordinal() != mPeriodicIO.ledMode) {
            //System.out.println("setLed: " + mode);
            //System.out.println("setLed: " + mPeriodicIO.ledMode);
            //System.out.println("-------------");
            mPeriodicIO.ledMode = mode.ordinal();
            mOutputsHaveChanged = true;
        }
    }

    public synchronized void setPipeline(int mode) {
        if (mode != mPeriodicIO.pipeline) {
            RobotState.getInstance().resetVision();
            mPeriodicIO.pipeline = mode;

            //System.out.println(mPeriodicIO.pipeline + ", " + mode);
            mOutputsHaveChanged = true;
        }
    }

    public synchronized void triggerOutputs() {
        mOutputsHaveChanged = true;
    }

    public synchronized int getPipeline() {
        return mPeriodicIO.pipeline;
    }

    public synchronized boolean seesTarget() {
        return mSeesTarget;
    }

    /**
     * @return two targets that make up one hatch/port or null if less than two targets are found
     */
    public synchronized List<TargetInfo> getTarget() {
        // boolean lookingForBall = (mConstants.kName != Constants.kFrontLimelightConstants.kName && getPipeline() == Constants.kAuxPipeline);
        List<TargetInfo> targets = getTargetInfo();
        
        // System.out.println(seesTarget());
        // System.out.println(targets != null);
        // System.out.println("---------");
        if (seesTarget() && targets != null)
        {
            //System.out.println("Camera returned a target from getTarget()");
            return targets;
        }

        return null;
    }

    private synchronized List<TargetInfo> getTargetInfo(){
        try{
            // System.out.println("----------------");
            // System.out.println("xAngle: " +mPeriodicIO.xAngle);
            // System.out.println("yAngle: " +mPeriodicIO.yAngle);
            var nY = mPeriodicIO.xAngle / (Constants.kHorizontalFOV_1X / 2.0); // camera x axis is robot y axis
            var nZ = mPeriodicIO.yAngle / (Constants.kVerticalFOV_1X / 2.0); // camera y axis is robot z axis
            // System.out.println("nY: " + nY);
            // System.out.println("nZ: " + nZ);
            // System.out.println("kVPW_1X: " + Constants.kVPW_1X);
            // System.out.println("kVPH_1X: " + Constants.kVPH_1X);
            var targets = new ArrayList<TargetInfo>();
            var y = Constants.kVPW_1X / 2.0 * nY;
            var z = Constants.kVPH_1X / 2.0 * nZ;
            //  System.out.println("y: " +y);
            //  System.out.println("z: " +z);
            var target = new TargetInfo(y, z);
            targets.add(target);
            // System.out.println("Returning "+targets.size()+" targets.");
            return targets;
        } catch (Exception ex) {
            //System.out.println(ex.getMessage());
            return null;
        }
    }

    // private synchronized List<TargetInfo> getRawTapeTargetInfos() {
    //     List<double[]> corners = getTopCorners();
    //     if (corners == null) {
    //         //System.out.println("returning null because corners is null");
    //         return null;
    //     }

    //     double slope = 1.0;
    //     if (Math.abs(corners.get(1)[0] - corners.get(0)[0]) > Util.kEpsilon) {
    //         slope = (corners.get(1)[1] - corners.get(0)[1]) /
    //                 (corners.get(1)[0] - corners.get(0)[0]);
    //     }

    //     // if the corners are too close to be real (aka reject a probable false positive)
    //     // but only if this is the front limelight
    //     if (mConstants.kName == Constants.kFrontLimelightConstants.kName) {
    //         if (Math.abs(corners.get(0)[0] - corners.get(1)[0]) < 20) {
    //             //System.out.println("rejected target because corners were too close");
    //             return null;
    //         }
    //     }

    //     mTargets.clear();
    //     for (int i = 0; i < 2; ++i) {
    //         // Average of y and z; // XXX ? DRL
    //         double y_pixels = corners.get(i)[0];
    //         double z_pixels = corners.get(i)[1];

    //         // https://docs.limelightvision.io/en/latest/theory.html#from-pixels-to-angles

    //         // Normalize
    //         double nY = -((y_pixels - 160.0) / 160.0);
    //         double nZ = -((z_pixels - 120.0) / 120.0);

    //         // Scale by view plane size at 1 inch
    //         double y, z;
    //         if (mConstants.kName == "Front Limelight" && getPipeline() == Constants.kAuxPipeline)
    //         {
    //             y = Constants.kVPW_2X / 2 * nY;
    //             z = Constants.kVPH_2X / 2 * nZ;
    //         } else {
    //             y = Constants.kVPW_1X / 2 * nY;
    //             z = Constants.kVPH_1X / 2 * nZ;
    //         }

    //         TargetInfo target = new TargetInfo(y, z);
    //         target.setSkew(slope);
    //         mTargets.add(target);
    //     }

    //     return mTargets;
    // }

    // private synchronized List<TargetInfo> getRawBallTargetInfos() 
    // {
    //     // already normalized to -1~1
    //     double nY = mNetworkTable.getEntry("cx0").getDouble(0.0);
    //     double nZ = mNetworkTable.getEntry("cy0").getDouble(0.0);
    //     double width = mNetworkTable.getEntry("tlong").getDouble(0.0); // longest side of bounding box
    //     mSeesTarget = mNetworkTable.getEntry("tv").getDouble(0) == 1.0;

    //     double nW = width / 320.0 * 2; // a full screen of ball would be 2 because -1 to 1;
        
    //     // Scale by view plane size at 1 inch
    //     double y = Constants.kVPW_1X / 2 * nY; // W
    //     double w = Constants.kVPW_1X / 2 * nW; // W
    //     double z = Constants.kVPH_1X / 2 * nZ; // H
        
    //     TargetInfo target = new TargetInfo(y, z);
    //     target.setSize(w);
    //     mTargets.clear();
    //     mTargets.add(target);
    //     return mTargets;
    // }

    // /**
    //  * Returns raw top-left and top-right corners
    //  *
    //  * @return list of corners: index 0 - top left, index 1 - top right
    //  */
    // private List<double[]> getTopCorners() {
    //     double[] xyCorners = mNetworkTable.getEntry("tcornxy").getDoubleArray(mZeroArray);
    //     double[] xCorners = new double[xyCorners.length/2]; //mNetworkTable.getEntry("tcornx").getDoubleArray(mZeroArray);
    //     double[] yCorners = new double[xyCorners.length/2];//mNetworkTable.getEntry("tcorny").getDoubleArray(mZeroArray);
        

    //     int corner = 0;
    //     for (int i = 0; i < xyCorners.length; i+=2)
    //     {
    //         xCorners[corner] = xyCorners[i];
    //         yCorners[corner] = xyCorners[i+1];
    //         corner++;
    //     }

    //     mSeesTarget = mNetworkTable.getEntry("tv").getDouble(0) == 1.0;

    //     // something went wrong
    //     if (!mSeesTarget 
    //         || Arrays.equals(xCorners, mZeroArray) 
    //         || Arrays.equals(yCorners, mZeroArray) 
    //         || xCorners.length != yCorners.length)
    //         return null;

    //     // good so far
    //     if (mConstants.kName == "Front Limelight") { // hexagon target
    //         return extractWidestCornersFromBoundingBoxes(xCorners, yCorners);
    //     } else { // feeder target
    //         return extractTopCornersFromBoundingBoxes(xCorners, yCorners);
    //     }
    // }

    // private static final Comparator<Translation2d> xSort = Comparator.comparingDouble(Translation2d::x);
    // private static final Comparator<Translation2d> ySort = Comparator.comparingDouble(Translation2d::y);
    // private static final Comparator<Translation2d> ySortMinus = Comparator.comparingDouble(t -> -(t.getTranslation().y()));

    // /**
    //  * Returns raw top-left and top-right corners
    //  *
    //  * @return list of corners: index 0 - top left, index 1 - top right
    //  */
    // private List<double[]> extractWidestCornersFromBoundingBoxes(double[] xCorners, double[] yCorners) {
    //     List<Translation2d> corners = new ArrayList<>();
    //     for (int i = 0; i < xCorners.length; i++) {
    //         corners.add(new Translation2d(xCorners[i], yCorners[i]));
    //     }

    //     corners.sort(xSort);

    //     Translation2d leftCorner = corners.get(0);
    //     Translation2d rightCorner = corners.get(corners.size() - 1);

    //     return List.of(new double[]{leftCorner.x(), leftCorner.y()}, new double[]{rightCorner.x(), rightCorner.y()});
    // }

    // public static List<double[]> extractTopCornersFromBoundingBoxes(double[] xCorners, double[] yCorners) {
    //     List<Translation2d> corners = new ArrayList<>();
    //     for (int i = 0; i < xCorners.length; i++) {
    //         corners.add(new Translation2d(xCorners[i], yCorners[i]));
    //     }

    //     corners.sort(ySortMinus);

    //     List<Translation2d> top = corners.subList(0, 2);

    //     top.sort(xSort);

    //     Translation2d leftTop = top.get(0);
    //     Translation2d rightTop = top.get(1);

    //     return List.of(new double[]{leftTop.x(), leftTop.y()}, new double[]{rightTop.x(), rightTop.y()});
    // }

    public double getLatency() {
        return mPeriodicIO.latency;
    }
}