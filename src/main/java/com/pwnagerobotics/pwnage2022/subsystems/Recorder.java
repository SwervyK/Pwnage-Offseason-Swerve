package com.pwnagerobotics.pwnage2022.subsystems;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Scanner;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Recorder {
    
    private File mAutoFile;
    private File[] mAutoFiles;
    private File mAutoDir = new File("home/lvuser/autoRecordings");
    private static SendableChooser<File> mAutoChooser = new SendableChooser<File>();
    private ArrayList<String> mAutoPath = new ArrayList<String>();
    
    public Recorder(){
        try {
            String[] fileNames = mAutoDir.list();
            mAutoFiles = new File[fileNames.length];
            if (fileNames.length > 0) {
                for (int i = 0; i < fileNames.length; i++) {
                    mAutoFiles[i] = new File(mAutoDir.getPath(), fileNames[i]);
                    mAutoChooser.addOption(fileNames[i], mAutoFiles[i]);
                }
            }
            SmartDashboard.putData("Auto Chooser", mAutoChooser);
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("Error in creating file");
        }
        
    }
    
    public void newAuto(String filename) {
        mAutoFile = new File(mAutoDir.getPath(), filename);
        try {
            if (!mAutoFile.exists()) 
                mAutoFile.createNewFile();
        } catch (IOException e) {
            e.printStackTrace();
        }
        System.out.println("new auto started");
    }
    
    public void recordInputs(double throttle, double strafe, double rotationX, double rotationY, double timestamp){
        mAutoPath.add(timestamp+":[rx="+rotationX+"]"+"[ry="+rotationY+"]"+"[t="+throttle+"]"+"[s="+strafe+"] ");
    }
    
    public void stopRecording() {
        String[] data = new String[mAutoPath.size()];
        data = mAutoPath.toArray(data);
        writeData(data, mAutoFile);
    }
    
    public class SwerveState {
        public double kThrottle = 0, kStrafe = 0, kRotationX = 0, kRotationY = 0, kTimestamp = 0;
    }
    
    private SwerveState[] swerveStates;
    
    public SwerveState getSwerveStateAtTimestamp(double timestamp){
        if (swerveStates == null) loadFromFile();
        int closestIndex = 0;
        double difference = 999999999; //INFINITY
        for(int i = 0; i < swerveStates.length; i++){
            if(timestamp - swerveStates[i].kTimestamp < difference){
                difference = timestamp - swerveStates[i].kTimestamp;
                if(difference < 0){
                    break;
                }
                closestIndex = i;
            }
        }
        return swerveStates[closestIndex];
    }
    
    private void loadFromFile(){
        String[] data = readData(mAutoChooser.getSelected());
        swerveStates = new SwerveState[data.length];
        for(int i = 0; i < data.length; i++){
            double t = Double.parseDouble(data[i].substring(data[i].indexOf("t=")+2,data[i].indexOf("][s")));
            double s = Double.parseDouble(data[i].substring(data[i].indexOf("s=")+2,data[i].indexOf("] ")));
            double rx = Double.parseDouble(data[i].substring(data[i].indexOf("rx=")+3,data[i].indexOf("][ry")));
            double ry = Double.parseDouble(data[i].substring(data[i].indexOf("ry=")+3,data[i].indexOf("][t")));
            double timeStamp = Double.parseDouble(data[i].substring(0, data[i].indexOf(":")));
            swerveStates[i] = new SwerveState();
            swerveStates[i].kThrottle = t;
            swerveStates[i].kStrafe = s;
            swerveStates[i].kRotationX = rx;
            swerveStates[i].kRotationY = ry;
            swerveStates[i].kTimestamp = timeStamp;
        }
    }
    
    
    
    private String[] readData(File file) {
        String[] result = new String[0];
        try {
            result = new String[(int)Files.lines(file.toPath()).count()];
            Scanner scanner = new Scanner(file);
            int index = 0;
            while (scanner.hasNextLine()) {
                result[index] = scanner.nextLine();
                index++;
            }
            scanner.close();
        }
        catch (Exception e) {
            e.printStackTrace();
        }
        return result;
    }
    
    private void writeData(String data, File file) {
        try  {
            FileWriter writer = new FileWriter(file);
            writer.write(data);
            writer.close();
        }
        catch (Exception e) {
            e.printStackTrace();
        }
    }
    
    private void writeData(String[] dataArr, File file) {
        String data = "";
        for (int i = 0; i < dataArr.length; i++) {
            data += (dataArr[i] + "\n");
        }
        writeData(data, file);
    }
}