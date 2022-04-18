package com.pwnagerobotics.pwnage2022.subsystems;

import java.io.File;
import java.nio.file.Files;
import java.util.Scanner;

import com.team254.lib.util.ReflectingCSVWriter;

public class Recorder {

    private ReflectingCSVWriter<String> CSVWriter;
    private Scanner scanner;

    public Recorder(String filename){
        try {
            CSVWriter = new ReflectingCSVWriter<>(filename, String.class);
            scanner = new Scanner(filename);
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("Error in creating file");
        }

    }

    public class SwerveState {
        public double kThrottle,kStrafe,kRotationX,kRotationY,kTimestamp;
    }

    private SwerveState[] swerveStates;

    private void writeToFile(String s){
        CSVWriter.add(s);
    }

    private String[] readFromFile(String fileName){
        try{
            String[] result = new String[(int)Files.lines(new File(fileName).toPath()).count()];
            for(int i = 0; i < result.length; i++){
                result[i] = scanner.nextLine();
            }
            return result;
        } catch (Exception e){
            return null;

        }
    }

    public void stopRecording() {
        CSVWriter.flush();
    }

    public SwerveState getSwerveStateAtTimestamp(double timestamp){
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

    public void loadFromFile(String fileName){
        String[] data = readFromFile(fileName);
        swerveStates = new SwerveState[data.length];
        for(int i = 0; i < data.length; i++){
            double t = Double.parseDouble(data[i].substring(data[i].indexOf("t=")+1,data[i].indexOf("][s")));
            double s = Double.parseDouble(data[i].substring(data[i].indexOf("s=")+1,data[i].indexOf("] ")));
            double rx = Double.parseDouble(data[i].substring(data[i].indexOf("rx=")+1,data[i].indexOf("][ry")));
            double ry = Double.parseDouble(data[i].substring(data[i].indexOf("ry=")+1,data[i].indexOf("][t")));
            double timeStamp = Double.parseDouble(data[i].substring(0, data[i].indexOf(":")));
            swerveStates[i].kThrottle = t;
            swerveStates[i].kStrafe = s;
            swerveStates[i].kRotationX = rx;
            swerveStates[i].kRotationY = ry;
            swerveStates[i].kTimestamp = timeStamp;
        }
    }

    public void recordInputs(double throttle, double strafe, double rotationX, double rotationY, double timestamp){
        writeToFile(timestamp+":[rx="+rotationX+"]"+"[ry="+rotationY+"]"+"[t="+throttle+"]"+"[s="+strafe+"] ");
    }
}