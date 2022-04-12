package com.pwnagerobotics.pwnage2022.subsystems;

import javax.swing.Box;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSlider;

import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.BasicStroke;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.Vector2d;

public class SwerveVisuial {
    
    private static JFrame frame = new JFrame();
    private static JSlider slider = new JSlider(0, 360);
    private static JSlider rotationSlider = new JSlider(-10, 10);
    private static JLabel text = new JLabel();
    public static void main(String[] args) {
        SwerveVisuial s = new SwerveVisuial();
        s.createAndShowGUI();
    }
    
    private void createAndShowGUI() {
        slider.setPaintTicks(false);
        slider.setValue(0);
        rotationSlider.setPaintTicks(false);
        rotationSlider.setValue(0);
        Box bottom = Box.createVerticalBox();
        bottom.add(slider);
        bottom.add(rotationSlider);
        bottom.add(text);
        frame.setLayout(new BorderLayout());
        frame.add(bottom, BorderLayout.SOUTH);
        frame.add(new Drive());
        frame.setSize(500, 400);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setVisible(true);
    }
    
    public static class Drive extends JPanel {
        
        private static double rotationEncoder = 0.0;
        private static double wantedPosition = 0.0;
        private static PIDController pidController = new PIDController(0.9, 0.015, 0);
        
        public Drive() {
            super.repaint();
            super.setVisible(true);
            pidController.enableContinuousInput(-90, 90);
            pidController.setTolerance(2);
        }
        
        @Override
        public void paintComponent(Graphics g){
            super.paintComponent(g);
            Graphics2D g2 = (Graphics2D) g;
            g2.setStroke(new BasicStroke(2));
            g2.setColor(Color.BLACK);
            
            // Controller
            double angle1 = slider.getValue() - 90.0;
            int startX1 = getWidth()/2;
            int startY1 = getHeight()/2;
            int length1 = 100;
            
            double endX1 = startX1 + Math.cos(Math.toRadians(angle1)) * length1;
            double endY1 = startY1 + Math.sin(Math.toRadians(angle1)) * length1;
            g2.setColor(Color.GREEN);
            g2.drawLine(startX1, startY1, (int)endX1, (int)endY1);
            
            // Current
            double angle2 = rotationEncoder - 90.0;
            
            int startX2 = getWidth()/2;
            int startY2 = getHeight()/2;
            int length2 = 100;
            
            double endX2 = startX2 + Math.cos(Math.toRadians(angle2)) * length2;
            double endY2 = startY2 + Math.sin(Math.toRadians(angle2)) * length2;
            g2.setColor(Color.RED);
            g2.drawLine(startX2, startY2, (int)endX2, (int)endY2);
            
            // Want
            double angle3 = wantedPosition - 90.0;
            
            int startX3 = getWidth()/2;
            int startY3 = getHeight()/2;
            int length3 = 100;
            
            double endX3 = startX3 + Math.cos(Math.toRadians(angle3)) * length3;
            double endY3 = startY3 + Math.sin(Math.toRadians(angle3)) * length3;
            g2.setColor(Color.BLUE);
            g2.drawLine(startX3, startY3, (int)endX3, (int)endY3);
            
            setVectorSwerveDrive(0.7, rotationSlider.getValue()/10.0, slider.getValue());
            repaint();
        }
        
        private void setModule(double rotation)
        {
            // Postion
            double controllerPosition = slider.getValue();
            double encoderPosition = rotationEncoder;
            if (encoderPosition < 0) encoderPosition += 360;
            // wantedPosition = slider.getValue();
            // if (wantedPosition > 360) wantedPosition -= 360;
            
            // // Distance
            // double distance = getDistance(encoderPosition, wantedPosition);
            // if (Math.abs(distance) > 90 && Math.abs(distance) < 270) {
            //     wantedPosition -= 180;
            //     if (wantedPosition < 0) wantedPosition += 360;
            //     distance = getDistance(encoderPosition, wantedPosition);
            // }

            /////////////////////////////////////////////////////////////////////////////////////

            wantedPosition = rotation;
            // Distance
            double distance = getDistance(encoderPosition, rotation);
            if (Math.abs(distance) > 90 && Math.abs(distance) < 270) {
                rotation -= 180;
                if (rotation < 0) rotation += 360;
                distance = getDistance(encoderPosition, rotation);
            }

            double motorValue = 0;
            // PID
            motorValue = pidController.calculate(0, -distance);
            motorValue /= 64;
            if (pidController.atSetpoint()) motorValue = 0;
            
            ///////////////////////////////////////////////////////////////////////////////////

            // Testing
            text.setText("Controller: " + (int)controllerPosition +" | Encoder: " + (int)rotationEncoder + " | Wanted: " + (int)wantedPosition + " | Distance: " + (int)distance + " | Rotation: " + (int)rotation);
            if (Math.abs(motorValue) != 0) rotationEncoder += motorValue/10.0;
            if (rotationEncoder > 360) rotationEncoder -= 360;
            if (rotationEncoder < 0) rotationEncoder += 360;
        }
        
        public void setVectorSwerveDrive(double forwardSpeed, double rotationSpeed, double robotAngle){
            Vector2d vector = addMovementComponents(forwardSpeed, robotAngle, rotationSpeed, 135);
            double maxMagnitude = Math.max(1.0, vector.magnitude());
            if(maxMagnitude > 1){
                //Normalize vectors, preserving proportions while reducing all below 1
                scaleVector2d(vector, 1. / maxMagnitude);
            }
            setModule(getVectorAngle(vector));
        }
        
        private Vector2d scaleVector2d(Vector2d v, double scalar){
            return new Vector2d(v.x * scalar, v.y * scalar);
        }
        
        private double getVectorAngle(Vector2d v){
            // double angle = Math.toDegrees(Math.atan2(v.x, v.y));
            // angle = (angle >= 0) ? angle : angle + 360;
            double angle = Math.atan(v.y / v.x) * 180 / 3.14;
            if (v.x < 0) angle += 180;
            else if (v.y < 0) angle += 360;
            return angle;
        }
        
        private Vector2d addMovementComponents(double forwardMagnitude, double rotation1, double rotationalMagnitude, double rotation2){
            Vector2d forwardVector = new Vector2d(forwardMagnitude * Math.cos(rotation1 * 3.14 / 180), forwardMagnitude * Math.sin(rotation1 * 3.14 / 180));
            Vector2d rotationVector = new Vector2d(rotationalMagnitude * Math.cos(rotation2 * 3.14 / 180), rotationalMagnitude * Math.sin(rotation2 * 3.14 / 180));
            return new Vector2d(forwardVector.x + rotationVector.x, forwardVector.y + rotationVector.y);
        }
        
        private double getDistance(double encoder, double controller) {
            double result = encoder - controller;
            if (Math.abs(result) > 180) {
                result += 360 * -Math.signum(result);
            }
            return result;
        }
    }
    
}
