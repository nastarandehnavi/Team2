package app;

import lejos.hardware.Button;
import lejos.hardware.motor.*;
import lejos.hardware.*;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

public class walkingDetactObstaclestop {

    public static void main(String[] args) {
       
        EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S3);
        SampleProvider colorProvider = colorSensor.getRedMode();
        
        EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S2);
        SampleProvider distanceProvider = ultrasonicSensor.getDistanceMode();
        
        final float EDGE_LOWER_THRESHOLD = 15.0f;
        final float EDGE_UPPER_THRESHOLD = 21.0f;
        final float OBSTACLE_DISTANCE_THRESHOLD = 20.0f; // Adjust as needed

        while (!Button.ENTER.isDown()) {
            // Edge detection
            float[] sample = new float[colorProvider.sampleSize()];
            colorProvider.fetchSample(sample, 0);
            float intensity = sample[0] * 100;

            // Obstacle detection
            float[] distanceSample = new float[distanceProvider.sampleSize()];
            distanceProvider.fetchSample(distanceSample, 0);
            float distance = distanceSample[0] * 100; // Convert to centimeters

            if (distance < OBSTACLE_DISTANCE_THRESHOLD) {
                // Stop the robot if obstacle detected
                Motor.C.stop();
                Motor.D.stop();
            } else if (intensity >= EDGE_LOWER_THRESHOLD && intensity <= EDGE_UPPER_THRESHOLD) {
                // Robot is on the edge
                Motor.C.setSpeed(250);
                Motor.D.setSpeed(250);
                Motor.C.forward();
                Motor.D.forward();
            } else if (intensity < EDGE_LOWER_THRESHOLD) {
                // Robot is off the edge to the left
                Motor.C.setSpeed(200);
                Motor.D.setSpeed(50);
                Motor.C.forward();
                Motor.D.forward();
            } else {
                // Robot is off the edge to the right
                Motor.C.setSpeed(50);
                Motor.D.setSpeed(200);
                Motor.C.forward();
                Motor.D.forward();
            }
        }

        colorSensor.close();
        ultrasonicSensor.close();
    }
}
