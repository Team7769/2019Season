package frc.robot.Configuration;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.lang.reflect.Field;

public final class Constants {
    public static String _path;

    //Spark Device ID's
    public static int kLFDriveId = 0;
    public static int kLMDriveId = 1;
    public static int kLRDriveId = 2;
    public static int kRFDriveId = 3;
    public static int kRMDriveId = 4;
    public static int kRRDriveId = 5;
    public static int kLArmId = 6;
    public static int kRArmId = 7;
    public static int kLElevatorId = 8;
    public static int kRElevatorId = 9;
    public static int kTCollectorId = 10;
    public static int kBCollectorId = 11;

    //Controller Usb Slots
    public static final int kDriverControllerSlot = 0;
    public static final int kOperatorControllerSlot = 0;

    //Elevator Setpoints
    public static final double kElevatorLowHatch = 0;
    public static final double kElevatorLowCargo = 1;
    public static final double kElevatorMidHatch = 2;
    public static final double kElevatorMidCargo = 3;
    public static final double kElevatorTopCargo = 4;

    public static final boolean kIsTestRobot = true;
    
    public static void loadConstantsFromFilePath(){
        String currentLine = "";
        String cvsSplitBy = ",";

        try (BufferedReader br = new BufferedReader(new FileReader(_path))) {

            while ((currentLine = br.readLine()) != null) {

                // use comma as separator
                System.out.println(currentLine);
                String[] setting = currentLine.split(cvsSplitBy);
                try {
                    Field field = Constants.class.getDeclaredField(setting[0]);
                    field.setAccessible(true);
                    switch (setting[1]){
                        case "int":
                        int valInt = Integer.parseInt(setting[2]);
                        field.setInt(Constants.class, valInt);
                        System.out.println("Set " + field.getName() + " to: " + valInt);
                        break;
                        case "boolean":
                        boolean valBool = Boolean.parseBoolean(setting[2]);
                        field.setBoolean(Constants.class, valBool);
                        System.out.println("Set " + field.getName() + " to: " + valBool);
                        break;
                        case "double":
                        double valDouble = Double.parseDouble(setting[2]);
                        field.setDouble(Constants.class, valDouble);
                        System.out.println("Set " + field.getName() + " to: " + valDouble);
                        break;
                        default:
                        break;
                    }
                } catch (NoSuchFieldException e){
                    e.printStackTrace();
                } catch (IllegalArgumentException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                } catch (IllegalAccessException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                } catch (Exception e) {
                    e.printStackTrace();
                }

            }

        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}