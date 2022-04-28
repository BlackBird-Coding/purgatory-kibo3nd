package purgatory.kibo;
import android.util.Log;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Mat;

public class Calculate {
    public double[] toQuaternion(double[] euler) {

        Log.i("Calulate toQuaternion [ Euler ]:", "X: "+euler[0]+" Y: "+euler[1]+" Z: "+euler[2]);

        double roll = Math.toRadians(euler[0]);
        double pitch = Math.toRadians(euler[1]);
        double yaw = Math.toRadians(euler[2]);

        double cosYaw = Math.cos(yaw / 2);
        double cosPitch = Math.cos(pitch / 2);
        double cosRoll = Math.cos(roll/2);

        double sinYaw = Math.sin(yaw / 2);
        double sinPitch = Math.sin(pitch / 2);
        double sinRoll = Math.sin(roll/2);

        double quaX = sinRoll * cosPitch * cosYaw + cosRoll * sinPitch * sinYaw;
        double quaY = cosRoll * sinPitch * cosYaw - sinRoll * cosPitch * sinYaw;
        double quaZ = cosRoll * cosPitch * sinYaw + sinRoll * sinPitch * cosYaw;
        double quaW = cosRoll * cosPitch * cosYaw - sinRoll * sinPitch * sinYaw;

        Log.i("Calculate toQuaternion [ Quaternion ]:", "X: "+ quaX +" Y: "+ quaY +" Z: "+ quaZ +" W: "+ quaW );

        return new double[]{quaX,quaY,quaZ,quaW};
    }

    public double[] combineQuaternion(double[] oldQuaternion,double[] newQuaternion){
        double quaX = oldQuaternion[3]*newQuaternion[0] + oldQuaternion[0]*newQuaternion[3] + oldQuaternion[1]*newQuaternion[2] - oldQuaternion[2]*newQuaternion[1];
        double quaY = oldQuaternion[3]*newQuaternion[1] + oldQuaternion[1]*newQuaternion[3] + oldQuaternion[2]*newQuaternion[0] - oldQuaternion[0]*newQuaternion[2];
        double quaZ = oldQuaternion[3]*newQuaternion[2] + oldQuaternion[2]*newQuaternion[3] + oldQuaternion[0]*newQuaternion[1] - oldQuaternion[1]*newQuaternion[0];
        double quaW = oldQuaternion[3]*newQuaternion[3] - oldQuaternion[0]*newQuaternion[0] - oldQuaternion[1]*newQuaternion[1] - oldQuaternion[2]*newQuaternion[2];

        Log.i("Calculate combineQuaternion [ Quaternion ]:", "X: "+ quaX +" Y: "+ quaY +" Z: "+ quaZ +" W: "+ quaW );

        return new double[]{quaX,quaY,quaZ,quaW};
    }

    public List<Integer> flatten(Mat src) {
        List<Integer> values = new ArrayList<>();
        for (int i = 0; i < src.rows(); i++) {
            for (int j = 0; j < src.cols(); j++) {
                values.add((int)src.get(i,j)[0]);
            }
        }
        return values;
    }

}