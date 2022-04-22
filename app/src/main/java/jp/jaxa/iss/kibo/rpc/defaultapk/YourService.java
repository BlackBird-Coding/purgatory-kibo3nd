package jp.jaxa.iss.kibo.rpc.defaultapk;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import android.os.SystemClock;
import android.util.Log;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.DetectorParameters;
import org.opencv.core.Mat;
import org.opencv.aruco.Dictionary;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.CvType;
import org.opencv.calib3d.Calib3d;

import java.util.ArrayList;
import java.util.List;
import  java.util.Arrays;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    @Override
    protected void runPlan1(){
        Log.i("Bla_ckB","start");
        moveToWrapper(10.71000, -7.70000, 4.48000,0, 0.707, 0, 0.707);
        moveToAr(10.71000, -7.70000, 4.48000,0, 0.707, 0, 0.707);
        api.reportMissionCompletion();
    }

    @Override
    protected void runPlan2(){
        // write here your plan 2
    }

    @Override
    protected void runPlan3(){
        // write here your plan 3
    }

    private void moveToWrapper(double pos_x, double pos_y, double pos_z, double qua_x, double qua_y, double qua_z,
                               double qua_w) {

        final int LOOP_MAX = 5;
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float) qua_x, (float) qua_y, (float) qua_z, (float) qua_w);

        int i = 0;

        Result result;
        do {
            result = api.moveTo(point, quaternion, true);
            ++i;
        } while (!result.hasSucceeded() && i < LOOP_MAX);
    }

    private void moveToAr(double pos_x, double pos_y, double pos_z, double qua_x, double qua_y, double qua_z,
                               double qua_w) {

        final int LOOP_MAX = 5;
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float) qua_x, (float) qua_y, (float) qua_z, (float) qua_w);

        int i = 0;

        float[] eu = new float[]{};

        do {
            Log.i("AR[count]:", String.valueOf(i));
            api.moveTo(point, quaternion, true);
            eu = DetectAR(api.getMatNavCam());
            moveToEuler(10.71000, -7.70000, 4.48000,eu[0], eu[1], eu[2] );
            ++i;
        } while (i < 5 && eu == null);

        api.laserControl(true);
        api.laserControl(false);
        api.takeTarget1Snapshot();
        api.saveMatImage(api.getMatNavCam(),"target.png");
        return;
    }



    float[] DetectAR(Mat matImage) {
        Log.i("AR[status]:", " start");
        long startTime = SystemClock.elapsedRealtime();

        Mat ids = new Mat();
        List<Mat> rejectedCondinates = new ArrayList<>();
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        List<Mat> corners = new ArrayList<>();
        DetectorParameters parameter = DetectorParameters.create();
        double[][] camIntrinsics = api.getNavCamIntrinsics();
        Mat camMat = new Mat(3, 3, CvType.CV_32FC1);
        Mat distCoeffs = new Mat(1, 5, CvType.CV_32FC1);

        camMat.put(0,0,camIntrinsics[0]);
        distCoeffs.put(0,0,camIntrinsics[1]);

        Mat rvecs = new Mat();
        Mat tvecs = new Mat();

        try {
            Aruco.detectMarkers(matImage, dictionary, corners, ids, parameter);
            List<Integer> values = flatten(ids);
            //for(int i = 0;i<values.size();i++) {
                Aruco.estimatePoseSingleMarkers(corners, 0.05f, camMat, distCoeffs, rvecs, tvecs);
            //}

            List<float[]> arucos = new ArrayList<>();

            for (int i = 0; i < rvecs.rows(); ++i) {
                Mat rotationMatrix = new Mat();
                Mat r = new Mat();
                Mat q = new Mat();
                Calib3d.Rodrigues(rvecs.row(i), rotationMatrix);
                double[] eulerAngles = Calib3d.RQDecomp3x3(rotationMatrix, r, q);

                float pitch = (float) eulerAngles[0];
                float yaw = (float) eulerAngles[1];
                float roll = (float) eulerAngles[2];

                arucos.add(new float[]{values.get(i),yaw,pitch,roll});
            }

            float[] eu = new float[]{};
            float pitch ;



            for(int i=0;i<arucos.size();i++){
                float[] aruco = arucos.get(i);
                Log.i("AR[status]:",Arrays.toString(aruco));
                if(aruco[0] == 4){
                    eu = new float[]{aruco[1],aruco[2],aruco[3]};
                }
            }


            return eu;
        } catch (Exception e) {
           Log.i("AR[status]:", " Not detected");
        }
        Log.i("AR[status]:", " end");
        return null;
    }

    List<Integer> flatten(Mat src) {
        List<Integer> values = new ArrayList<>();
        for (int i = 0; i < src.rows(); i++) {
            for (int j = 0; j < src.cols(); j++) {
                values.add((int)src.get(i,j)[0]);
            }
        }
        return values;
    }

    public void moveToEuler(double x_org, double y_org, double z_org, double x_des, double y_des, double z_des) {
        double dx = x_des - x_org;
        double dy = y_des - y_org;
        double dz = z_des - z_org;
        double magnitude = Math.sqrt((dx * dx) + (dy * dy) + (dz * dz));
        double x_unit = dx / magnitude;
        double y_unit = dy / magnitude;
        double z_unit = dz / magnitude;

        double[][] matrix =
                {
                        {1, 0, 0},
                        {x_unit, y_unit, z_unit}
                };

        double x = matrix[0][1] * matrix[1][2] - matrix[1][1] * matrix[0][2];
        double y = matrix[0][2] * matrix[1][0] - matrix[1][2] * matrix[0][0];
        double z = matrix[0][0] * matrix[1][1] - matrix[1][0] * matrix[0][1];
        double i = matrix[1][0] - matrix[0][0];
        double j = matrix[1][1] - matrix[0][1];
        double k = matrix[1][2] - matrix[0][2];
        double q = Math.sqrt(x * x + y * y + z * z);
        double p = Math.sqrt(i * i + j * j + k * k);
        double theta = Math.acos((2 - p * p) / 2);

        double a = Math.sin(theta / 2) * x / q;
        double b = Math.sin(theta / 2) * y / q;
        double c = Math.sin(theta / 2) * z / q;
        double w = Math.cos(theta / 2);

        double pitch = -Math.atan((2 * (a * w + b * c)) / (w * w - a * a - b * b + c * c));
        double roll = -Math.asin(2 * (a * c - b * w));
        double yaw = Math.atan((2 * (c * w + a * b)) / (w * w + a * a - b * b - c * c));
        double sx = (0.103 * Math.cos(roll + 0.279) / Math.cos(1.57080 + yaw));
        double sy = (0.103 * Math.sin(roll + 0.279) / Math.cos(pitch));

        moveToWrapper((float) x_org - (float) sx, (float) y_org, (float) z_org + (float) sy, (float) a, (float) b, (float) c, (float) w);
    }

}

