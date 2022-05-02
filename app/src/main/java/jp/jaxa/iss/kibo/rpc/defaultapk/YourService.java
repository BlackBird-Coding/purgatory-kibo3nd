package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.os.SystemClock;
import android.util.Log;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.DetectorParameters;
import org.opencv.aruco.Dictionary;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Core;

import java.util.ArrayList;
import java.util.List;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import gov.nasa.arc.astrobee.Kinematics;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import purgatory.kibo.Calculate;

public class YourService extends KiboRpcService {
    @Override
    protected void runPlan1(){
        Log.i("Bla_ckB","start");
        api.startMission();
        moveToWrapper(10.71000, -7.70000, 4.48000,0, 0.707, 0, 0.707);
        api.reportPoint1Arrival();

       /* moveToAr(10.71000, -7.70000, 4.48000,0, 0.707, 0, 0.707);
        api.laserControl(true);
        api.saveMatImage(api.getMatNavCam(),"target1.png");
        api.takeTarget1Snapshot();
        api.laserControl(false);*/

        moveToWrapper(11.3,-8.0000,4.4,0,0,-0.707, 0.707);
        moveToWrapper(11.3,-9.5000,4.4,0,0,-0.707, 0.707);
        moveToWrapper(11.27460, -9.92284, 5.29881,0, 0, -0.707, 0.707);


        moveToAr(11.27460, -9.92284, 5.29881,0, 0, -0.707, 0.707);
        api.laserControl(true);
        api.saveMatImage(api.getMatNavCam(),"target2.png");
        api.takeTarget2Snapshot();
        api.laserControl(false);

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

        double[] eu = new double[]{};

        do {
            Log.i("AR[count]:", String.valueOf(i));
            api.moveTo(point, quaternion, true);

            Kinematics kinematics = api.getRobotKinematics();

            api.flashlightControlFront((float)0.125);
            eu = DetectAR(api.getMatNavCam());
            api.flashlightControlFront(0);

            Point nPoint = kinematics.getPosition();
            Quaternion nQuaternion = kinematics.getOrientation();

            double[] oldQuaternion = new double[]{nQuaternion.getX(),nQuaternion.getY(),nQuaternion.getZ(),nQuaternion.getW()};
            double[] newQuaternion = new Calculate().EulertoQuaternion(new double[]{eu[0],eu[1],eu[2]});
            double[] result = new Calculate().combineQuaternion(oldQuaternion, newQuaternion);

            moveToWrapper(nPoint.getX(),nPoint.getY(),nPoint.getZ(),result[0],result[1],result[2],result[3]);

            ++i;
        } while (i < 5 && eu == null);

        return;
    }



    private double[] DetectAR(Mat matImage) {
        Log.i("AR[status]:", " start");
        long startTime = SystemClock.elapsedRealtime();

        Mat _ids = new Mat();
        List<Mat> rejectedCondinates = new ArrayList<>();
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        List<Mat> corners = new ArrayList<>();
        DetectorParameters parameters = DetectorParameters.create();
        parameters.set_adaptiveThreshWinSizeMin(5);
        parameters.set_adaptiveThreshWinSizeMax(10);
        parameters.set_minMarkerPerimeterRate(0.05d);
        parameters.set_maxMarkerPerimeterRate(5.0d);
        parameters.set_errorCorrectionRate(0.03d);

        double[][] camIntrinsics = api.getNavCamIntrinsics();
        Mat camMat = new Mat(3, 3, CvType.CV_32FC1);
        Mat distCoeffs = new Mat(1, 5, CvType.CV_32FC1);

        camMat.put(0,0,camIntrinsics[0]);
        distCoeffs.put(0,0,camIntrinsics[1]);

        Mat rvecs = new Mat();
        Mat tvecs = new Mat();

        Mat image = new Mat();

        Imgproc.undistort(matImage,image,camMat,distCoeffs);


        try {
            Aruco.detectMarkers(image, dictionary, corners, _ids, parameters);
            List<Integer> ids = new Calculate().flatten(_ids);
            //for(int i = 0;i<values.size();i++) {
            Aruco.estimatePoseSingleMarkers(corners, 0.05f, camMat, distCoeffs, rvecs, tvecs);
            //}

            List<double[]> arucos = new ArrayList<>();


            for(int i = 0; i < ids.size(); i++)
            {
                Mat rotationMatrix = new Mat();
                Mat r = new Mat();
                Mat q = new Mat();
                Calib3d.Rodrigues(rvecs.row(i), rotationMatrix);
                double[] eulerAngles = Calib3d.RQDecomp3x3(rotationMatrix, r, q);

                arucos.add(new double[]{ids.get(i),eulerAngles[0],-eulerAngles[1],-eulerAngles[2]});
            }


            double[] eu = new double[]{(arucos.get(0)[1] + arucos.get(1)[1] + arucos.get(2)[1] + arucos.get(3)[1]) / 4,
                    (arucos.get(0)[2] + arucos.get(1)[2] + arucos.get(2)[2] + arucos.get(3)[2]) / 4,
                    (arucos.get(0)[3] + arucos.get(1)[3] + arucos.get(2)[3] + arucos.get(3)[3]) / 4};


            Log.i("AR[status]:", " end");
            return eu;
        } catch (Exception e) {
           Log.i("AR[status]:", " Not detected");
        }
        return null;
    }
}

