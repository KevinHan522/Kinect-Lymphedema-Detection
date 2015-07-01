///How to use

using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Globalization;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Forms;
using System.Drawing;
using Microsoft.Kinect;
using System.Drawing.Imaging;

namespace VolumeScanner
{

    class Program
    {       
        [STAThread]
        static void Main(string[] args)
        {           
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);

            Form form = new Display();
            form.FormBorderStyle = FormBorderStyle.FixedDialog;
            form.MaximizeBox = false;
            form.MinimizeBox = false;
            form.StartPosition = FormStartPosition.CenterScreen;

            //thread for running kinect functions
            Thread KinectThread = new Thread(WorkerThreadProc);
            KinectThread.Start();
            Application.Run(form);

        }

        static void WorkerThreadProc()
        {
            //ensures form exists before any sort of running
            
            while (Display.ActiveForm == null)
            {
            }
            //creates a new volume scanner class
            //realistically, it would be a better idea to make this a library
            
            if (Display.ActiveForm != null)
            {
                Form mainForm = Display.ActiveForm;
                KinectVolumeScanner scanner = new KinectVolumeScanner(mainForm);
                scanner.Run();
            }
            
        }
    }

    class KinectVolumeScanner
    {
        //sensor object
        private KinectSensor sensor = null;

        //reader that reads in the various frames (depth, color, infrared, etc.)
        private MultiSourceFrameReader reader = null;

        //reader that takes in body frames
        private BodyFrameReader bodyReader = null;

        //flag that controls whether median/mode smoothing is enabled
        private bool smoothEnabled = false;

        //flag that controls whether color is enabled
        private bool colorEnabled = false;

        //boolean that tells when the program is processing depth data and thus not to update it
        private bool isProcessing = false;

        //variables for the dimensions of the frames to be read in
        private int depthWidth = 0;
        private int depthHeight = 0;
        private int colorWidth = 0;
        private int colorHeight = 0;

        //stores depth frame data
        private ushort[] depthImagePixels;

        //table used for one of the ways of converting from depth space to camera space
        private Microsoft.Kinect.PointF[] cameraSpaceFactors;

        //stores color frame data
        private byte[] colorImagePixels;

        //stores color frame data aligned to the actual depth frame's resolution
        private int[] alignedColorImagePixels;

        //private ushort[] infImagePixels;

        //the average of several depth frame data
        private ushort[] avgDepthImagePixels = null;

        private ushort[] workingAvgDepthImagePixels = null;

        //temporary variable used in calculation of avgDepthImagePixels
        private double[] calcAvgDepthImg = null;

        //contains all depth frames that are being averaged
        private Queue<ushort[]> depthFrames = null;

        //number of frames to average for smoothing
        private int framesToIntegrate;

        //where to start the integration method from, deprecated now that the spine z value is used
        private int depthStart = 0;

        //holds the bodies being detected
        private Body[] bodies = null;

        //struct definition for a skeleton
        //simply holds several joint positions
        private struct Skeleton
        {
            public CameraSpacePoint rightElbow;
            public CameraSpacePoint rightShoulder;
            public CameraSpacePoint leftElbow;
            public CameraSpacePoint leftShoulder;
            public CameraSpacePoint rightHand;
            public CameraSpacePoint leftHand;
            public CameraSpacePoint spine;
        }
        
        //joint data for the primary body being tracked
        private Skeleton bodyJoints;

        //frame variables
        private ManualResetEvent depthReadyEvent = null;
        private ManualResetEvent colorReadyEvent = null;
        private ManualResetEvent workerThreadStopEvent = null;
        private Thread FrameThread = null;

        private Control mainForm;

        //graphics variables
        private Graphics graphics = null;
        private Form control = null;
        private TrackBar weightSlider = null;
        private Label weightLabel = null;
        private TrackBar threshSlider = null;
        private Label threshLabel = null;
        private TrackBar frameSlider = null;
        private Label frameLabel = null;
        //private TrackBar depthStartSlider = null;
        //private Label depthStartLabel = null;
        private Button smoothButton = null;
        private Button colorButton = null;
        private Button dumpButton = null;

        //coordinate mapper class, allows conversion between different spaces
        private CoordinateMapper mapper = null;

        //coordinate mapper arrays
        private ColorSpacePoint[] colorCoordinates = null;
        private CameraSpacePoint[] cameraSpacePoints = null;

        //how much to weigh depth differences in segmentation, depthWeight + colorWeight = 1
        private double depthWeight;

        //how much to weight color differences in segmentation
        private double colorWeight;

        //the threshold to use for adding pixels to a cluster, fairly arbitrary
        private int absoluteThreshold;

        private const double OPTIMUM_DEPTH = .87;

        //point struct, basically just a point in 3D
        private struct Point3D:IEquatable<Point3D>
        {
            public int x;
            public int y;
            public double z;
            public double camX;
            public double camY;
            public Point3D(int a, int b, double c)
            {
                x = a;
                y = b;
                z = c;
                camX = 0;
                camY = 0;
            }

            public Point3D(int a, int b, double c, Microsoft.Kinect.PointF pt)
            {
                x = a;
                y = b;
                z = c;
                camX = 1000 * pt.X * z;
                camY = 1000 * pt.Y * z;
            }

            public Point3D(int a, int b, double c, CameraSpacePoint p)
            {
                x = a;
                y = b;
                z = c;
                camX = 1000 * p.X;
                camY = 1000 * p.Y;
            }

            //get real distance between two points, taking into account x, y, and z values
            public double FindDistance(Point3D other)
            {
                return Math.Sqrt(Math.Pow(this.camX - other.camX, 2) + Math.Pow(this.camY - other.camY, 2) + Math.Pow(this.z - other.z, 2));
            }

            public bool Equals(Point3D oth)
            {
                if (x == oth.x && y == oth.y) 
                {
                    return true;
                }
                return false;
            }

            
        }

        public KinectVolumeScanner(Form form)
        {
            //gets the Kinect sensor
            sensor = KinectSensor.GetDefault();
            
            //opens the body reader
            //bodyReader = sensor.BodyFrameSource.OpenReader();

            //opens the sensor
            sensor.Open();

            //open the multiframe reader to take in depth and color
            reader = sensor.OpenMultiSourceFrameReader(FrameSourceTypes.Depth | FrameSourceTypes.Color | FrameSourceTypes.Body);
            //reader = sensor.OpenMultiSourceFrameReader(FrameSourceTypes.Depth | FrameSourceTypes.Color | FrameSourceTypes.Infrared);

            //get frame information (such as dimensions) and initialize variables
            FrameDescription depthFrameDescription = sensor.DepthFrameSource.FrameDescription;
            depthWidth = depthFrameDescription.Width;
            depthHeight = depthFrameDescription.Height;
            depthImagePixels = new ushort[depthWidth * depthHeight];

            FrameDescription colorFrameDescription = sensor.ColorFrameSource.FrameDescription;
            colorWidth = colorFrameDescription.Width;
            colorHeight = colorFrameDescription.Height;
            colorImagePixels = new byte[colorWidth * colorHeight * sizeof(int)];

            cameraSpacePoints = new CameraSpacePoint[depthWidth * depthHeight];

            alignedColorImagePixels = new int[depthWidth * depthHeight];
            //infImagePixels = new ushort[depthImagePixels.Length];
            avgDepthImagePixels = new ushort[depthWidth * depthHeight];
            workingAvgDepthImagePixels = new ushort[depthWidth * depthHeight];
            calcAvgDepthImg = new double[depthWidth * depthHeight];
            depthFrames = new Queue<ushort[]>();

            //get coordinate mapper
            mapper = sensor.CoordinateMapper;

            bodyJoints = new Skeleton();
            
            depthReadyEvent = new ManualResetEvent(false);
            colorReadyEvent = new ManualResetEvent(false);
            workerThreadStopEvent = new ManualResetEvent(false);
            FrameThread = new Thread(WorkerThreadProc);
            FrameThread.Start();

            //initialize all graphics-related controls
            mainForm = form;

            graphics = mainForm.CreateGraphics();
            control = (Form) mainForm;
            control.MouseClick += Control_MouseClick;
            weightSlider = (TrackBar)(control.Controls["weightSlider"]);
            weightSlider.Scroll += Weight_Scroll;
            weightLabel = (Label)(control.Controls["wgtValueLbl"]);
            threshSlider = (TrackBar)(control.Controls["threshSlider"]);
            threshSlider.Scroll += Thresh_Scroll;
            threshLabel = (Label)(control.Controls["absValueLbl"]);
            frameSlider = (TrackBar)(control.Controls["frameSlider"]);
            frameSlider.Scroll += Frame_Scroll;
            frameLabel = (Label)(control.Controls["intValueLbl"]);
 /*
            depthStartSlider = (TrackBar)(control.Controls["depStSlider"]);
            depthStartLabel = (Label)(control.Controls["depStLbl"]);
            depthStartSlider.Scroll += DepthStart_Scroll;
  */
            smoothButton = (Button)(control.Controls["smoothButton"]);
            smoothButton.Click += Smooth_Click;
            colorButton = (Button)(control.Controls["colorButton"]);
            colorButton.Click += Color_Click;
            dumpButton = (Button)(control.Controls["dumpButton"]);
            dumpButton.Click += Dump_Click;

            //depth from which to start integrating, deprecated
            depthStart = 500;

            //initialize to higher segmentation on depth
            depthWeight = .60;
            colorWeight = 1 - depthWeight;

            //initialize to a relatively high threshold
            absoluteThreshold = 150;

            //initialize to 1 (i.e. no smoothing);
            framesToIntegrate = 1;
        }

        public void Run()
        {         
            //attach frame arrival event to the Reader_MultiSourceFrameArrived method
            reader.MultiSourceFrameArrived += Reader_MultiSourceFrameArrived;

            //timer counters for how often to check for a volume cluster
            int timer = 50;
            int timer2 = 5;

            //depth clip range, this is the default when no bodies are detected
            double depthClipMin = 0;
            double depthClipMax = 6000;
            Thread frameProc = new Thread(AcquireFrames);
            frameProc.Start();
            while (true)
            {              
                //bitmap images to display on screen
                Bitmap img = new Bitmap(depthWidth, depthHeight);   
                //Bitmap img2 = new Bitmap(depthWidth, depthHeight);
                BitmapData imgData = img.LockBits(new Rectangle(0, 0, depthWidth, depthHeight), System.Drawing.Imaging.ImageLockMode.ReadWrite, img.PixelFormat);
                //img2.LockBits(new Rectangle(0, 0, depthWidth, depthHeight), System.Drawing.Imaging.ImageLockMode.ReadWrite, img2.PixelFormat);


                
                lock (this)
                {
                
                    //cameraSpaceFactors = mapper.GetDepthFrameToCameraSpaceTable();
                    //map color data to the depth data
                    MapColorToDepth();
                    bool hasDrawn = false;

                    int[] depthColors = new int[depthWidth * depthHeight];
                    IntPtr ptr = imgData.Scan0;
                    //if color is enabled, set the image to the color data
                    if (colorEnabled)
                    {
                        depthColors = alignedColorImagePixels;
                        //System.Runtime.InteropServices.Marshal.Copy(alignedColorImagePixels, 0, ptr, depthWidth * depthHeight);
                        /*
                        for (int i = 0; i < depthWidth * depthHeight; i++)
                        {
                            img.SetPixel(i % depthWidth, i / depthWidth, Color.FromArgb(alignedColorImagePixels[i]));
                        }*/
                        /*
                        for (int i = 0; i < depthWidth * depthHeight; i++)
                        {
                            img.SetPixel(i % depthWidth, i / depthWidth, Color.FromArgb((int)((Math.Min(infImagePixels[i] / 30000.0, 1) * 255)), Color.Red));
                        }
                        */
                    }


                    //otherwise, color a depth representation, darker is farther, lighter is closer
                    else
                    {
                        Parallel.For(0, depthHeight, i =>
                            {
                                for (int j = 0; j < depthWidth; j++)
                                {
                                    depthColors[i * depthWidth + j] = (Color.FromArgb((int)((Math.Min(Math.Max((avgDepthImagePixels[i * depthWidth + j] - depthClipMin), 0) / (depthClipMax - depthClipMin), 1) * 255)), Color.Blue)).ToArgb();
                                }
                            }
                        );

                        //System.Runtime.InteropServices.Marshal.Copy(depthIntensities, 0, ptr, depthWidth * depthHeight);

                        /*
                        for (int i = 0; i < depthWidth * depthHeight; i++)
                        {
                            img.SetPixel(i % depthWidth, i / depthWidth, Color.FromArgb((int)((Math.Min(Math.Max((avgDepthImagePixels[i] - depthClipMin) , 0) / (depthClipMax - depthClipMin), 1) * 255)), Color.Blue));
                        }*/
                    }
                    //if a body is tracked, then do volume cluster finding and calculation
                    if (bodies != null)
                    {
                        

                        //map them to depth space for GUI purposes
                        DepthSpacePoint spine = mapper.MapCameraPointToDepthSpace(bodyJoints.spine);
                        DepthSpacePoint elbowLeft = mapper.MapCameraPointToDepthSpace(bodyJoints.leftElbow);
                        DepthSpacePoint elbowRight = mapper.MapCameraPointToDepthSpace(bodyJoints.rightElbow);
                        DepthSpacePoint handLeft = mapper.MapCameraPointToDepthSpace(bodyJoints.leftHand);
                        DepthSpacePoint handRight = mapper.MapCameraPointToDepthSpace(bodyJoints.rightHand);
                        DepthSpacePoint shoulderLeft = mapper.MapCameraPointToDepthSpace(bodyJoints.leftShoulder);
                        DepthSpacePoint shoulderRight = mapper.MapCameraPointToDepthSpace(bodyJoints.rightShoulder);

                        /*
                        try
                        {
                            for (int j = (int)elbowRight.Y - 5; j < elbowRight.Y + 5; j++)
                            {
                                for (int i = (int)elbowRight.X - 5; i < elbowRight.X + 5; i++)
                                {
                                    img2.SetPixel(i, j, Color.Red);
                                }
                            }

                            for (int j = (int)elbowLeft.Y - 5; j < elbowLeft.Y + 5; j++)
                            {
                                for (int i = (int)elbowLeft.X - 5; i < elbowLeft.X + 5; i++)
                                {
                                    img2.SetPixel(i, j, Color.Red);
                                }
                            }



                            for (int j = (int)shoulderRight.Y - 5; j < shoulderRight.Y + 5; j++)
                            {
                                for (int i = (int)shoulderRight.X - 5; i < shoulderRight.X + 5; i++)
                                {
                                    img2.SetPixel(i, j, Color.Red);
                                }
                            }


                            for (int j = (int)shoulderLeft.Y - 5; j < shoulderLeft.Y + 5; j++)
                            {
                                for (int i = (int)shoulderLeft.X - 5; i < shoulderLeft.X + 5; i++)
                                {
                                    img2.SetPixel(i, j, Color.Red);
                                }
                            }

                            for (int j = (int)handRight.Y - 5; j < handRight.Y + 5; j++)
                            {
                                for (int i = (int)handRight.X - 5; i < handRight.X + 5; i++)
                                {
                                    img2.SetPixel(i, j, Color.Red);
                                }
                            }


                            for (int j = (int)handLeft.Y - 5; j < handLeft.Y + 5; j++)
                            {
                                for (int i = (int)handLeft.X - 5; i < handLeft.X + 5; i++)
                                {
                                    img2.SetPixel(i, j, Color.Red);
                                }
                            }
                        }
                        catch (Exception e)
                        {

                        }
                         * */
                        //only do so if timer has been reset
                        if (timer <= 0)
                        {
                            //threshold for how far spine can be from OPTIMUM_DEPTH
                            double spineDiffThresh = .01;
                            //threshold for how different shoulder depths can be
                            double shoulderDiffThresh = .06;
                            //threshold for how different elbow depths can be
                            double elbowDiffThresh = .04;
                            //threshold for how different hand depths can be
                            double handDiffThresh = .05;

                            //alter the depth clip range to focus around the body
                            if (bodyJoints.spine.Z != 0)
                            {
                                depthClipMin = 1000 * bodyJoints.spine.Z - 300;
                                depthClipMax = 1000 * bodyJoints.spine.Z + 300;
                            }

                            //find the depth differences between each body joint
                            double leftElbowDiff = bodyJoints.spine.Z - bodyJoints.leftElbow.Z;
                            double rightElbowDiff = bodyJoints.spine.Z - bodyJoints.rightElbow.Z;
                            double leftShoulderDiff = bodyJoints.spine.Z - bodyJoints.leftShoulder.Z;
                            double rightShoulderDiff = bodyJoints.spine.Z - bodyJoints.rightShoulder.Z;
                            double leftHandDiff = bodyJoints.spine.Z - bodyJoints.leftHand.Z;
                            double rightHandDiff = bodyJoints.spine.Z - bodyJoints.rightHand.Z;
                            double spineDiff = bodyJoints.spine.Z - OPTIMUM_DEPTH;
                            
                            
                            //draw a red box over any joints that are too far back behind the other joint
                            //draw box over spine indicating too close/too far
                            //red too far back, blue too far forward
                            try
                            {
                                if (Math.Abs(spineDiff) > spineDiffThresh)
                                {
                                    Color c;

                                    if (spineDiff < 0)
                                    {
                                        c = Color.DarkBlue;
                                    }
                                    else
                                    {
                                        c = Color.DarkRed;
                                    }


                                    for (int j = (int)spine.Y - 10; j < spine.Y + 10; j++)
                                    {
                                        for (int i = (int)spine.X - 10; i < spine.X + 10; i++)
                                        {
                                            depthColors[j * depthWidth + i] = c.ToArgb();
                                        }
                                    }
                                }

                                if (Math.Abs(leftElbowDiff) > elbowDiffThresh)
                                {
                                    Color c;
                                    if (leftElbowDiff > 0)
                                    {
                                        c = Color.Blue;
                                    }
                                    else
                                    {
                                        c = Color.Red;
                                    }
                                    for (int j = (int)elbowLeft.Y - 10; j < elbowLeft.Y + 10; j++)
                                    {
                                        for (int i = (int)elbowLeft.X - 10; i < elbowLeft.X + 10; i++)
                                        {
                                            depthColors[j * depthWidth + i] = c.ToArgb();
                                        }
                                    }
                                }

                                if (Math.Abs(rightElbowDiff) > elbowDiffThresh)
                                {
                                    Color c;
                                    if (rightElbowDiff > 0)
                                    {
                                        c = Color.Blue;
                                    }
                                    else
                                    {
                                        c = Color.Red;
                                    }
                                    for (int j = (int)elbowRight.Y - 10; j < elbowRight.Y + 10; j++)
                                    {
                                        for (int i = (int)elbowRight.X - 10; i < elbowRight.X + 10; i++)
                                        {
                                            depthColors[j * depthWidth + i] = c.ToArgb();
                                        }
                                    }
                                }

                                if (Math.Abs(leftShoulderDiff) > shoulderDiffThresh)
                                {
                                    Color c;
                                    if (leftShoulderDiff > 0)
                                    {
                                        c = Color.Blue;
                                    }
                                    else
                                    {
                                        c = Color.Red;
                                    }
                                    for (int j = (int)shoulderLeft.Y - 10; j < shoulderLeft.Y + 10; j++)
                                    {
                                        for (int i = (int)shoulderLeft.X - 10; i < shoulderLeft.X + 10; i++)
                                        {
                                            depthColors[j * depthWidth + i] = c.ToArgb();
                                        }
                                    }
                                }
                                if (Math.Abs(rightShoulderDiff) > shoulderDiffThresh)
                                {
                                    Color c;
                                    if (rightShoulderDiff > 0)
                                    {
                                        c = Color.Blue;
                                    }
                                    else
                                    {
                                        c = Color.Red;
                                    }
                                    for (int j = (int)shoulderRight.Y - 10; j < shoulderRight.Y + 10; j++)
                                    {
                                        for (int i = (int)shoulderRight.X - 10; i < shoulderRight.X + 10; i++)
                                        {
                                            depthColors[j * depthWidth + i] = c.ToArgb();
                                        }
                                    }
                                }

                                if (Math.Abs(leftHandDiff) > handDiffThresh)
                                {
                                    Color c;
                                    if (leftHandDiff > 0)
                                    {
                                        c = Color.Blue;
                                    }
                                    else
                                    {
                                        c = Color.Red;
                                    }
                                    for (int j = (int)handLeft.Y - 10; j < handLeft.Y + 10; j++)
                                    {
                                        for (int i = (int)handLeft.X - 10; i < handLeft.X + 10; i++)
                                        {
                                            depthColors[j * depthWidth + i] = c.ToArgb();
                                        }
                                    }
                                }
                                if (Math.Abs(rightHandDiff) > handDiffThresh)
                                {
                                    Color c;
                                    if (rightHandDiff > 0)
                                    {
                                        c = Color.Blue;
                                    }
                                    else
                                    {
                                        c = Color.Red;
                                    }
                                    for (int j = (int)handRight.Y - 10; j < handRight.Y + 10; j++)
                                    {
                                        for (int i = (int)handRight.X - 10; i < handRight.X + 10; i++)
                                        {
                                            depthColors[j * depthWidth + i] = c.ToArgb();
                                        }
                                    }
                                }
                                 
                            }
                            catch
                            { 
                            }

                            //only continue to volume calculation if the joints are relatively close enough on distance
                            if (Math.Abs(spineDiff) <= spineDiffThresh && Math.Abs(leftElbowDiff) <= elbowDiffThresh && Math.Abs(rightElbowDiff) <= elbowDiffThresh && Math.Abs(leftShoulderDiff) <= shoulderDiffThresh && Math.Abs(rightShoulderDiff) <= shoulderDiffThresh && Math.Abs(leftHandDiff) <= handDiffThresh && Math.Abs(rightHandDiff) <= handDiffThresh && bodyJoints.rightElbow.Z != 0 && bodyJoints.leftElbow.Z != 0)
                            {
                                timer2--;
                            }
                            else
                            {
                                timer2 = 5;
                            }
                            if (timer2 <= 0)
                            {
                                //avgDepthImagePixels = SmoothDepthFrame(avgDepthImagePixels);
                                //reset timer
                                timer = 1;

                                //use the elbows as the starting pixel for the clusters
                                DepthSpacePoint leftElbowD = mapper.MapCameraPointToDepthSpace(bodyJoints.leftElbow);
                                DepthSpacePoint rightElbowD = mapper.MapCameraPointToDepthSpace(bodyJoints.rightElbow);
                                DepthSpacePoint leftShoulderD = mapper.MapCameraPointToDepthSpace(bodyJoints.leftShoulder);
                                DepthSpacePoint rightShoulderD = mapper.MapCameraPointToDepthSpace(bodyJoints.rightShoulder);
                                DepthSpacePoint leftHandD = mapper.MapCameraPointToDepthSpace(bodyJoints.leftHand);
                                DepthSpacePoint rightHandD = mapper.MapCameraPointToDepthSpace(bodyJoints.rightHand);
                                isProcessing = true;
                                //avgDepthImagePixels.CopyTo(workingAvgDepthImagePixels, 0);

                                //map depth data to real space
                                mapper.MapDepthFrameToCameraSpace(workingAvgDepthImagePixels, cameraSpacePoints);

                                ushort[] yMaxDepths1 = new ushort[depthHeight];
                                ushort[] xMaxDepths1 = new ushort[depthWidth];
                                ushort[] yMaxDepths2 = new ushort[depthHeight];
                                ushort[] xMaxDepths2 = new ushort[depthWidth];

                                //get volume depths and center pixels for both arms
                                List<Point3D> volumeDepths1 = new List<Point3D>();
                                Point3D center1 = new Point3D((int)leftElbowD.X, (int)leftElbowD.Y, workingAvgDepthImagePixels[(int)(leftElbowD.Y * depthWidth + leftElbowD.X)]);
                                int[] arr1 = FindClusterBounded(center1, (int)leftHandD.X + 2, (int)leftShoulderD.X - 3, (int) leftShoulderD.Y - 2, (int)leftHandD.Y - 2, workingAvgDepthImagePixels, xMaxDepths1, yMaxDepths1);
                                List<Point3D> volumeDepths2 = new List<Point3D>();
                                Point3D center2 = new Point3D((int)rightElbowD.X, (int)rightElbowD.Y, workingAvgDepthImagePixels[(int)(rightElbowD.Y * depthWidth + rightElbowD.X)]);
                                int[] arr2 = FindClusterBounded(center2, (int)rightShoulderD.X + 3, (int)rightHandD.X - 2, (int) rightShoulderD.Y - 2, (int)rightHandD.Y - 2, workingAvgDepthImagePixels, xMaxDepths2, yMaxDepths2);

                                for (int i = 0; i < depthWidth * depthHeight; i++)
                                {
                                    double dep = workingAvgDepthImagePixels[i];
                                    if (arr1[i] == 1 && dep != 0)
                                    {
                                        volumeDepths1.Add(new Point3D(i % depthWidth, i / depthWidth, dep, cameraSpacePoints[i]));
                                        depthColors[i] = (Color.FromArgb(140, Color.Purple)).ToArgb();
                                    }
                                    if (arr2[i] == 1 && dep != 0)
                                    {
                                        volumeDepths2.Add(new Point3D(i % depthWidth, i / depthWidth, dep, cameraSpacePoints[i]));
                                        depthColors[i] = (Color.FromArgb(140, Color.Purple)).ToArgb();
                                    }
                                }

                                //draw everything
                                if (mainForm.IsDisposed == true)
                                {
                                    break;
                                }
                                try
                                {
                                    img.UnlockBits(imgData);

                                    System.Runtime.InteropServices.Marshal.Copy(depthColors, 0, ptr, depthWidth * depthHeight);

                                    graphics.Clear(Color.White);
                                    graphics.DrawImage(img, new Point(0, 0));
                                    graphics.DrawString("Shoulder Left: " + Math.Round(bodyJoints.leftShoulder.Z, 4), new Font("Tahoma", 8), Brushes.Red, new System.Drawing.PointF(10, 10));
                                    graphics.DrawString("Elbow Left: " + Math.Round(bodyJoints.leftElbow.Z, 4), new Font("Tahoma", 8), Brushes.Red, new System.Drawing.PointF(10, 20));
                                    graphics.DrawString("Hand Left: " + Math.Round(bodyJoints.leftHand.Z, 4), new Font("Tahoma", 8), Brushes.Red, new System.Drawing.PointF(10, 30));
                                    graphics.DrawString("Shoulder Right: " + Math.Round(bodyJoints.rightShoulder.Z, 4), new Font("Tahoma", 8), Brushes.Red, new System.Drawing.PointF(140, 10));
                                    graphics.DrawString("Elbow Right: " + Math.Round(bodyJoints.rightElbow.Z, 4), new Font("Tahoma", 8), Brushes.Red, new System.Drawing.PointF(140, 20));
                                    graphics.DrawString("Hand Right: " + Math.Round(bodyJoints.rightHand.Z, 4), new Font("Tahoma", 8), Brushes.Red, new System.Drawing.PointF(140, 30));
                                    graphics.DrawLine(new Pen(Color.Red), new Point(depthWidth / 2 - 3, depthHeight / 2), new Point(depthWidth / 2 + 3, depthHeight / 2));
                                    graphics.DrawLine(new Pen(Color.Red), new Point(depthWidth / 2, depthHeight / 2 - 3), new Point(depthWidth / 2, depthHeight / 2 + 3));
                                    hasDrawn = true;
                                }
                                catch(Exception e)
                                {
                                    break;
                                }

                                //integrate depths
                                /*
                                double leftAlign = (.001 * IntegrateDepths(volumeDepths1, bodyJoints.spine.Z * 1000));
                                Console.WriteLine("Volume of Left: " + leftAlign + " cm^3");
                                double rightAlign = (.001 * IntegrateDepths(volumeDepths2, bodyJoints.spine.Z * 1000));
                                Console.WriteLine("Volume of Right: " + rightAlign + " cm^3");
                                */
                                
                                /*
                                double leftLine = (.001 * IntegrateDepths(volumeDepths1, arr1, bodyJoints.leftHand.X * 1000, bodyJoints.leftHand.Z * 1000, bodyJoints.leftElbow.X * 1000, bodyJoints.leftElbow.Z * 1000, bodyJoints.leftShoulder.X * 1000, bodyJoints.leftShoulder.Z * 1000));
                                Console.WriteLine("Volume of Left (Line) : " + leftLine + " cm^3");
                                double rightLine = (.001 * IntegrateDepths(volumeDepths2, arr2, bodyJoints.rightHand.X * 1000, bodyJoints.rightHand.Z * 1000, bodyJoints.rightElbow.X * 1000, bodyJoints.rightElbow.Z * 1000, bodyJoints.rightShoulder.X * 1000, bodyJoints.rightShoulder.Z * 1000));
                                Console.WriteLine("Volume of Right (Line) : " + rightLine + " cm^3");
                                 */
                                
                                double leftMax = (.001 * IntegrateDepths(volumeDepths1, arr1, xMaxDepths1, yMaxDepths1));
                                Console.WriteLine("Volume of Left (Maxes) : " + leftMax + " cm^3");
                                double rightMax = (.001 * IntegrateDepths(volumeDepths2, arr2, xMaxDepths2, yMaxDepths2));
                                Console.WriteLine("Volume of Right (Maxes) : " + rightMax + " cm^3");
                                
                                string uniqueIdentifier = DateTime.Now.ToString("dd-MM-yyyy_hh-mm-ss").Substring(0,5);
                                System.IO.File.AppendAllText(@"C:\Users\Kevin\Kinect\Data" + uniqueIdentifier + ".txt", "" + leftMax + " " + rightMax + " " + bodyJoints.spine.Z + Environment.NewLine);
                                //System.IO.File.AppendAllText(@"C:\Users\Kevin\Kinect\Data" + uniqueIdentifier + ".txt", "" + leftLine + " " + leftMax + " " + rightLine + " " + rightMax + " " + bodyJoints.spine.Z + Environment.NewLine);
                                //System.IO.File.AppendAllText(@"C:\Users\Kevin\Kinect\Data" + uniqueIdentifier + ".txt", "" + leftAlign + " " + rightAlign + Environment.NewLine);

                                isProcessing = false;
                                
                            }
                            
                        }
                    }
                    else
                    {
                        depthClipMin = 0;
                        depthClipMax = 6000;
                    }
                    if (!hasDrawn)
                    {
                        //draw everything
                        if (mainForm.IsDisposed == true)
                        {
                            break;
                        }
                        try
                        {
                            System.Runtime.InteropServices.Marshal.Copy(depthColors, 0, ptr, depthWidth * depthHeight);
                            img.UnlockBits(imgData);
                            graphics.Clear(Color.White);
                            graphics.DrawImage(img, new Point(0, 0));
                            graphics.DrawString("Shoulder Left: " + Math.Round(bodyJoints.leftShoulder.Z, 4), new Font("Tahoma", 8), Brushes.Red, new System.Drawing.PointF(10, 10));
                            graphics.DrawString("Elbow Left: " + Math.Round(bodyJoints.leftElbow.Z, 4), new Font("Tahoma", 8), Brushes.Red, new System.Drawing.PointF(10, 20));
                            graphics.DrawString("Hand Left: " + Math.Round(bodyJoints.leftHand.Z, 4), new Font("Tahoma", 8), Brushes.Red, new System.Drawing.PointF(10, 30));
                            graphics.DrawString("Shoulder Right: " + Math.Round(bodyJoints.rightShoulder.Z, 4), new Font("Tahoma", 8), Brushes.Red, new System.Drawing.PointF(140, 10));
                            graphics.DrawString("Elbow Right: " + Math.Round(bodyJoints.rightElbow.Z, 4), new Font("Tahoma", 8), Brushes.Red, new System.Drawing.PointF(140, 20));
                            graphics.DrawString("Hand Right: " + Math.Round(bodyJoints.rightHand.Z, 4), new Font("Tahoma", 8), Brushes.Red, new System.Drawing.PointF(140, 30));
                            graphics.DrawLine(new Pen(Color.Red), new Point(depthWidth / 2 - 3, depthHeight / 2), new Point(depthWidth / 2 + 3, depthHeight / 2));
                            graphics.DrawLine(new Pen(Color.Red), new Point(depthWidth / 2, depthHeight / 2 - 3), new Point(depthWidth / 2, depthHeight / 2 + 3));

                            img.Dispose();
                        }
                        catch(Exception e)
                        {
                            break;
                        }
                    }
                }
                timer--;
                
            }
        }

        private void AcquireFrames()
        {
            while (true)
            {
                //acquire latest frames
                reader.AcquireLatestFrame();
                //bodyReader.AcquireLatestFrame();
                Thread.Sleep(10000);
            }
        }

        /// <summary>
        /// Method mainly used for debugging.  Attached to mouse click event.
        /// Displays relevant information in console
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Control_MouseClick(Object sender, MouseEventArgs e)
        {
            try
            {
                Console.WriteLine("X: " + e.X);
                Console.WriteLine("Y: " + e.Y);
                int col = alignedColorImagePixels[e.Y * depthWidth + e.X];
                int col2 = alignedColorImagePixels[(e.Y - 1) * depthWidth + e.X];
                Console.WriteLine("Color Diff: " + CompareColors(col, col2));

                int loc1 = e.Y * depthWidth + e.X;
                int loc2 = (e.Y - 1) * depthWidth + e.X;

                Point3D pt1 = new Point3D(loc1 % depthWidth, loc1 / depthWidth, workingAvgDepthImagePixels[loc1], cameraSpacePoints[loc1]);
                Point3D pt2 = new Point3D(loc2 % depthWidth, loc2 / depthWidth, workingAvgDepthImagePixels[loc2], cameraSpacePoints[loc2]);
                Console.WriteLine("Real World X and Y: " + (pt1.camX) + " " + (pt1.camY));
                Console.WriteLine("Current Depth: " + pt1.z);
                Console.WriteLine("Depth Diff: " + pt1.FindDistance(pt2));
                //Console.WriteLine("Inf: " + infImagePixels[e.Y * depthWidth + e.X]);
                //Console.WriteLine("Inf Above: " + infImagePixels[(e.Y - 1) * depthWidth + e.X]);
            }
            catch
            {
                Console.WriteLine("Out of bounds");
            }
        }

        /// <summary>
        /// Event handler attached to Enable Smoothing button
        /// Enables smoothing if disabled and disables if enabled
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Smooth_Click(Object sender, EventArgs e)
        {
            if (!smoothEnabled)
            {
                smoothButton.Text = "Disable Smoothing";
            }
            else
            {
                smoothButton.Text = "Enable Smoothing";
            }
            smoothEnabled = !smoothEnabled;
        }

        /// <summary>
        /// Event handler attached to the Enable Color button
        /// Enabled color if disabled and disables if enabled
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Color_Click(Object sender, EventArgs e)
        {
            if (!colorEnabled)
            {
                colorButton.Text = "Disable Color";
                //depthWeight = weightSlider.Value;
                //colorWeight = 1 - depthWeight;
                //weightSlider.Enabled = true;
            } 
            else
            {
                colorButton.Text = "Enable Color";
                //depthWeight = 1;
                //colorWeight = 0;
                //weightSlider.Value = 100;
                //weightSlider.Enabled = false;
            }
            colorEnabled = !colorEnabled;           
        }

        /// <summary>
        /// Event handler attached to Dump Depth Frame Data button
        /// Dumps the depth frame data into a text file 
        /// Ordered X, Y, Z with X and Y being pixel X and Y (y is top to bottom) and Z is real-space Z
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Dump_Click(Object sender, EventArgs e)
        {
            if (!isProcessing)
            {
                isProcessing = true;
                string[] lines = new string[depthWidth * depthHeight];
                for (int i = 0; i < depthWidth * depthHeight; i++)
                {
                    
                    lines[i] = "" + (i % depthWidth) + " " + (i / depthWidth) + " "  + avgDepthImagePixels[i];
                    
                }
                string uniqueIdentifier = DateTime.Now.ToString("dd-MM-yyyy_hh-mm-ss");
                System.IO.File.WriteAllLines(@"C:\Users\Public\KinectData\DepthData" + uniqueIdentifier + ".txt", lines);
                isProcessing = false;
            }
        }

        /// <summary>
        /// Event handler attached to scrolling of the weight scrollbar
        /// Updates variables and textbox based on the where the scrollbar is
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Weight_Scroll(object sender, EventArgs e)
        {
            depthWeight = weightSlider.Value/100.0;
            colorWeight = 1 - weightSlider.Value / 100.0;
            weightLabel.Text = "" + depthWeight;
        }

        /// <summary>
        /// Event handler attacched to scrolling of absolute threshold scrollbar
        /// Updates absolute threshold based on scrollbar's value
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Thresh_Scroll(object sender, EventArgs e)
        {
            absoluteThreshold = threshSlider.Value;
            threshLabel.Text = "" + absoluteThreshold;
        }

        /// <summary>
        /// Event handler attached to frame integration scrollbar
        /// Updates the number of frames to average based on the scrollbar's value
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Frame_Scroll(object sender, EventArgs e)
        {
            framesToIntegrate = frameSlider.Value;
            frameLabel.Text = "" + framesToIntegrate;
        }

        /// <summary>
        /// Event handler attached to the depth start scrollbar
        /// Updates where to start integrating from
        /// DEPRECATED
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        /// 
        /*
        private void DepthStart_Scroll(object sender, EventArgs e)
        {
            depthStart = depthStartSlider.Value;
            depthStartLabel.Text = "" + depthStart;
        }
        */
        /// <summary>
        /// Method to find the volume cluster using a seed pixel as a basis to start from
        /// Could probably be optimized better.
        /// Last updated: 10/3/2014
        /// </summary>
        /// <param name="seed">pixel to use to compare with other pixels</param>
        /// <returns>array of addresses of pixels that are part of the cluster</returns>
        private int[] FindCluster(Point3D seed, ushort[] depthPixels)
        {
            int[] arr = new int[depthWidth * depthHeight];
            lock (this)
            {
                //stack for holding pixels that still need to be checked
                Stack<int> toCheck = new Stack<int>();
                //push the seed onto it
                toCheck.Push(seed.y * depthWidth + seed.x);
                //continue popping until there is nothing left to check
                while (toCheck.Count != 0)
                {
                    //put the top pixel into curr
                    int curr = toCheck.Pop();
                    //set the address to 1, indicating it is part of the cluster
                    arr[curr] = 1;
                    //check pixel above current
                    if (curr - depthWidth >= 0 && arr[curr - depthWidth] != 1 && CalculateDifference(curr, curr - depthWidth, depthPixels) < absoluteThreshold)
                    {
                        toCheck.Push(curr - depthWidth);
                    }
                    //check pixel below current
                    if (curr + depthWidth < depthWidth * depthHeight && arr[curr + depthWidth] != 1 && CalculateDifference(curr, curr + depthWidth, depthPixels) < absoluteThreshold)
                    {
                        toCheck.Push(curr + depthWidth);
                    }
                    //check pixel left of current
                    if (curr - 1 >= 0 && arr[curr - 1] != 1 && CalculateDifference(curr, curr - 1, depthPixels) < absoluteThreshold)
                    {
                        toCheck.Push(curr - 1);
                    }
                    //check pixel right of current
                    if (curr + 1 < depthWidth * depthHeight && arr[curr + 1] != 1 && CalculateDifference(curr, curr + 1, depthPixels) < absoluteThreshold)
                    {
                        toCheck.Push(curr + 1);
                    }
                }
            }
            return arr;
        }

        /// <summary>
        /// Method to find the volume cluster using a seed pixel within a certain bounded range
        /// Last updated: 11/7/2014
        /// </summary>
        /// <param name="seed"></param>
        /// <param name="leftBound"></param>
        /// <param name="rightBound"></param>
        /// <param name="upperBound"></param>
        /// <param name="lowerBound"></param>
        /// <returns></returns>
        private int[] FindClusterBounded(Point3D seed, int leftBound, int rightBound, int upperBound, int lowerBound, ushort[] depthPixels, ushort[] xMaxDepth, ushort[]yMaxDepth)
        {
            int[] arr = new int[depthWidth * depthHeight];
            lock (this)
            {
                Stack<int> toCheck = new Stack<int>();
                toCheck.Push(seed.y * depthWidth + seed.x);
                while (toCheck.Count != 0)
                {
                    int curr = toCheck.Pop();
                    int x = curr % depthWidth;
                    int y = curr / depthWidth;
                    if (y > upperBound && y < lowerBound && x > leftBound && x < rightBound)
                    {
                        double up = CalculateDifference(curr, curr - depthWidth, depthPixels);
                        double down = CalculateDifference(curr, curr + depthWidth, depthPixels);
                        double left = CalculateDifference(curr, curr - 1, depthPixels);
                        double right = CalculateDifference(curr, curr + 1, depthPixels);
                        if (!(Double.IsInfinity(up) || Double.IsInfinity(down) || Double.IsInfinity(left) || Double.IsInfinity(right)))
                        {
                            arr[curr] = 1;
                            if (yMaxDepth[y] < depthPixels[curr]) yMaxDepth[y] = depthPixels[curr];
                            if (xMaxDepth[x] < depthPixels[curr]) xMaxDepth[x] = depthPixels[curr];
                            if (curr - depthWidth >= 0 && arr[curr - depthWidth] != 1 && up < absoluteThreshold)
                            {
                                toCheck.Push(curr - depthWidth);
                            }
                            if (curr + depthWidth < depthWidth * depthHeight && arr[curr + depthWidth] != 1 && down < absoluteThreshold)
                            {
                                toCheck.Push(curr + depthWidth);
                            }
                            if (curr - 1 >= 0 && arr[curr - 1] != 1 && left < absoluteThreshold)
                            {
                                toCheck.Push(curr - 1);
                            }
                            if (curr + 1 < depthWidth * depthHeight && arr[curr + 1] != 1 && right < absoluteThreshold)
                            {
                                toCheck.Push(curr + 1);
                            }
                        }
                    }
                }
            }
            return arr;
        }

        /// <summary>
        /// Finds a value to be used with absolute threshold to determine how different two pixels are
        /// and whether or not to include them in the object being recognized.
        /// Last updated: 10/3/2014
        /// </summary>
        /// <param name="loc1">location of one of the pixels</param>
        /// <param name="loc2">location of another pixel</param>
        /// <param name="depthPixels">depth data to work with</param>
        /// <returns>value to be used with absolute threshold</returns>
        private double CalculateDifference(int loc1, int loc2, ushort[] depthPixels)
        {
            double depthDiff = 0;
            double colorDiff = 0;
            if (depthWeight != 0)
            {
                depthDiff = depthWeight * CompareDepths(loc1, loc2, depthPixels);
            }
            //Console.WriteLine("Depth Diff: " + depthWeight * (Math.Exp(Math.Abs(depthImagePixels[loc1] - depthImagePixels[loc2]))));
            if (colorWeight != 0)
            {
                colorDiff = colorWeight * CompareColors(alignedColorImagePixels[loc1], alignedColorImagePixels[loc2]);
            }
            return depthDiff + colorDiff;
        }

        /// <summary>
        /// Method for comparing two depths at two locations and representing the difference between them.
        /// Weighted mostly exponentially such that the farther the distance, the higher the value returned and 
        /// thus the harder to fit in the absolute threshold.
        /// Last updated: 10/10/2014
        /// </summary>
        /// <param name="loc1">location of first depth to compare</param>
        /// <param name="loc2">location of second depth to compare</param>
        /// <param name="depthPixels">depth data to work with</param>
        /// <returns>value to be used with depth weight and absolute threshold to determine when depths are too different</returns>
        private double CompareDepths(int loc1, int loc2, ushort[] depthPixels)
        {
            Point3D pt1 = new Point3D(loc1 % depthWidth, loc1 / depthWidth, depthPixels[loc1], cameraSpacePoints[loc1]);
            Point3D pt2 = new Point3D(loc2 % depthWidth, loc2 / depthWidth, depthPixels[loc2], cameraSpacePoints[loc2]);

            //int inf1 = infImagePixels[loc1];
            //int inf2 = infImagePixels[loc2];
            //Console.WriteLine((inf1 + inf2) / 1000);
            //Console.WriteLine(pt1.FindDistance(pt2));

            //fairly arbitrary way of finding a depth difference
            return pt1.FindDistance(pt2) * 20;

            /*double depDiff = Math.Abs(avgDepthImagePixels[loc1] - avgDepthImagePixels[loc2]);
            return Math.Pow(4, depDiff - 2);*/
        }

        /// <summary>
        /// Method for comparing two colors at two pixels, giving a value representing the difference between the two. 
        /// Last updated: 11/26/2014
        /// </summary>
        /// <param name="col1">first color to compare</param>
        /// <param name="col2">second color to compare to first</param>
        /// <returns>arbitrary value used with absolute threshold and color weight to determine when color is too different</returns>
        private double CompareColors(int col1, int col2)
        {
            //bad data
            if ((col1 | col2) == 0) return 1000;

            //isolate RGB values
            int red1 = (col1 & 0x00ff0000) >> 16;
            int gre1 = (col1 & 0x0000ff00) >> 8;
            int blu1 = col1 & 0x000000ff;
            int red2 = (col2 & 0x00ff0000) >> 16;
            int gre2 = (col2 & 0x0000ff00) >> 8;
            int blu2 = col2 & 0x000000ff;

            //convert to LAB
            double[] LAB1 = RGBToLAB(red1, gre1, blu1);
            double[] LAB2 = RGBToLAB(red2, gre2, blu2);

            //return value based on CIE76
            return 60 * Math.Sqrt(Math.Pow(LAB1[0] - LAB2[0], 2) + Math.Pow(LAB1[1] - LAB2[1], 2) + Math.Pow(LAB1[2] - LAB2[2], 2));

        }

        /// <summary>
        /// Method for converting RGB values to LAB for calculation
        /// Used for color-based object segmentation
        /// </summary>
        /// <param name="red">red value</param>
        /// <param name="green">green value</param>
        /// <param name="blue">blue value</param>
        /// <returns>array of three values holding L,A,B</returns>
        private double[] RGBToLAB(double red, double green, double blue)
        {
            double[] toReturn  = new double[3];
            double rLinear = (double)red / 255.0;
            double gLinear = (double)green / 255.0;
            double bLinear = (double)blue / 255.0;

            // convert to a sRGB form
            double r = (rLinear > 0.04045) ? Math.Pow((rLinear + 0.055) / (
                1 + 0.055), 2.2) : (rLinear / 12.92);
            double g = (gLinear > 0.04045) ? Math.Pow((gLinear + 0.055) / (
                1 + 0.055), 2.2) : (gLinear / 12.92);
            double b = (bLinear > 0.04045) ? Math.Pow((bLinear + 0.055) / (
                1 + 0.055), 2.2) : (bLinear / 12.92);

            // converts
            double x = r * 0.4124 + g * 0.3576 + b * 0.1805;
            double y = r * 0.2126 + g * 0.7152 + b * 0.0722;
            double z = r * 0.0193 + g * 0.1192 + b * 0.9505;

            toReturn[0] = 116.0 * Fxyz( y/1.0 ) -16;
            toReturn[1] = 500.0 * (Fxyz( x/.9505 ) - Fxyz( y/1.0) );
            toReturn[2] = 200.0 * (Fxyz( y/1.0 ) - Fxyz( z/1.0890) );

            return toReturn;
        }

        /// <summary>
        /// Method used in RGB to LAB conversion
        /// </summary>
        /// <param name="t"></param>
        /// <returns></returns>
        private double Fxyz(double t)
        {
            return ((t > 0.008856) ? Math.Pow(t, (1.0 / 3.0)) : (7.787 * t + 16.0 / 116.0));
        }

        /// <summary>
        /// Method for estimating volumes from depth data
        /// This could definitely use a better method, may have overcomplicated it at some point
        /// Last updated: 11/10/2014
        /// </summary>
        /// <param name="data">the data to integrate</param>
        /// <param name="depthStart">the value to assume integration from, usually the highest value</param>
        /// <returns>estimated volume</returns>
        private double IntegrateDepths(List<Point3D> data, double depthStart)
        {
            
            double sum = 0;
            /*
            foreach (Point3D p in data)
            {
                double newDepth = depthStart - p.z;
                Point3D up = new Point3D(p.x, p.y - 1, avgDepthImagePixels[(p.y - 1) * depthWidth + p.x], cameraSpaceFactors[(p.y - 1) * depthWidth + p.x]);
                Point3D down = new Point3D(p.x, p.y + 1, avgDepthImagePixels[(p.y + 1) * depthWidth + p.x], cameraSpaceFactors[(p.y + 1) * depthWidth + p.x]);
                Point3D left = new Point3D(p.x - 1, p.y, avgDepthImagePixels[p.y * depthWidth + p.x - 1], cameraSpaceFactors[p.y * depthWidth + p.x - 1]);
                Point3D right = new Point3D(p.x + 1, p.y, avgDepthImagePixels[p.y * depthWidth + p.x + 1], cameraSpaceFactors[p.y * depthWidth + p.x + 1]);
                
                sum += Math.Pow(unit * (depthStart + p.z) / 2.0, 2) * newDepth;
                if (maxNew < newDepth) maxNew = newDepth;
            }*/
            lock (this)
            {
                //double unit = cameraSpaceFactors[(int)(Math.Round(depthHeight / 2.0 * depthWidth + depthWidth / 2.0))].X - cameraSpaceFactors[(int)(Math.Round(depthHeight / 2.0 * depthWidth + depthWidth / 2.0)) + 1].X;
                if (data.Count >= 5)
                {
                    //double unit = 1000000 * Math.Abs((cameraSpacePoints[data[0].y * depthWidth + data[0].x].X - cameraSpacePoints[data[0].y * depthWidth + data[0].x + 1].X) * (cameraSpacePoints[data[0].y * depthWidth + data[0].x].Y - cameraSpacePoints[(data[0].y + 1) * depthWidth + data[0].x].Y));
                    //Console.WriteLine(unit);
                    foreach (Point3D p in data)
                    {
                        //upon further thought, this may be fairly inaccurate because of the edge points
                        try
                        {
                            /*
                            double left = Math.Abs(cameraSpacePoints[p.y * depthWidth + p.x].X - cameraSpacePoints[p.y * depthWidth + p.x - 1].X);
                            double right = Math.Abs(cameraSpacePoints[p.y * depthWidth + p.x].X - cameraSpacePoints[p.y * depthWidth + p.x + 1].X);
                            double up = Math.Abs(cameraSpacePoints[p.y * depthWidth + p.x].Y - cameraSpacePoints[(p.y - 1) * depthWidth + p.x].Y);
                            double down = Math.Abs(cameraSpacePoints[p.y * depthWidth + p.x].Y - cameraSpacePoints[(p.y + 1) * depthWidth + p.x].Y);
                            double Xavg;
                            double Yavg;
                            if (double.IsInfinity(left) || double.IsNaN(left) || left == 0) Xavg = 2 * right;
                            else if (double.IsInfinity(right) || double.IsNaN(right) || right == 0) Xavg = 2 * left;
                            else Xavg = (right + left) / 2.0;
                            if (double.IsInfinity(up) || double.IsNaN(up) || up == 0) Yavg = 2 * down;
                            else if (double.IsInfinity(down) || double.IsNaN(down) || down == 0) Yavg = 2 * up;
                            else Yavg = (up + down) / 2.0;
                            double unit = 1000000 * Xavg * Yavg;
                            //double unit = 1000000 * (Math.Abs(cameraSpacePoints[p.y * depthWidth + p.x - 1].X - cameraSpacePoints[p.y * depthWidth + p.x + 1].X) * Math.Abs(cameraSpacePoints[(p.y - 1) * depthWidth + p.x].Y - cameraSpacePoints[(p.y + 1) * depthWidth + p.x].Y))/4.0;
                            //Console.WriteLine(p.z);
                            //Console.WriteLine(unit);
                            if (double.IsInfinity(unit) || double.IsNaN(unit)) continue;
                             */
                            int multiplier = 1;
                            double left;
                            double right;
                            double up;
                            double down;
                            if (data.Contains(new Point3D(p.x - 1, p.y, p.z)))
                            {
                                left = Math.Abs(cameraSpacePoints[p.y * depthWidth + p.x].X - cameraSpacePoints[p.y * depthWidth + p.x - 1].X);
                            }
                            else
                            {
                                left = 0;
                                multiplier = 4;
                            }
                            if (data.Contains(new Point3D(p.x + 1, p.y, p.z)))
                            {
                                right = Math.Abs(cameraSpacePoints[p.y * depthWidth + p.x].X - cameraSpacePoints[p.y * depthWidth + p.x + 1].X);
                            }
                            else if (left == 0)
                            {
                                continue;
                            }
                            else
                            {
                                right = 0;
                                multiplier = 4;
                            }
                            double Xavg = multiplier * (left + right) / 2.0;
                            multiplier = 1;
                            if (data.Contains(new Point3D(p.x, p.y - 1, p.z)))
                            {
                                up = Math.Abs(cameraSpacePoints[p.y * depthWidth + p.x].Y - cameraSpacePoints[(p.y - 1) * depthWidth + p.x].Y);
                            }
                            else
                            {
                                up = 0;
                                multiplier = 4;
                            }
                            if (data.Contains(new Point3D(p.x, p.y + 1, p.z)))
                            {
                                down = Math.Abs(cameraSpacePoints[p.y * depthWidth + p.x].Y - cameraSpacePoints[(p.y + 1) * depthWidth + p.x].Y);
                            }
                            else if (up == 0)
                            {
                                continue;
                            }
                            else
                            {
                                down = 0;
                                multiplier = 4;
                            }
                            double Yavg = multiplier * (up + down) / 2.0;
                            double unit = 1000000 * Xavg * Yavg;
                            double newDepth = depthStart - p.z;
                            if (newDepth < 0) newDepth = 0;
                            
                            //better method for this is probably necessary
                            sum += (unit * newDepth);
                        }
                        catch
                        {

                        }
                    }
                }
            }
            return sum;
        }

        /// <summary>
        /// Method for estimating volumes from depth data
        /// This could definitely use a better method, may have overcomplicated it at some point
        /// Last updated: 11/10/2014
        /// </summary>
        /// <param name="data">the data to integrate</param>
        /// 
        /// <returns>estimated volume</returns>
        private double IntegrateDepths(List<Point3D> data, int[] contains, ushort[] xMaxDepths, ushort[] yMaxDepths)
        {

            double xSum = 0;
            double ySum = 0;
            /*
            foreach (Point3D p in data)
            {
                double newDepth = depthStart - p.z;
                Point3D up = new Point3D(p.x, p.y - 1, avgDepthImagePixels[(p.y - 1) * depthWidth + p.x], cameraSpaceFactors[(p.y - 1) * depthWidth + p.x]);
                Point3D down = new Point3D(p.x, p.y + 1, avgDepthImagePixels[(p.y + 1) * depthWidth + p.x], cameraSpaceFactors[(p.y + 1) * depthWidth + p.x]);
                Point3D left = new Point3D(p.x - 1, p.y, avgDepthImagePixels[p.y * depthWidth + p.x - 1], cameraSpaceFactors[p.y * depthWidth + p.x - 1]);
                Point3D right = new Point3D(p.x + 1, p.y, avgDepthImagePixels[p.y * depthWidth + p.x + 1], cameraSpaceFactors[p.y * depthWidth + p.x + 1]);
                
                sum += Math.Pow(unit * (depthStart + p.z) / 2.0, 2) * newDepth;
                if (maxNew < newDepth) maxNew = newDepth;
            }*/
            lock (this)
            {
                //double unit = cameraSpaceFactors[(int)(Math.Round(depthHeight / 2.0 * depthWidth + depthWidth / 2.0))].X - cameraSpaceFactors[(int)(Math.Round(depthHeight / 2.0 * depthWidth + depthWidth / 2.0)) + 1].X;
                if (data.Count >= 5)
                {
                    //double unit = 1000000 * Math.Abs((cameraSpacePoints[data[0].y * depthWidth + data[0].x].X - cameraSpacePoints[data[0].y * depthWidth + data[0].x + 1].X) * (cameraSpacePoints[data[0].y * depthWidth + data[0].x].Y - cameraSpacePoints[(data[0].y + 1) * depthWidth + data[0].x].Y));
                    //Console.WriteLine(unit);
                    foreach (Point3D p in data)
                    {
                        try
                        {
                            /*
                            int multiplier = 1;
                            double left;
                            double right;
                            double up;
                            double down;
                            if (contains[p.x - 1 + p.y * depthWidth] == 1)
                            {
                                left = Math.Abs(cameraSpacePoints[p.y * depthWidth + p.x].X - cameraSpacePoints[p.y * depthWidth + p.x - 1].X);
                            }
                            else
                            {
                                left = 0;
                                multiplier = 2;
                            }
                            if (contains[p.x + 1 + p.y * depthWidth] == 1)
                            {
                                right = Math.Abs(cameraSpacePoints[p.y * depthWidth + p.x].X - cameraSpacePoints[p.y * depthWidth + p.x + 1].X);
                            }
                            else if (left == 0)
                            {
                                continue;
                            }
                            else
                            {
                                right = 0;
                                multiplier = 2;
                            }
                            double Xavg = multiplier * (left + right) / 2.0;
                            multiplier = 1;
                            if (contains[p.x + (p.y - 1) * depthWidth] == 1)
                            {
                                up = Math.Abs(cameraSpacePoints[p.y * depthWidth + p.x].Y - cameraSpacePoints[(p.y - 1) * depthWidth + p.x].Y);
                            }
                            else
                            {
                                up = 0;
                                multiplier = 2;
                            }
                            if (contains[p.x + (p.y + 1) * depthWidth] == 1)
                            {
                                down = Math.Abs(cameraSpacePoints[p.y * depthWidth + p.x].Y - cameraSpacePoints[(p.y + 1) * depthWidth + p.x].Y);
                            }
                            else if (up == 0)
                            {
                                continue;
                            }
                            else
                            {
                                down = 0;
                                multiplier = 2;
                            }
                            double Yavg = multiplier * (up + down) / 2.0;
                            double unit = 1000000 * Xavg * Yavg;
                            */

                            double unit = (Math.Tan(70) * 2 * p.z / depthWidth) * (Math.Tan(60) * 2 * p.z / depthHeight);

                            double xNewDepth = xMaxDepths[p.x] - p.z;
                            double yNewDepth = yMaxDepths[p.y] - p.z;

                            /*
                            if (xNewDepth > 70)
                            {
                                Console.WriteLine(xNewDepth);
                                while (true) ;
                            }*/

                            if (xNewDepth < 0) xNewDepth = 0;
                            if (yNewDepth < 0) yNewDepth = 0;
                            xSum += (unit * xNewDepth);
                            ySum += (unit * yNewDepth);
                        }
                        catch
                        {

                        }
                    }


                }
            }
            Console.WriteLine("Volume with respect to X: " + xSum);
            Console.WriteLine("Volume with respect to Y: " + ySum);
            return Math.Max(xSum, ySum);
            /*
            if (ySum > xSum) return ySum;
            else return xSum;*/
        }

        private double IntegrateDepths(List<Point3D> data, int[] contains, double x1, double z1, double x2, double z2, double x3, double z3)
        {
            
            double sum = 0;
            
            lock (this)
            {
                if (data.Count >= 5)
                {
                    foreach (Point3D p in data)
                    {
                        double depthStart = 0;
                        if (Math.Sign(p.camX - x1) != Math.Sign(p.camX - x2))
                        {
                            depthStart = (z2 - z1) / (x2 - x1) * p.camX + (z1 - (z2 - z1) * (x1) / (x2 - x1));
                        }
                        else if (Math.Sign(p.camX - x2) != Math.Sign(p.camX - x3))
                        {
                            depthStart = (z3 - z2) / (x3 - x2) * p.camX + (z2 - (z3 - z2) * (x2) / (x3 - x2));
                        }
                        try
                        {
                            int multiplier = 1;
                            double left;
                            double right;
                            double up;
                            double down;
                            if (contains[p.y * depthWidth + p.x - 1] == 1)
                            {
                                left = Math.Abs(cameraSpacePoints[p.y * depthWidth + p.x].X - cameraSpacePoints[p.y * depthWidth + p.x - 1].X);
                            }
                            else
                            {
                                left = 0;
                                multiplier = 2;
                            }
                            if (contains[p.y * depthWidth + p.x + 1] == 1)
                            {
                                right = Math.Abs(cameraSpacePoints[p.y * depthWidth + p.x].X - cameraSpacePoints[p.y * depthWidth + p.x + 1].X);
                            }
                            else if (left == 0)
                            {
                                continue;
                            }
                            else
                            {
                                right = 0;
                                multiplier = 2;
                            }
                            double Xavg = multiplier * (left + right) / 2.0;
                            multiplier = 1;
                            if (contains[(p.y - 1) * depthWidth + p.x] == 1)
                            {
                                up = Math.Abs(cameraSpacePoints[p.y * depthWidth + p.x].Y - cameraSpacePoints[(p.y - 1) * depthWidth + p.x].Y);
                            }
                            else
                            {
                                up = 0;
                                multiplier = 2;
                            }
                            if (contains[(p.y + 1) * depthWidth + p.x] == 1)
                            {
                                down = Math.Abs(cameraSpacePoints[p.y * depthWidth + p.x].Y - cameraSpacePoints[(p.y + 1) * depthWidth + p.x].Y);
                            }
                            else if (up == 0)
                            {
                                continue;
                            }
                            else
                            {
                                down = 0;
                                multiplier = 2;
                            }
                            double Yavg = multiplier * (up + down) / 2.0;
                            double unit = 1000000 * Xavg * Yavg;
                            double newDepth = depthStart - p.z;
                            if (newDepth < 0) newDepth = 0;
                            sum += (unit * newDepth);
                        }
                        catch
                        {

                        }
                    }
                }
            }
            return sum;
        }

        /*
        /// <summary>
        /// Event handler for when a body frame arrives
        /// Updates body info and sets data in bodyJoints
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Reader_BodySourceFrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            Console.WriteLine("Arrived!");
            bool dataReceived = false;
            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (bodyFrame.BodyCount != 0)
                    {
                        bodies = new Body[bodyFrame.BodyCount];                           
                        bodyFrame.GetAndRefreshBodyData(bodies);
                        dataReceived = true;
                    }
                }
            }
            int i = 0;
            if (dataReceived && bodies != null)
            {
                foreach (Body body in bodies)
                {
                    if (body.IsTracked && !isProcessing)
                    {
                        i++;
                        bodyJoints.rightElbow = body.Joints[JointType.ElbowRight].Position;
                        bodyJoints.rightShoulder = body.Joints[JointType.ShoulderRight].Position;
                        bodyJoints.leftElbow = body.Joints[JointType.ElbowLeft].Position;
                        bodyJoints.leftShoulder = body.Joints[JointType.ShoulderLeft].Position;
                        bodyJoints.spine = body.Joints[JointType.SpineMid].Position;
                        bodyJoints.rightHand = body.Joints[JointType.HandRight].Position;
                        bodyJoints.leftHand = body.Joints[JointType.HandLeft].Position;
                        Console.WriteLine("Confidence: " + body.HandLeftConfidence);
                    }
                    else if (i == 0)
                    {
                        bodyJoints = new Skeleton();
                    }
                }
            }
            if (i == 0)
            {
                bodies = null;
            }
         //DISPOSE BODY FRAME!

        }
        */
        /// <summary>
        /// Event handler for multiSourceFrame arrived event
        /// Fills in depth and color data to their appropriate variables as well as any smoothing
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            bool validDepth = false;
            bool validColor = false;

            MultiSourceFrameReference frameReference = e.FrameReference;
            MultiSourceFrame multiSourceFrame = null;
            DepthFrame depthFrame = null;
            ColorFrame colorFrame = null;
            BodyFrame bodyFrame = null;
            
            //InfraredFrame infFrame = null;

            ushort[] temp = new ushort[depthImagePixels.Length];

            multiSourceFrame = frameReference.AcquireFrame();

            if (multiSourceFrame != null)
            {
                DepthFrameReference depthFrameReference = multiSourceFrame.DepthFrameReference;
                ColorFrameReference colorFrameReference = multiSourceFrame.ColorFrameReference;
                BodyFrameReference bodyFrameReference = multiSourceFrame.BodyFrameReference;
                //InfraredFrameReference infFrameReference = multiSourceFrame.InfraredFrameReference;

                depthFrame = depthFrameReference.AcquireFrame();
                colorFrame = colorFrameReference.AcquireFrame();
                bodyFrame = bodyFrameReference.AcquireFrame();
                //infFrame = infFrameReference.AcquireFrame();

                if (depthFrame != null)
                {
                    FrameDescription depthFrameDescription = depthFrame.FrameDescription;
                    depthWidth = depthFrameDescription.Width;
                    depthHeight = depthFrameDescription.Height;

                    if ((depthWidth * depthHeight) == depthImagePixels.Length)
                    {
                        depthFrame.CopyFrameDataToArray(temp);
                        depthImagePixels = temp;
                        validDepth = true;
                    }

                    depthFrame.Dispose();
                }
                if (colorFrame != null)
                {
                    FrameDescription colorFrameDescription = colorFrame.FrameDescription;
                    int colorWidth = colorFrameDescription.Width;
                    int colorHeight = colorFrameDescription.Height;

                    if ((colorWidth * colorHeight * sizeof(int)) == this.colorImagePixels.Length)
                    {
                        colorFrame.CopyConvertedFrameDataToArray(this.colorImagePixels, ColorImageFormat.Bgra);
                        validColor = true;
                    }
                    colorFrame.Dispose();
                }
               if (bodyFrame != null)
               {
                   bool dataReceived = false;
                   

                    if (bodyFrame.BodyCount != 0)
                    {
                        bodies = new Body[bodyFrame.BodyCount];
                        bodyFrame.GetAndRefreshBodyData(bodies);
                        dataReceived = true;
                    }
                    
                   
                   int i = 0;
                   if (dataReceived && bodies != null)
                   {
                       foreach (Body body in bodies)
                       {
                           i++;
                           if (!isProcessing)
                           {
                               if (body != null && body.IsTracked)
                               {
                                   bodyJoints.rightElbow = body.Joints[JointType.ElbowRight].Position;
                                   bodyJoints.rightShoulder = body.Joints[JointType.ShoulderRight].Position;
                                   bodyJoints.leftElbow = body.Joints[JointType.ElbowLeft].Position;
                                   bodyJoints.leftShoulder = body.Joints[JointType.ShoulderLeft].Position;
                                   bodyJoints.spine = body.Joints[JointType.SpineMid].Position;
                                   bodyJoints.rightHand = body.Joints[JointType.HandRight].Position;
                                   bodyJoints.leftHand = body.Joints[JointType.HandLeft].Position;
                                   //Console.WriteLine("Confidence: " + body.HandLeftConfidence);
                               }
                               else if (i == 0)
                               {
                                   bodyJoints = new Skeleton();
                               }
                           }
                       }
                   }
                   if (i == 0)
                   {
                       bodies = null;
                   }
                   bodyFrame.Dispose();
               }
                /*
                if (infFrame != null)
                {
                    FrameDescription infFrameDescription = infFrame.FrameDescription;
                    int infWidth = infFrameDescription.Width;
                    int infHeight = infFrameDescription.Height;

                    infFrame.CopyFrameDataToArray(infImagePixels);

                    infFrame.Dispose();
                }
                */
                if (validDepth)
                {
                     // Signal worker thread to process
                     this.depthReadyEvent.Set();
                }

                if (validColor)
                {
                    this.colorReadyEvent.Set();
                }
                
            }
            if (framesToIntegrate == 1 || smoothEnabled)
            {
                if (!smoothEnabled)
                {
                    avgDepthImagePixels = temp;
                }
                else
                {
                    avgDepthImagePixels = SmoothDepthFrame2(temp);
                }
            }
            else
            {
                depthFrames.Enqueue(temp);

                for (int i = 0; i < depthImagePixels.Length; i++)
                {
                    calcAvgDepthImg[i] = (calcAvgDepthImg[i] * (depthFrames.Count - 1) + temp[i]) / (double)depthFrames.Count;
                }

                while (depthFrames.Count > framesToIntegrate)
                {
                    ushort[] toRemove = depthFrames.Dequeue();
                    for (int i = 0; i < calcAvgDepthImg.Length; i++)
                    {
                        calcAvgDepthImg[i] = (calcAvgDepthImg[i] * (depthFrames.Count + 1) - toRemove[i]) / (double)(depthFrames.Count);
                    }
                }

                for (int i = 0; i < avgDepthImagePixels.Length; i++)
                {
                    avgDepthImagePixels[i] = (ushort)(Math.Round(calcAvgDepthImg[i]));
                }
            }

            if (!isProcessing) avgDepthImagePixels.CopyTo(workingAvgDepthImagePixels, 0);
        }

        /// <summary>
        /// Frame smoothing method.  Works fairly well but is fairly slow.  Can be used in order to get more accurate calculations. Uses median
        /// Other method is faster, so it is probably better to use that.
        /// Updated: 11/14/2014
        /// </summary>
        /// <param name="depthArray">depth data to smooth</param>
        /// <returns>smoothed depth data</returns>
        private ushort[] SmoothDepthFrame(ushort[] depthArray)
        {
            ushort[] toReturn = depthArray;
            
            //for (int i = 0; i < depthArray.Length; i++)
            Parallel.For(0, depthHeight, row =>
            {
                for (int col = 0; col < depthWidth; col++)
                {
                    try
                    {
                        int i = row * depthWidth + col;
                        if (depthArray[i] == 0 && !(depthArray[i - 1] == 0 && depthArray[i + 1] == 0 && depthArray[i - depthWidth] == 0 && depthArray[i + depthWidth] == 0))
                        {
                            int x = i % depthWidth;
                            int y = i / depthWidth;

                            List<ushort> list = new List<ushort>();
                            int range = 1;
                            for (int j = x - range; j <= x + range; j++)
                            {
                                for (int k = y - range; k <= y + range; k++)
                                {
                                    if (k != y && j != x) list.Add(depthArray[k * depthWidth + j]);
                                }
                            }
                            list.Sort();
                            toReturn[i] = list[list.Count / 2];
                        }
                    }
                    catch (Exception e)
                    {
                        Console.WriteLine(e.Message);
                    }
                }
            });
            return toReturn;
        }

        /// <summary>
        /// Alternate depth smoothing method that uses mode for replacing no-depth pixels.
        /// Currently in use.
        /// NOT ORIGINALLY MY CODE though alterred slightly for use in this program (Comments are also not mine)
        /// Source: http://www.codeproject.com/Articles/317974/KinectDepthSmoothing
        /// Updated: 11/14/2014
        /// </summary>
        /// <param name="depthArray">depth frame data to smooth</param>
        /// <returns>smoothed frame data</returns>
        private ushort[] SmoothDepthFrame2(ushort[] depthArray)
        {
            ushort[] smoothDepthArray = new ushort[depthArray.Length];

            // We will be using these numbers for constraints on indexes
            int widthBound = depthWidth - 1;
            int heightBound = depthHeight - 1;

            // We process each row in parallel
            Parallel.For(0, depthHeight, depthArrayRowIndex =>
            {
                // Process each pixel in the row
                for (int depthArrayColumnIndex = 0; depthArrayColumnIndex < depthWidth; depthArrayColumnIndex++)
                {
                    var depthIndex = depthArrayColumnIndex + (depthArrayRowIndex * depthWidth);

                    // We are only concerned with eliminating 'white' noise from the data.
                    // We consider any pixel with a depth of 0 as a possible candidate for filtering.
                    if (depthArray[depthIndex] == 0)
                    {
                        // From the depth index, we can determine the X and Y coordinates that the index
                        // will appear in the image. We use this to help us define our filter matrix.
                        int x = depthIndex % depthWidth;
                        int y = (depthIndex - x) / depthWidth;

                        // The filter collection is used to count the frequency of each
                        // depth value in the filter array. This is used later to determine
                        // the statistical mode for possible assignment to the candidate.
                        ushort[,] filterCollection = new ushort[24, 2];

                        // The inner and outer band counts are used later to compare against the threshold 
                        // values set in the UI to identify a positive filter result.
                        int innerBandCount = 0;
                        int outerBandCount = 0;

                        // The following loops will loop through a 5 X 5 matrix of pixels surrounding the 
                        // candidate pixel. This defines 2 distinct 'bands' around the candidate pixel.
                        // If any of the pixels in this matrix are non-0, we will accumulate them and count
                        // how many non-0 pixels are in each band. If the number of non-0 pixels breaks the
                        // threshold in either band, then the average of all non-0 pixels in the matrix is applied
                        // to the candidate pixel.
                        for (int yi = -2; yi < 3; yi++)
                        {
                            for (int xi = -2; xi < 3; xi++)
                            {
                                // yi and xi are modifiers that will be subtracted from and added to the
                                // candidate pixel's x and y coordinates that we calculated earlier. From the
                                // resulting coordinates, we can calculate the index to be addressed for processing.

                                // We do not want to consider the candidate
                                // pixel (xi = 0, yi = 0) in our process at this point.
                                // We already know that it's 0
                                if (xi != 0 || yi != 0)
                                {
                                    // We then create our modified coordinates for each pass
                                    var xSearch = x + xi;
                                    var ySearch = y + yi;

                                    // While the modified coordinates may in fact calculate out to an actual index, it 
                                    // might not be the one we want. Be sure to check
                                    // to make sure that the modified coordinates
                                    // match up with our image bounds.
                                    if (xSearch >= 0 && xSearch <= widthBound &&
                                                 ySearch >= 0 && ySearch <= heightBound)
                                    {
                                        var index = xSearch + (ySearch * depthWidth);
                                        // We only want to look for non-0 values
                                        if (depthArray[index] != 0)
                                        {
                                            // We want to find count the frequency of each depth
                                            for (int i = 0; i < 24; i++)
                                            {
                                                if (filterCollection[i, 0] == depthArray[index])
                                                {
                                                    // When the depth is already in the filter collection
                                                    // we will just increment the frequency.
                                                    filterCollection[i, 1]++;
                                                    break;
                                                }
                                                else if (filterCollection[i, 0] == 0)
                                                {
                                                    // When we encounter a 0 depth in the filter collection
                                                    // this means we have reached the end of values already counted.
                                                    // We will then add the new depth and start it's frequency at 1.
                                                    filterCollection[i, 0] = depthArray[index];
                                                    filterCollection[i, 1]++;
                                                    break;
                                                }
                                            }

                                            // We will then determine which band the non-0 pixel
                                            // was found in, and increment the band counters.
                                            if (yi != 2 && yi != -2 && xi != 2 && xi != -2)
                                                innerBandCount++;
                                            else
                                                outerBandCount++;
                                        }
                                    }
                                }
                            }
                        }

                        // Once we have determined our inner and outer band non-zero counts, and 
                        // accumulated all of those values, we can compare it against the threshold
                        // to determine if our candidate pixel will be changed to the
                        // statistical mode of the non-zero surrounding pixels.
                        if (innerBandCount >= 2 || outerBandCount >= 3)
                        {
                            ushort frequency = 0;
                            ushort depth = 0;
                            // This loop will determine the statistical mode
                            // of the surrounding pixels for assignment to
                            // the candidate.
                            for (int i = 0; i < 24; i++)
                            {
                                // This means we have reached the end of our
                                // frequency distribution and can break out of the
                                // loop to save time.
                                if (filterCollection[i, 0] == 0)
                                    break;
                                if (filterCollection[i, 1] > frequency)
                                {
                                    depth = filterCollection[i, 0];
                                    frequency = filterCollection[i, 1];
                                }
                            }

                            smoothDepthArray[depthIndex] = depth;
                        }
                    }
                    else
                    {
                        // If the pixel is not zero, we will keep the original depth.
                        smoothDepthArray[depthIndex] = depthArray[depthIndex];
                    }
                }
            });
            return smoothDepthArray;
        }

        private void WorkerThreadProc()
        {
            WaitHandle[] events = new WaitHandle[2] { this.workerThreadStopEvent, this.depthReadyEvent };
            while (true)
            {
                int index = WaitHandle.WaitAny(events);
                if (0 == index)
                {
                    // Stop event has been set. Exit thread
                    break;
                }
                // Reset depth ready event
                this.depthReadyEvent.Reset();
                this.colorReadyEvent.Reset();
            }
        }

        /// <summary>
        /// Puts color on top of the depth pixels, needs an array of equal size to the depth array size
        /// </summary>
        private unsafe void MapColorToDepth()
        {
            colorCoordinates = new ColorSpacePoint[depthWidth * depthHeight];
            
            mapper.MapDepthFrameToColorSpace(depthImagePixels, colorCoordinates);
            fixed (byte* ptrColorPixels = this.colorImagePixels)
            {
                int* rawColorPixels = (int*)ptrColorPixels;
                for (int i = 0; i < depthHeight; i++)
                {
                    for (int j = 0; j < depthWidth; j++)
                    {
                        ColorSpacePoint pt = colorCoordinates[i * depthWidth + j];
                        pt.X = (int)Math.Floor(pt.X + 0.5);
                        pt.Y = (int)Math.Floor(pt.Y + 0.5);
                        if (pt.Y >= 0 && pt.Y < colorHeight && pt.X >= 0 && pt.X < colorWidth)
                        {
                            alignedColorImagePixels[i * depthWidth + j] = rawColorPixels[(int)(pt.Y * colorWidth + pt.X)];
                        }
                    }
                }
            }
        }
    }
}
