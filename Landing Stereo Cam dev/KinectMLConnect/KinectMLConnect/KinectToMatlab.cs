using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using Microsoft.Kinect;


namespace KinectMLConnect
{
    public partial class KinectToMatlab : Form
    {
        #region Members
        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Reader for depth frames
        /// </summary>
        private DepthFrameReader depthFrameReader = null;

        /// <summary>
        /// Description of the data contained in the depth frame
        /// </summary>
        private FrameDescription depthFrameDescription = null;

        /// <summary>
        /// Intermediate storage for frame data
        /// </summary>
        public ushort[] depthFrameData = null;

        /// <summary>
        /// Reader for IR frames
        /// </summary>
        private InfraredFrameReader IRFrameReader = null;

        /// <summary>
        /// Description of the data contained in the depth frame
        /// </summary>
        private FrameDescription IRFrameDescription = null;

        /// <summary>
        /// Intermediate storage for frame data converted to color
        /// </summary>
        public ushort[] IRFrameData = null;


        /// <summary>
        /// Current status text to display
        /// </summary>
        private string textBoxText = null;

        /// <summary>
        /// Keep track of the frames
        /// </summary>
        private int frameCount;

        /// <summary>
        /// writes to output files
        /// </summary>
        private MATWriter matfw = null;

        /// <summary>
        /// String path to save the framefiles
        /// </summary>
        private string filePath = null;

        /// <summary>
        /// Timing of the frames
        /// </summary>
        private ushort[] timing = null;

        /// <summary>
        /// Control which type of data to recieve
        /// </summary>
        private string extractType = null;

        #endregion

        /// <summary>
        /// Calling the constructor
        /// </summary>
        public KinectToMatlab()
        {
            // get the kinectSensor object
            this.kinectSensor = KinectSensor.GetDefault();

            // open the reader for the depth frames
            this.depthFrameReader = this.kinectSensor.DepthFrameSource.OpenReader();

            // Open IR reader for IR frames
            this.IRFrameReader = this.kinectSensor.InfraredFrameSource.OpenReader();

            // wire handler for frame arrival
            this.IRFrameReader.FrameArrived += this.IR_Reader_FrameArrived;
            this.depthFrameReader.FrameArrived += this.Depth_Reader_FrameArrived;

            // get FrameDescription from DepthFrameSource
            this.depthFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // get FrameDescription from InfraredFrameSOurce
            this.IRFrameDescription = this.kinectSensor.InfraredFrameSource.FrameDescription;

            // allocate space to put the pixels being received and converted
            this.depthFrameData = new ushort[this.depthFrameDescription.Width * this.depthFrameDescription.Height];

            // allocate space for IR frame
            this.IRFrameData = new ushort[this.IRFrameDescription.Width * this.IRFrameDescription.Height];

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // initialize the components (controls) of the window
            this.InitializeComponent();

            // set system initial status
            this.StatusText = Properties.Resources.InitialStatusText;

            // framecount is zero
            this.frameCount = 0;

            // Create output directory
            System.IO.Directory.CreateDirectory(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments) + "/Kinect to MatLab");

            // Choose options
            this.DepthRadio.CheckedChanged += new EventHandler(Options_CheckedChanged);
            this.IRRadio.CheckedChanged += new EventHandler(Options_CheckedChanged);

            // Run option
            this.extractType = "Depth";

            // Allocate timing table
            this.timing = new ushort[10000];
        }

        /// <summary>
        /// Handles the IR frame data arriving from the sensor
        /// </summary>
        private void IR_Reader_FrameArrived(object sender, InfraredFrameArrivedEventArgs e)
        {
            if (this.StatusText == Properties.Resources.RunningStatusText)
            {
                if (this.extractType == "IR")
                {
                    using (InfraredFrame IRFrame = e.FrameReference.AcquireFrame())
                    {
                        if (IRFrame != null)
                        {
                            // the fastest way to process the body index data is to directly access 
                            // the underlying buffer
                            using (Microsoft.Kinect.KinectBuffer IRbuffer = IRFrame.LockImageBuffer())
                            {

                                {
                                    IRFrame.CopyFrameDataToArray(IRFrameData);
                                    this.frameCount++;
                                    this.timing[frameCount] = (ushort)IRFrame.RelativeTime.Milliseconds;
                                    filePath = Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments) + "/Kinect to MatLab" + "/IRframe" + frameCount.ToString() + ".MAT";
                                    this.matfw = new MATWriter("IRmat", filePath, IRFrameData, IRFrame.FrameDescription.Height, IRFrame.FrameDescription.Width);

                                }
                            }
                        }
                    }
                }
            }
            else
            {
                SaveParamsToFile(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments) + "/Kinect to MatLab" + "/Intrinsic parameters.txt");
                this.StatusText = Properties.Resources.SensorIsAvailableStatusText;
            }
        }

        /// <summary>
        /// Chnage options based on radio buttons
        /// </summary>
        private void Options_CheckedChanged(object sender, EventArgs e)
        {
            RadioButton radioButton = sender as RadioButton;

            if (DepthRadio.Checked)
            {
                this.extractType = "Depth";
                this.IRRadio.Checked = false;
            }
            else if (IRRadio.Checked)
            {
                this.extractType = "IR";
                this.DepthRadio.Checked = false;
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.depthFrameReader != null)
            {
                // DepthFrameReader is IDisposable
                this.depthFrameReader.Dispose();
                this.depthFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

        /// <summary>
        /// Handles the depth frame data arriving from the sensor
        /// </summary>
        private void Depth_Reader_FrameArrived(object sender, DepthFrameArrivedEventArgs e)
        {
            if (this.StatusText == Properties.Resources.RunningStatusText)
            {
                if(this.extractType == "Depth")
                {
                    using (DepthFrame depthFrame = e.FrameReference.AcquireFrame())
                    {
                        if (depthFrame != null)
                        {
                            // the fastest way to process the body index data is to directly access 
                            // the underlying buffer
                            using (Microsoft.Kinect.KinectBuffer depthBuffer = depthFrame.LockImageBuffer())
                            {
                            
                                {
                                    depthFrame.CopyFrameDataToArray(depthFrameData);
                                    this.frameCount++;
                                    this.timing[frameCount] = (ushort)depthFrame.RelativeTime.Milliseconds;
                                    filePath = Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments) + "/Kinect to MatLab" + "/Depthframe" + frameCount.ToString() + ".MAT";
                                    this.matfw = new MATWriter( "depthmat",filePath, depthFrameData, depthFrame.FrameDescription.Height, depthFrame.FrameDescription.Width);
                                
                                }
                            }
                        }
                    }
                }
            }
            else
            {
                SaveParamsToFile(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments) + "/Kinect to MatLab" + "/Intrinsic parameters.txt");
                this.StatusText = Properties.Resources.SensorIsAvailableStatusText;
            }
        }

        /// <summary>
        /// Actions on startup
        /// </summary>
        private void Form1_Load(object sender, EventArgs e)
        {
            StopButton.Enabled = false;
            DepthRadio.Checked = true;
        }

        /// <summary>
        /// Actions on Start button click
        /// </summary>
        private void StartButton_Click(object sender, EventArgs e)
        {
            this.StatusText = Properties.Resources.RunningStatusText;
            StartButton.Enabled = false;
            StopButton.Enabled = true;
        }

        /// <summary>
        /// Actions on Stop button click
        /// </summary>
        private void StopButton_Click(object sender, EventArgs e)
        {
            StopButton.Enabled = false;
            StartButton.Enabled = true;
            this.StatusText = Properties.Resources.StoppedStatusText;
            
            this.matfw = new MATWriter("timing", Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments) + "/Kinect to MatLab" + "/FrameTimings.mat", this.timing, this.timing.Length, 1);
            
            this.frameCount = 0;
            System.Array.Clear(this.timing, 0, this.timing.Length);
        }

        /// <summary>
        /// Listener for sensor availability
        /// </summary>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            if(e.IsAvailable == true)
            {
                this.StatusText = Properties.Resources.SensorIsAvailableStatusText;
            }
            else
            {
                this.StatusText = Properties.Resources.NoSensorStatusText;
            }
        }

        /// <summary>
        /// Listener for status changes (updates the status text)
        /// </summary>
        public string StatusText
        {
            get { return textBoxText; }
            set
            {
                textBoxText = value;
                {
                    this.StatusBox.Text = textBoxText;
                }
            }
        }

        /// <summary>
        /// Extracts the intrinsic parameters of the camera, to enable 3D point clouds
        /// </summary>
        private void SaveParamsToFile(string filePath)
        {
            // Get all the intrinsic parameters
            float Hfov = this.kinectSensor.DepthFrameSource.FrameDescription.HorizontalFieldOfView;
            float Vfov = this.kinectSensor.DepthFrameSource.FrameDescription.VerticalFieldOfView;
            float Dfov = this.kinectSensor.DepthFrameSource.FrameDescription.DiagonalFieldOfView;
            CameraIntrinsics camIntrinsics = this.kinectSensor.CoordinateMapper.GetDepthCameraIntrinsics();
            float Vfl = camIntrinsics.FocalLengthY;
            float Hfl = camIntrinsics.FocalLengthX;
            float Hpp = camIntrinsics.PrincipalPointX;
            float Vpp = camIntrinsics.PrincipalPointY;
            float Rdist = camIntrinsics.RadialDistortionSecondOrder;

            System.IO.File.WriteAllText(filePath,
                "Horizontal Field of View is: " + Hfov.ToString() + "\r\n" +
                "Vertical Field of View is: " + Vfov.ToString() + "\r\n" +
                "Diagonal Field of View is: " + Dfov.ToString() + "\r\n" +
                "Horizontal focal length is: " + Hfl.ToString() + "\r\n" + 
                "Vertical focal length is: " + Vfl.ToString() + "\r\n" +
                "Horizontal principle point is: " + Hpp.ToString() + "\r\n" +
                "Vertical Principal point is: " + Vpp.ToString() + "\r\n" +
                "Radial distortion Second order term is: "+ Rdist.ToString() + "\r\n");



        }

    }
}


