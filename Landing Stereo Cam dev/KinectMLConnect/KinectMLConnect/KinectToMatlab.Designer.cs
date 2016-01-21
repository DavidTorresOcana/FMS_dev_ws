namespace KinectMLConnect
{
    partial class KinectToMatlab
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.StartButton = new System.Windows.Forms.Button();
            this.StopButton = new System.Windows.Forms.Button();
            this.StatusBox = new System.Windows.Forms.TextBox();
            this.Options = new System.Windows.Forms.GroupBox();
            this.IRRadio = new System.Windows.Forms.RadioButton();
            this.DepthRadio = new System.Windows.Forms.RadioButton();
            this.Options.SuspendLayout();
            this.SuspendLayout();
            // 
            // StartButton
            // 
            this.StartButton.Location = new System.Drawing.Point(12, 12);
            this.StartButton.Name = "StartButton";
            this.StartButton.Size = new System.Drawing.Size(95, 23);
            this.StartButton.TabIndex = 0;
            this.StartButton.Text = "Start capture";
            this.StartButton.UseVisualStyleBackColor = true;
            this.StartButton.Click += new System.EventHandler(this.StartButton_Click);
            // 
            // StopButton
            // 
            this.StopButton.Location = new System.Drawing.Point(113, 12);
            this.StopButton.Name = "StopButton";
            this.StopButton.Size = new System.Drawing.Size(92, 23);
            this.StopButton.TabIndex = 1;
            this.StopButton.Text = "Stop capture";
            this.StopButton.UseVisualStyleBackColor = true;
            this.StopButton.Click += new System.EventHandler(this.StopButton_Click);
            // 
            // StatusBox
            // 
            this.StatusBox.BackColor = System.Drawing.SystemColors.Info;
            this.StatusBox.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.StatusBox.Location = new System.Drawing.Point(223, 17);
            this.StatusBox.Name = "StatusBox";
            this.StatusBox.ReadOnly = true;
            this.StatusBox.Size = new System.Drawing.Size(218, 13);
            this.StatusBox.TabIndex = 2;
            this.StatusBox.Text = "Ready...";
            // 
            // Options
            // 
            this.Options.Controls.Add(this.IRRadio);
            this.Options.Controls.Add(this.DepthRadio);
            this.Options.Location = new System.Drawing.Point(12, 42);
            this.Options.Name = "Options";
            this.Options.Size = new System.Drawing.Size(200, 67);
            this.Options.TabIndex = 3;
            this.Options.TabStop = false;
            this.Options.Text = "Options";
            // 
            // IRRadio
            // 
            this.IRRadio.AutoSize = true;
            this.IRRadio.Location = new System.Drawing.Point(7, 44);
            this.IRRadio.Name = "IRRadio";
            this.IRRadio.Size = new System.Drawing.Size(72, 17);
            this.IRRadio.TabIndex = 1;
            this.IRRadio.TabStop = true;
            this.IRRadio.Text = "Extract IR";
            this.IRRadio.UseVisualStyleBackColor = true;
            // 
            // DepthRadio
            // 
            this.DepthRadio.AutoSize = true;
            this.DepthRadio.Location = new System.Drawing.Point(7, 20);
            this.DepthRadio.Name = "DepthRadio";
            this.DepthRadio.Size = new System.Drawing.Size(90, 17);
            this.DepthRadio.TabIndex = 0;
            this.DepthRadio.TabStop = true;
            this.DepthRadio.Text = "Extract Depth";
            this.DepthRadio.UseVisualStyleBackColor = true;
            // 
            // KinectToMatlab
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(453, 133);
            this.Controls.Add(this.Options);
            this.Controls.Add(this.StatusBox);
            this.Controls.Add(this.StopButton);
            this.Controls.Add(this.StartButton);
            this.Name = "KinectToMatlab";
            this.Text = "Kinect to MatLab datacapture";
            this.Load += new System.EventHandler(this.Form1_Load);
            this.Options.ResumeLayout(false);
            this.Options.PerformLayout();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Button StartButton;
        private System.Windows.Forms.Button StopButton;
        private System.Windows.Forms.TextBox StatusBox;
        private System.Windows.Forms.GroupBox Options;
        private System.Windows.Forms.RadioButton IRRadio;
        private System.Windows.Forms.RadioButton DepthRadio;
    }
}

