﻿namespace VolumeScanner
{
    partial class Display
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
            this.weightSlider = new System.Windows.Forms.TrackBar();
            this.label1 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.wgtValueLbl = new System.Windows.Forms.Label();
            this.absValueLbl = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.threshSlider = new System.Windows.Forms.TrackBar();
            this.label5 = new System.Windows.Forms.Label();
            this.smoothButton = new System.Windows.Forms.Button();
            this.colorButton = new System.Windows.Forms.Button();
            this.frameSlider = new System.Windows.Forms.TrackBar();
            this.intValueLbl = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.dumpButton = new System.Windows.Forms.Button();
            ((System.ComponentModel.ISupportInitialize)(this.weightSlider)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.threshSlider)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.frameSlider)).BeginInit();
            this.SuspendLayout();
            // 
            // weightSlider
            // 
            this.weightSlider.Location = new System.Drawing.Point(78, 521);
            this.weightSlider.Maximum = 100;
            this.weightSlider.Name = "weightSlider";
            this.weightSlider.Size = new System.Drawing.Size(347, 45);
            this.weightSlider.TabIndex = 0;
            this.weightSlider.TickFrequency = 20;
            this.weightSlider.Value = 85;
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(431, 521);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(36, 13);
            this.label1.TabIndex = 1;
            this.label1.Text = "Depth";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(41, 521);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(31, 13);
            this.label2.TabIndex = 2;
            this.label2.Text = "Color";
            // 
            // wgtValueLbl
            // 
            this.wgtValueLbl.AutoSize = true;
            this.wgtValueLbl.Location = new System.Drawing.Point(234, 505);
            this.wgtValueLbl.Name = "wgtValueLbl";
            this.wgtValueLbl.Size = new System.Drawing.Size(22, 13);
            this.wgtValueLbl.TabIndex = 3;
            this.wgtValueLbl.Text = ".85";
            // 
            // absValueLbl
            // 
            this.absValueLbl.AutoSize = true;
            this.absValueLbl.Location = new System.Drawing.Point(234, 441);
            this.absValueLbl.Name = "absValueLbl";
            this.absValueLbl.Size = new System.Drawing.Size(25, 13);
            this.absValueLbl.TabIndex = 7;
            this.absValueLbl.Text = "150";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(84, 441);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(98, 13);
            this.label4.TabIndex = 6;
            this.label4.Text = "Absolute Threshold";
            // 
            // threshSlider
            // 
            this.threshSlider.Location = new System.Drawing.Point(78, 457);
            this.threshSlider.Maximum = 200;
            this.threshSlider.Name = "threshSlider";
            this.threshSlider.Size = new System.Drawing.Size(347, 45);
            this.threshSlider.TabIndex = 4;
            this.threshSlider.TickFrequency = 20;
            this.threshSlider.Value = 150;
            this.threshSlider.Scroll += new System.EventHandler(this.threshSlider_Scroll);
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(84, 505);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(55, 13);
            this.label5.TabIndex = 8;
            this.label5.Text = "Weighting";
            // 
            // smoothButton
            // 
            this.smoothButton.Location = new System.Drawing.Point(157, 639);
            this.smoothButton.Name = "smoothButton";
            this.smoothButton.Size = new System.Drawing.Size(119, 31);
            this.smoothButton.TabIndex = 9;
            this.smoothButton.Text = "Enable Smoothing";
            this.smoothButton.UseVisualStyleBackColor = true;
            // 
            // colorButton
            // 
            this.colorButton.Location = new System.Drawing.Point(65, 639);
            this.colorButton.Name = "colorButton";
            this.colorButton.Size = new System.Drawing.Size(77, 31);
            this.colorButton.TabIndex = 10;
            this.colorButton.Text = "Enable Color";
            this.colorButton.UseVisualStyleBackColor = true;
            // 
            // frameSlider
            // 
            this.frameSlider.Location = new System.Drawing.Point(78, 588);
            this.frameSlider.Maximum = 100;
            this.frameSlider.Minimum = 1;
            this.frameSlider.Name = "frameSlider";
            this.frameSlider.Size = new System.Drawing.Size(347, 45);
            this.frameSlider.TabIndex = 11;
            this.frameSlider.TickFrequency = 20;
            this.frameSlider.Value = 1;
            // 
            // intValueLbl
            // 
            this.intValueLbl.AutoSize = true;
            this.intValueLbl.Location = new System.Drawing.Point(234, 569);
            this.intValueLbl.Name = "intValueLbl";
            this.intValueLbl.Size = new System.Drawing.Size(13, 13);
            this.intValueLbl.TabIndex = 12;
            this.intValueLbl.Text = "1";
            this.intValueLbl.Click += new System.EventHandler(this.intValueLbl_Click);
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(84, 569);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(122, 13);
            this.label6.TabIndex = 13;
            this.label6.Text = "Frame Integration Factor";
            // 
            // dumpButton
            // 
            this.dumpButton.Location = new System.Drawing.Point(296, 639);
            this.dumpButton.Name = "dumpButton";
            this.dumpButton.Size = new System.Drawing.Size(142, 31);
            this.dumpButton.TabIndex = 17;
            this.dumpButton.Text = "Dump Depth Frame Data";
            this.dumpButton.UseVisualStyleBackColor = true;
            // 
            // Display
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(509, 691);
            this.Controls.Add(this.dumpButton);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.intValueLbl);
            this.Controls.Add(this.frameSlider);
            this.Controls.Add(this.colorButton);
            this.Controls.Add(this.smoothButton);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.absValueLbl);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.threshSlider);
            this.Controls.Add(this.wgtValueLbl);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.weightSlider);
            this.Name = "Display";
            this.Text = "Display";
            this.Load += new System.EventHandler(this.Display_Load);
            ((System.ComponentModel.ISupportInitialize)(this.weightSlider)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.threshSlider)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.frameSlider)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.TrackBar weightSlider;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label wgtValueLbl;
        private System.Windows.Forms.Label absValueLbl;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.TrackBar threshSlider;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Button smoothButton;
        private System.Windows.Forms.Button colorButton;
        private System.Windows.Forms.TrackBar frameSlider;
        private System.Windows.Forms.Label intValueLbl;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Button dumpButton;

    }
}