namespace gclib_example
{
    partial class MainForm
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
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(MainForm));
            this.MainToolStrip = new System.Windows.Forms.ToolStrip();
            this.Output = new System.Windows.Forms.RichTextBox();
            this.toolStripLabel1 = new System.Windows.Forms.ToolStripLabel();
            this.AddressTextBox = new System.Windows.Forms.ToolStripTextBox();
            this.GoButton = new System.Windows.Forms.ToolStripButton();
            this.toolStripSeparator1 = new System.Windows.Forms.ToolStripSeparator();
            this.HelpLabel = new System.Windows.Forms.ToolStripLabel();
            this.GclibBackgroundWorker = new System.ComponentModel.BackgroundWorker();
            this.MainToolStrip.SuspendLayout();
            this.SuspendLayout();
            // 
            // MainToolStrip
            // 
            this.MainToolStrip.GripStyle = System.Windows.Forms.ToolStripGripStyle.Hidden;
            this.MainToolStrip.Items.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.toolStripLabel1,
            this.AddressTextBox,
            this.GoButton,
            this.toolStripSeparator1,
            this.HelpLabel});
            this.MainToolStrip.Location = new System.Drawing.Point(0, 0);
            this.MainToolStrip.Name = "MainToolStrip";
            this.MainToolStrip.Size = new System.Drawing.Size(742, 25);
            this.MainToolStrip.TabIndex = 0;
            this.MainToolStrip.Text = "toolStrip1";
            // 
            // Output
            // 
            this.Output.BackColor = System.Drawing.SystemColors.Window;
            this.Output.Dock = System.Windows.Forms.DockStyle.Fill;
            this.Output.Font = new System.Drawing.Font("Consolas", 9F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.Output.Location = new System.Drawing.Point(0, 25);
            this.Output.Name = "Output";
            this.Output.ReadOnly = true;
            this.Output.Size = new System.Drawing.Size(742, 548);
            this.Output.TabIndex = 1;
            this.Output.Text = "";
            // 
            // toolStripLabel1
            // 
            this.toolStripLabel1.Name = "toolStripLabel1";
            this.toolStripLabel1.Size = new System.Drawing.Size(94, 22);
            this.toolStripLabel1.Text = "GOpen() Address:";
            // 
            // AddressTextBox
            // 
            this.AddressTextBox.Name = "AddressTextBox";
            this.AddressTextBox.Size = new System.Drawing.Size(250, 25);
            this.AddressTextBox.Text = "192.168.0.42 --subscribe ALL";
            // 
            // GoButton
            // 
            this.GoButton.DisplayStyle = System.Windows.Forms.ToolStripItemDisplayStyle.Text;
            this.GoButton.Image = ((System.Drawing.Image)(resources.GetObject("GoButton.Image")));
            this.GoButton.ImageTransparentColor = System.Drawing.Color.Magenta;
            this.GoButton.Name = "GoButton";
            this.GoButton.Size = new System.Drawing.Size(24, 22);
            this.GoButton.Text = "Go";
            this.GoButton.Click += new System.EventHandler(this.GoButton_Click);
            // 
            // toolStripSeparator1
            // 
            this.toolStripSeparator1.Name = "toolStripSeparator1";
            this.toolStripSeparator1.Size = new System.Drawing.Size(6, 25);
            // 
            // HelpLabel
            // 
            this.HelpLabel.IsLink = true;
            this.HelpLabel.Name = "HelpLabel";
            this.HelpLabel.Size = new System.Drawing.Size(28, 22);
            this.HelpLabel.Text = "Help";
            this.HelpLabel.Click += new System.EventHandler(this.HelpLabel_Click);
            // 
            // GclibBackgroundWorker
            // 
            this.GclibBackgroundWorker.DoWork += new System.ComponentModel.DoWorkEventHandler(this.GclibBackgroundWorker_DoWork);
            this.GclibBackgroundWorker.RunWorkerCompleted += new System.ComponentModel.RunWorkerCompletedEventHandler(this.GclibBackgroundWorker_RunWorkerCompleted);
            // 
            // MainForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(742, 573);
            this.Controls.Add(this.Output);
            this.Controls.Add(this.MainToolStrip);
            this.Name = "MainForm";
            this.Text = "C# gclib Example";
            this.Load += new System.EventHandler(this.MainForm_Load);
            this.MainToolStrip.ResumeLayout(false);
            this.MainToolStrip.PerformLayout();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.ToolStrip MainToolStrip;
        private System.Windows.Forms.RichTextBox Output;
        private System.Windows.Forms.ToolStripLabel toolStripLabel1;
        private System.Windows.Forms.ToolStripTextBox AddressTextBox;
        private System.Windows.Forms.ToolStripButton GoButton;
        private System.Windows.Forms.ToolStripSeparator toolStripSeparator1;
        private System.Windows.Forms.ToolStripLabel HelpLabel;
        private System.ComponentModel.BackgroundWorker GclibBackgroundWorker;
    }
}

