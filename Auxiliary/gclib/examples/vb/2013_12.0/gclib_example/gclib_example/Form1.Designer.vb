<Global.Microsoft.VisualBasic.CompilerServices.DesignerGenerated()> _
Partial Class MainForm
    Inherits System.Windows.Forms.Form

    'Form overrides dispose to clean up the component list.
    <System.Diagnostics.DebuggerNonUserCode()> _
    Protected Overrides Sub Dispose(ByVal disposing As Boolean)
        Try
            If disposing AndAlso components IsNot Nothing Then
                components.Dispose()
            End If
        Finally
            MyBase.Dispose(disposing)
        End Try
    End Sub

    'Required by the Windows Form Designer
    Private components As System.ComponentModel.IContainer

    'NOTE: The following procedure is required by the Windows Form Designer
    'It can be modified using the Windows Form Designer.  
    'Do not modify it using the code editor.
    <System.Diagnostics.DebuggerStepThrough()> _
    Private Sub InitializeComponent()
        Dim resources As System.ComponentModel.ComponentResourceManager = New System.ComponentModel.ComponentResourceManager(GetType(MainForm))
        Me.MainToolStrip = New System.Windows.Forms.ToolStrip()
        Me.ToolStripLabel1 = New System.Windows.Forms.ToolStripLabel()
        Me.AddressTextBox = New System.Windows.Forms.ToolStripTextBox()
        Me.GoButton = New System.Windows.Forms.ToolStripButton()
        Me.ToolStripSeparator1 = New System.Windows.Forms.ToolStripSeparator()
        Me.HelpLabel = New System.Windows.Forms.ToolStripLabel()
        Me.Output = New System.Windows.Forms.RichTextBox()
        Me.GclibBackgroundWorker = New System.ComponentModel.BackgroundWorker()
        Me.MainToolStrip.SuspendLayout()
        Me.SuspendLayout()
        '
        'MainToolStrip
        '
        Me.MainToolStrip.GripStyle = System.Windows.Forms.ToolStripGripStyle.Hidden
        Me.MainToolStrip.Items.AddRange(New System.Windows.Forms.ToolStripItem() {Me.ToolStripLabel1, Me.AddressTextBox, Me.GoButton, Me.ToolStripSeparator1, Me.HelpLabel})
        Me.MainToolStrip.Location = New System.Drawing.Point(0, 0)
        Me.MainToolStrip.Name = "MainToolStrip"
        Me.MainToolStrip.Size = New System.Drawing.Size(742, 25)
        Me.MainToolStrip.TabIndex = 0
        Me.MainToolStrip.Text = "ToolStrip1"
        '
        'ToolStripLabel1
        '
        Me.ToolStripLabel1.Name = "ToolStripLabel1"
        Me.ToolStripLabel1.Size = New System.Drawing.Size(94, 22)
        Me.ToolStripLabel1.Text = "GOpen() Address:"
        '
        'AddressTextBox
        '
        Me.AddressTextBox.Name = "AddressTextBox"
        Me.AddressTextBox.Size = New System.Drawing.Size(250, 25)
        Me.AddressTextBox.Text = "192.168.0.42 --subscribe ALL"
        '
        'GoButton
        '
        Me.GoButton.DisplayStyle = System.Windows.Forms.ToolStripItemDisplayStyle.Text
        Me.GoButton.Image = CType(resources.GetObject("GoButton.Image"), System.Drawing.Image)
        Me.GoButton.ImageTransparentColor = System.Drawing.Color.Magenta
        Me.GoButton.Name = "GoButton"
        Me.GoButton.Size = New System.Drawing.Size(24, 22)
        Me.GoButton.Text = "Go"
        '
        'ToolStripSeparator1
        '
        Me.ToolStripSeparator1.Name = "ToolStripSeparator1"
        Me.ToolStripSeparator1.Size = New System.Drawing.Size(6, 25)
        '
        'HelpLabel
        '
        Me.HelpLabel.IsLink = True
        Me.HelpLabel.Name = "HelpLabel"
        Me.HelpLabel.Size = New System.Drawing.Size(28, 22)
        Me.HelpLabel.Text = "Help"
        '
        'Output
        '
        Me.Output.BackColor = System.Drawing.SystemColors.Window
        Me.Output.Dock = System.Windows.Forms.DockStyle.Fill
        Me.Output.Font = New System.Drawing.Font("Consolas", 9.75!, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.Output.Location = New System.Drawing.Point(0, 25)
        Me.Output.Name = "Output"
        Me.Output.ReadOnly = True
        Me.Output.Size = New System.Drawing.Size(742, 548)
        Me.Output.TabIndex = 1
        Me.Output.Text = ""
        '
        'GclibBackgroundWorker
        '
        '
        'MainForm
        '
        Me.AutoScaleDimensions = New System.Drawing.SizeF(6.0!, 13.0!)
        Me.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font
        Me.ClientSize = New System.Drawing.Size(742, 573)
        Me.Controls.Add(Me.Output)
        Me.Controls.Add(Me.MainToolStrip)
        Me.Name = "MainForm"
        Me.Text = "Visual Basic gclib Example"
        Me.MainToolStrip.ResumeLayout(False)
        Me.MainToolStrip.PerformLayout()
        Me.ResumeLayout(False)
        Me.PerformLayout()

    End Sub
    Friend WithEvents MainToolStrip As System.Windows.Forms.ToolStrip
    Friend WithEvents Output As System.Windows.Forms.RichTextBox
    Friend WithEvents HelpLabel As System.Windows.Forms.ToolStripLabel
    Friend WithEvents AddressTextBox As System.Windows.Forms.ToolStripTextBox
    Friend WithEvents GoButton As System.Windows.Forms.ToolStripButton
    Friend WithEvents ToolStripLabel1 As System.Windows.Forms.ToolStripLabel
    Friend WithEvents ToolStripSeparator1 As System.Windows.Forms.ToolStripSeparator
    Friend WithEvents GclibBackgroundWorker As System.ComponentModel.BackgroundWorker

End Class
