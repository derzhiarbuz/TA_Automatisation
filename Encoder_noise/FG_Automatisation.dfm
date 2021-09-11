object Form1: TForm1
  Left = 0
  Top = 0
  Caption = 'Form1'
  ClientHeight = 534
  ClientWidth = 784
  Color = clBtnFace
  Font.Charset = DEFAULT_CHARSET
  Font.Color = clWindowText
  Font.Height = -11
  Font.Name = 'Tahoma'
  Font.Style = []
  OldCreateOrder = False
  PixelsPerInch = 96
  TextHeight = 13
  object StatusLabel: TLabel
    Left = 701
    Top = 87
    Width = 3
    Height = 13
  end
  object OutputLabel1: TLabel
    Left = 8
    Top = 415
    Width = 329
    Height = 25
    AutoSize = False
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clRed
    Font.Height = -21
    Font.Name = 'Tahoma'
    Font.Style = []
    ParentFont = False
  end
  object OutputLabel2: TLabel
    Left = 8
    Top = 445
    Width = 329
    Height = 25
    AutoSize = False
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clMaroon
    Font.Height = -21
    Font.Name = 'Tahoma'
    Font.Style = []
    ParentFont = False
  end
  object OutputLabel3: TLabel
    Left = 8
    Top = 505
    Width = 329
    Height = 25
    AutoSize = False
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clLime
    Font.Height = -21
    Font.Name = 'Tahoma'
    Font.Style = []
    ParentFont = False
  end
  object OutputLabel4: TLabel
    Left = 366
    Top = 415
    Width = 329
    Height = 25
    AutoSize = False
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clBlue
    Font.Height = -21
    Font.Name = 'Tahoma'
    Font.Style = []
    ParentFont = False
  end
  object OutputLabel5: TLabel
    Left = 366
    Top = 445
    Width = 329
    Height = 25
    AutoSize = False
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clNavy
    Font.Height = -21
    Font.Name = 'Tahoma'
    Font.Style = []
    ParentFont = False
  end
  object OutputLabel6: TLabel
    Left = 366
    Top = 505
    Width = 329
    Height = 25
    AutoSize = False
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clGreen
    Font.Height = -21
    Font.Name = 'Tahoma'
    Font.Style = []
    ParentFont = False
  end
  object OutputLabel7: TLabel
    Left = 8
    Top = 474
    Width = 330
    Height = 25
    AutoSize = False
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clMaroon
    Font.Height = -21
    Font.Name = 'Tahoma'
    Font.Style = []
    ParentFont = False
    Visible = False
  end
  object OutputLabel8: TLabel
    Left = 366
    Top = 475
    Width = 330
    Height = 25
    AutoSize = False
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clNavy
    Font.Height = -21
    Font.Name = 'Tahoma'
    Font.Style = []
    ParentFont = False
    Visible = False
  end
  object ListBox1: TListBox
    Left = 701
    Top = 8
    Width = 75
    Height = 73
    ItemHeight = 13
    Items.Strings = (
      'COM 1'
      'COM 2'
      'COM 3'
      'COM 4'
      'COM 5')
    TabOrder = 0
  end
  object ConnectButton: TButton
    Left = 701
    Top = 103
    Width = 75
    Height = 25
    Caption = 'Connect'
    TabOrder = 1
    OnClick = ConnectButtonClick
  end
  object DisconnectButton: TButton
    Left = 701
    Top = 133
    Width = 75
    Height = 25
    Caption = 'Disconnect'
    TabOrder = 2
    OnClick = DisconnectButtonClick
  end
  object DrawPanel: TPanel
    Left = 8
    Top = 8
    Width = 687
    Height = 393
    BevelInner = bvLowered
    BevelOuter = bvNone
    Color = clWhite
    ParentBackground = False
    TabOrder = 3
    object Image1: TImage
      Left = 0
      Top = 0
      Width = 687
      Height = 393
    end
  end
  object Timer1: TTimer
    OnTimer = Timer1Timer
    Left = 720
    Top = 480
  end
end
