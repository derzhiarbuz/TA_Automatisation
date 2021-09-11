object Form1: TForm1
  Left = 0
  Top = 0
  Caption = 'TADZHIK-1 control panel'
  ClientHeight = 579
  ClientWidth = 908
  Color = clBtnFace
  Font.Charset = DEFAULT_CHARSET
  Font.Color = clWindowText
  Font.Height = -11
  Font.Name = 'Tahoma'
  Font.Style = []
  OldCreateOrder = False
  PixelsPerInch = 96
  TextHeight = 13
  object ConnectionLabel: TLabel
    Left = 567
    Top = 499
    Width = 68
    Height = 13
    Alignment = taRightJustify
    Caption = 'No connection'
  end
  object LogLabel: TLabel
    Left = 8
    Top = 455
    Width = 21
    Height = 13
    Caption = 'Log:'
  end
  object Panel1: TPanel
    Left = 8
    Top = 8
    Width = 553
    Height = 441
    BevelInner = bvLowered
    TabOrder = 1
    object DrawSpace: TImage
      Left = 2
      Top = 2
      Width = 550
      Height = 439
    end
  end
  object PortCombo: TComboBox
    Left = 641
    Top = 496
    Width = 97
    Height = 21
    TabOrder = 0
    Text = 'Printer port'
    Items.Strings = (
      'COM1'#11
      'COM2'
      'COM3'
      'COM4'
      'COM5'
      'COM6')
  end
  object StartButton: TButton
    Left = 663
    Top = 455
    Width = 75
    Height = 25
    Caption = 'Start'
    TabOrder = 2
    OnClick = StartButtonClick
  end
  object LoadButton: TButton
    Left = 567
    Top = 455
    Width = 75
    Height = 25
    Caption = 'Load G-Code'
    TabOrder = 3
    OnClick = LoadButtonClick
  end
  object PauseButton: TButton
    Left = 744
    Top = 455
    Width = 75
    Height = 25
    Caption = 'Pause'
    TabOrder = 4
  end
  object StopButton: TButton
    Left = 825
    Top = 455
    Width = 75
    Height = 25
    Caption = 'Stop'
    TabOrder = 5
  end
  object GCodeListBox: TListBox
    Left = 567
    Top = 8
    Width = 333
    Height = 441
    ExtendedSelect = False
    ItemHeight = 13
    TabOrder = 6
    OnClick = GCodeListBoxClick
    OnMouseDown = GCodeListBoxMouseDown
  end
  object SendCommandButton: TButton
    Left = 825
    Top = 530
    Width = 75
    Height = 25
    Caption = 'Send'
    TabOrder = 7
    OnClick = SendCommandButtonClick
  end
  object CommandEdit: TEdit
    Left = 567
    Top = 532
    Width = 252
    Height = 21
    TabOrder = 8
  end
  object ConnectButton: TButton
    Left = 744
    Top = 494
    Width = 75
    Height = 25
    Caption = 'Connect'
    TabOrder = 9
    OnClick = ConnectButtonClick
  end
  object DisconnectButton: TButton
    Left = 825
    Top = 494
    Width = 75
    Height = 25
    Caption = 'Disconnect'
    TabOrder = 10
    OnClick = DisconnectButtonClick
  end
  object LogMemo: TMemo
    Left = 8
    Top = 472
    Width = 553
    Height = 99
    TabOrder = 11
  end
  object OpenDialog1: TOpenDialog
    FileName = 'C:\Projects\Tadzhik\TA_Automatisation\Tadzhik_control\TA1Panel.h'
    Left = 560
    Top = 552
  end
  object Timer1: TTimer
    Interval = 1
    OnTimer = Timer1Timer
    Left = 600
    Top = 560
  end
end
