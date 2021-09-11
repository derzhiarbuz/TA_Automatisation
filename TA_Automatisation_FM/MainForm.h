//---------------------------------------------------------------------------

#ifndef MainFormH
#define MainFormH
//---------------------------------------------------------------------------
#include <System.Classes.hpp>
#include <FMX.Controls.hpp>
#include <FMX.Forms.hpp>
#include <FMX.Controls3D.hpp>
#include <FMX.Objects3D.hpp>
#include <FMX.Types.hpp>
#include <FMX.Viewport3D.hpp>
#include <System.Math.Vectors.hpp>
#include <FMX.Controls.Presentation.hpp>
#include <FMX.Memo.hpp>
#include <FMX.ScrollBox.hpp>
#include <FMX.StdCtrls.hpp>
#include <FMX.ComboEdit.hpp>
#include <FMX.Dialogs.hpp>
#include <FMX.Edit.hpp>
#include <FMX.Layouts.hpp>
#include <FMX.ListBox.hpp>
//---------------------------------------------------------------------------

#include "CNCManager.h"
#include "WireFMX3DObjectGenerator.h"
#include <FMX.Ani.hpp>
#include <FMX.Types3D.hpp>
#include <FMX.MaterialSources.hpp>

class TForm1 : public TForm, public CNCManagerDelegate
{
__published:	// IDE-managed Components
	TViewport3D *Viewport3D;
	TMemo *LogMemo;
	TButton *LoadButton;
	TOpenDialog *OpenDialog1;
	TTimer *Timer1;
	TListBox *GCodeListBox;
	TButton *StartButton;
	TButton *PauseButton;
	TButton *StopButton;
	TComboEdit *PortCombo;
	TLabel *ConnectionLabel;
	TButton *ConnectButton;
	TButton *DisconnectButton;
	TEdit *CommandEdit;
	TButton *SendCommandButton;
	TDummy *AnimationDummy;
	TFloatAnimation *FloatAnimation1;
	TDummy *ObjectDummy;
	TLight *Light1;
	TColorMaterialSource *ColorMaterialSource1;
	TLightMaterialSource *LightMaterialSource1;
	void __fastcall Timer1Timer(TObject *Sender);
	void __fastcall LoadButtonClick(TObject *Sender);
	void __fastcall SendCommandButtonClick(TObject *Sender);
	void __fastcall GCodeListBoxMouseDown(TObject *Sender, TMouseButton Button, TShiftState Shift,
          float X, float Y);
	void __fastcall GCodeListBoxClick(TObject *Sender);
	void __fastcall ConnectButtonClick(TObject *Sender);
	void __fastcall DisconnectButtonClick(TObject *Sender);
	void __fastcall StartButtonClick(TObject *Sender);
	void __fastcall PauseButtonClick(TObject *Sender);
private:	// User declarations
	CNCManager cncManager;
	WireFMX3DObjectGenerator objectGenerator;
	int lineIndex;
	bool processingGcode;
	bool paused;

	void sendNextString();
	void showGcodeInViewport3D();
public:		// User declarations
	__fastcall TForm1(TComponent* Owner);

	//CNCManagerDelegate
	void cncPosition(float x, float y, float z);
	void cncAsksForCommand();
	void cncEcho(const char *str);
};
//---------------------------------------------------------------------------
extern PACKAGE TForm1 *Form1;
//---------------------------------------------------------------------------
#endif
