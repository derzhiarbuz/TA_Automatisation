//---------------------------------------------------------------------------

#ifndef TA1PanelH
#define TA1PanelH
//---------------------------------------------------------------------------
#include <System.Classes.hpp>
#include <Vcl.Controls.hpp>
#include <Vcl.StdCtrls.hpp>
#include <Vcl.Forms.hpp>
#include <Vcl.ExtCtrls.hpp>
#include <Vcl.Dialogs.hpp>
#include <Vcl.Grids.hpp>
//---------------------------------------------------------------------------
#include "CNCManager.h"

class TForm1 : public TForm, public CNCManagerDelegate
{
__published:	// IDE-managed Components
	TComboBox *PortCombo;
	TPanel *Panel1;
	TImage *DrawSpace;
	TButton *StartButton;
	TButton *LoadButton;
	TButton *PauseButton;
	TButton *StopButton;
	TOpenDialog *OpenDialog1;
	TListBox *GCodeListBox;
	TButton *SendCommandButton;
	TEdit *CommandEdit;
	TButton *ConnectButton;
	TLabel *ConnectionLabel;
	TButton *DisconnectButton;
	TMemo *LogMemo;
	TLabel *LogLabel;
	TTimer *Timer1;
	void __fastcall LoadButtonClick(TObject *Sender);
	void __fastcall SendCommandButtonClick(TObject *Sender);
	void __fastcall GCodeListBoxMouseDown(TObject *Sender, TMouseButton Button, TShiftState Shift,
          int X, int Y);
	void __fastcall GCodeListBoxClick(TObject *Sender);
	void __fastcall ConnectButtonClick(TObject *Sender);
	void __fastcall DisconnectButtonClick(TObject *Sender);
	void __fastcall StartButtonClick(TObject *Sender);
	void __fastcall Timer1Timer(TObject *Sender);
private:	// User declarations
	CNCManager cncManager;
	int lineIndex;
	bool processingGcode;

	void sendNextString();

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
