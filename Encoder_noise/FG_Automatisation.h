//---------------------------------------------------------------------------

#ifndef FG_AutomatisationH
#define FG_AutomatisationH
//---------------------------------------------------------------------------
#include <System.Classes.hpp>
#include <Vcl.Controls.hpp>
#include <Vcl.StdCtrls.hpp>
#include <Vcl.Forms.hpp>
#include <Vcl.ExtCtrls.hpp>
//---------------------------------------------------------------------------
#include "Serial.h"
#include <Vcl.ComCtrls.hpp>

class TForm1 : public TForm
{
__published:	// IDE-managed Components
	TListBox *ListBox1;
	TButton *ConnectButton;
	TButton *DisconnectButton;
	TPanel *DrawPanel;
	TImage *Image1;
	TTimer *Timer1;
	TLabel *StatusLabel;
	TLabel *OutputLabel1;
	TLabel *OutputLabel2;
	TLabel *OutputLabel3;
	TLabel *OutputLabel4;
	TLabel *OutputLabel5;
	TLabel *OutputLabel6;
	TLabel *OutputLabel7;
	TLabel *OutputLabel8;
	void __fastcall Timer1Timer(TObject *Sender);
	void __fastcall DisconnectButtonClick(TObject *Sender);
	void __fastcall ConnectButtonClick(TObject *Sender);

private:	// User declarations
public:		// User declarations
	__fastcall TForm1(TComponent* Owner);
	void drawCanvasNet();

	CSerial ser;
	char readedData[10000];
};
//---------------------------------------------------------------------------
extern PACKAGE TForm1 *Form1;
//---------------------------------------------------------------------------
#endif
