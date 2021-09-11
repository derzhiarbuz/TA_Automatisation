//---------------------------------------------------------------------------

#include <vcl.h>
#pragma hdrstop

#include "TA1Panel.h"
#include <iostream.h>
//---------------------------------------------------------------------------
#pragma package(smart_init)
#pragma resource "*.dfm"
TForm1 *Form1;
//---------------------------------------------------------------------------
__fastcall TForm1::TForm1(TComponent* Owner)
	: TForm(Owner)
{
	cncManager.delegate = this;
	DrawSpace->Canvas->MoveTo(DrawSpace->Width/2, DrawSpace->Height/2);
}
//---------------------------------------------------------------------------
void __fastcall TForm1::LoadButtonClick(TObject *Sender)
{
	if(OpenDialog1->Execute())
		if(FileExists(OpenDialog1->FileName)) {
			GCodeListBox->Items->LoadFromFile(OpenDialog1->FileName);
			GCodeListBox->ItemIndex = 0;
			lineIndex = 0;
            processingGcode = false;
        }
}
//---------------------------------------------------------------------------
void __fastcall TForm1::SendCommandButtonClick(TObject *Sender)
{
	if(CommandEdit->Text.Length()) {
		cncManager.sendString(CommandEdit->Text);
    }
}
//---------------------------------------------------------------------------


void __fastcall TForm1::GCodeListBoxMouseDown(TObject *Sender, TMouseButton Button,
          TShiftState Shift, int X, int Y)
{
	GCodeListBox->ItemIndex = lineIndex;
}
//---------------------------------------------------------------------------

void __fastcall TForm1::GCodeListBoxClick(TObject *Sender)
{
    GCodeListBox->ItemIndex = lineIndex;
}
//---------------------------------------------------------------------------

void __fastcall TForm1::ConnectButtonClick(TObject *Sender)
{
	if(PortCombo->ItemIndex >= 0) {
		cncManager.connect(PortCombo->ItemIndex+1, 115200);
		if(cncManager.conected()) {
			ConnectionLabel->Caption = u"Connected";
		}
		else
			ConnectionLabel->Caption = u"No connection";
	}
}
//---------------------------------------------------------------------------

void __fastcall TForm1::DisconnectButtonClick(TObject *Sender)
{
    cncManager.disconnect();
	ConnectionLabel->Caption = u"No connection";
}
//---------------------------------------------------------------------------

void __fastcall TForm1::StartButtonClick(TObject *Sender)
{
	if(processingGcode || !cncManager.conected()) return;

	this->sendNextString();
}
//---------------------------------------------------------------------------

void TForm1::sendNextString() {
    if(lineIndex < GCodeListBox->Items->Count) {
		cncManager.sendString(GCodeListBox->Items->operator [](lineIndex));
		lineIndex++;
        GCodeListBox->ItemIndex = lineIndex;
	}
}

//---------------------------------------------------------------------------
//-----------------------CNCManagerDelegate

void TForm1::cncPosition(float x, float y, float z) {
	int h = DrawSpace->Height;
	int w = DrawSpace->Width;
	DrawSpace->Canvas->LineTo(w/2 + x/4, h/2-y/4);
}

void TForm1::cncAsksForCommand() {
	this->sendNextString();
    //LogMemo->Lines->Add(u"Asked for command");
}

void TForm1::cncEcho(const char *str) {
	UnicodeString s = str;
	LogMemo->Lines->Add(s);
}

//---------------------------------------------------------------------------

void __fastcall TForm1::Timer1Timer(TObject *Sender)
{
    cncManager.checkSerial();
}
//---------------------------------------------------------------------------

