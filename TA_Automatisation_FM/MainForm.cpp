//---------------------------------------------------------------------------

#include <fmx.h>
#pragma hdrstop

#include "MainForm.h"
//---------------------------------------------------------------------------
#pragma package(smart_init)
#pragma resource "*.fmx"
TForm1 *Form1;
//---------------------------------------------------------------------------
__fastcall TForm1::TForm1(TComponent* Owner)
	: TForm(Owner)
{
	cncManager.delegate = this;
    paused = false;

	TPoint3D p1, p2;
	p1.X = 0;
	p1.Y = 0;
	p1.Z = 0;
	p2.X = 5;
	p2.Y = 5;
	p2.Z = 0;

  /*	TDummy *dum = objectGenerator.getWireForLine(&p1, &p2, Viewport3D);
	ObjectDummy->AddObject(dum);
	p1.X = 3;
	p1.Y = 7;
	p1.Z = 2;
	dum = objectGenerator.getWireForLine(&p2, &p1, Viewport3D);
	ObjectDummy->AddObject(dum);  */
}
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
//-----------------------CNCManagerDelegate

void TForm1::cncPosition(float x, float y, float z) {
   //	int h = DrawSpace->Height;
   //	int w = DrawSpace->Width;
   //	DrawSpace->Canvas->LineTo(w/2 + x/4, h/2-y/4);
}

void TForm1::cncAsksForCommand() {
	this->sendNextString();
   // LogMemo->Lines->Add(u"Asked for command");
}

void TForm1::cncEcho(const char *str) {
	UnicodeString s = str;
	LogMemo->Lines->Add(s);
    LogMemo->GoToTextEnd();
}
void __fastcall TForm1::Timer1Timer(TObject *Sender)
{
	if(!paused)
		 cncManager.checkSerial();
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
           // this->showGcodeInViewport3D();
		}
}
//---------------------------------------------------------------------------

void __fastcall TForm1::SendCommandButtonClick(TObject *Sender)
{
	if(CommandEdit->Text.Length()) {
		cncManager.sendString(CommandEdit->Text+"\n");
	}
}
//---------------------------------------------------------------------------

void __fastcall TForm1::GCodeListBoxMouseDown(TObject *Sender, TMouseButton Button, TShiftState Shift,
          float X, float Y)
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
			ConnectionLabel->Text = u"Connected";
		}
		else
			ConnectionLabel->Text = u"No connection";
	}

}
//---------------------------------------------------------------------------

void __fastcall TForm1::DisconnectButtonClick(TObject *Sender)
{
	cncManager.disconnect();
	ConnectionLabel->Text = u"No connection";
}
//---------------------------------------------------------------------------

void __fastcall TForm1::StartButtonClick(TObject *Sender)
{
	if(processingGcode || !cncManager.conected()) return;

	this->sendNextString();
}
//---------------------------------------------------------------------------

void TForm1::sendNextString() {
	int sended = 0;
	UnicodeString str;
	while(lineIndex < GCodeListBox->Items->Count && !sended) {
		str = GCodeListBox->Items->operator [](lineIndex);
		if(str.Length() && (str[1]=='G' || str[1]=='g')) {
			cncManager.sendString(str);
            sended = 1;
		}
		lineIndex++;
		GCodeListBox->ItemIndex = lineIndex;
	}
}

//---------------------------------------------------------------------------
void TForm1::showGcodeInViewport3D() {
	TDummy *dum;
    ObjectDummy->DeleteChildren();
	for(int i=0; i<GCodeListBox->Items->Count; i++) {
		dum = objectGenerator.getObjectForGCodeString(GCodeListBox->Items->operator [](i), Viewport3D, this->LightMaterialSource1);
		if(dum)
			ObjectDummy->AddObject(dum);
	}
    this->FloatAnimation1->Start();
}





void __fastcall TForm1::PauseButtonClick(TObject *Sender)
{
	paused = !paused;
}
//---------------------------------------------------------------------------

