//---------------------------------------------------------------------------

#include <vcl.h>
#pragma hdrstop

#include <string>
#include <stdlib.h>
#include "FG_Automatisation.h"
//---------------------------------------------------------------------------
#pragma package(smart_init)
#pragma resource "*.dfm"
const float MASS_DIV=20.0;
const float DISCHARGE_DIV=100.0;
const float MILLIS_DIV=1000.0;

TForm1 *Form1;
//---------------------------------------------------------------------------
__fastcall TForm1::TForm1(TComponent* Owner)
	: TForm(Owner)
{
	drawCanvasNet();
	Image1->Canvas->MoveTo(0, Image1->Height);
}

//---------------------------------------------------------------------------

void TForm1::drawCanvasNet() {
	Image1->Canvas->Rectangle(0,0,Image1->Width,Image1->Height);
	int h;

	Image1->Canvas->Pen->Color = clSilver;
    for(h=Image1->Height; h>0; h-=(Image1->Height/DISCHARGE_DIV)) {
		Image1->Canvas->MoveTo(0, h);
		Image1->Canvas->LineTo(Image1->Width, h);
	}
	Image1->Canvas->Pen->Color = clGray;
	for(h=Image1->Height; h>0; h-=(Image1->Height/MASS_DIV)) {
		Image1->Canvas->MoveTo(0, h);
		Image1->Canvas->LineTo(Image1->Width, h);
	}
	Image1->Canvas->Pen->Color = clSilver;
	for(h=0; h<Image1->Width; h+=(Image1->Width/MILLIS_DIV)*60) {
		Image1->Canvas->MoveTo(h, 0);
		Image1->Canvas->LineTo(h, Image1->Height);
	}
}


//---------------------------------------------------------------------------
void __fastcall TForm1::Timer1Timer(TObject *Sender)
{
	static int k=0;
	static float p = 0;
	static float prev_p = -1;
	static float value1_prev = .0;
	static float value2_prev = .0;

	if(1) {
				prev_p=-1;
				p=0;
				drawCanvasNet();
				Image1->Canvas->MoveTo(p, Image1->Height*(1.0 - value1_prev/MASS_DIV)-1);
			}

	while(ser.ReadDataWaiting()) {
		int bytesRead = ser.ReadData(readedData+k, 1);
		k+=bytesRead;
		if(readedData[k-1]=='\n') {
			readedData[k]=0;
			k=0;

			int value1;
			int value2;

			sscanf(readedData, "%d", &value1);
			prev_p = p;
			p+=1;

			Image1->Canvas->Pen->Color = OutputLabel1->Font->Color;
			Image1->Canvas->MoveTo(prev_p, (Image1->Height-10)*(1.0 - value1_prev/1024.0*5.0)+5);
			Image1->Canvas->LineTo(p, (Image1->Height-10)*(1.0 - value1/1024.0*5.0)+5);
		 /*	Image1->Canvas->Pen->Color = OutputLabel2->Font->Color;
			Image1->Canvas->MoveTo(prev_p, Image1->Height*(1.0 - value2_prev/1024.0)-1);
			Image1->Canvas->LineTo(p, Image1->Height*(1.0 - value2/1024.0*5.0)-1); */
			value1_prev = value1;
			value2_prev = value2;

		   /*	if(p>Image1->Width) {
				prev_p=-1;
				p=0;
				drawCanvasNet();
				Image1->Canvas->MoveTo(p, Image1->Height*(1.0 - value1/MASS_DIV)-1);
			} */
		}

	}
}
//---------------------------------------------------------------------------
void __fastcall TForm1::DisconnectButtonClick(TObject *Sender)
{
	ser.Close();
	StatusLabel->Caption = "Connection closed";
}
//---------------------------------------------------------------------------
void __fastcall TForm1::ConnectButtonClick(TObject *Sender)
{
	BOOL result = ser.Open(ListBox1->ItemIndex+1, 115200);
	if(!result) StatusLabel->Caption = "Error create usb connection";
	wchar_t szPort[15];
	wchar_t szComParams[50];

	wsprintf( szPort, L"COM  %d", ListBox1->ItemIndex+1 );
	StatusLabel->Caption = szPort;
}
//---------------------------------------------------------------------------







