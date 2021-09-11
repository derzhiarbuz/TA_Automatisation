/***************************************************************
 * Name:      Tadzhik_control_panel_wxMain.h
 * Purpose:   Defines Application Frame
 * Author:    Me ()
 * Created:   2018-05-13
 * Copyright: Me ()
 * License:
 **************************************************************/

#ifndef TADZHIK_CONTROL_PANEL_WXMAIN_H
#define TADZHIK_CONTROL_PANEL_WXMAIN_H

#include "TadzhikController.h"

//(*Headers(Tadzhik_control_panel_wxFrame)
#include <wx/button.h>
#include <wx/choice.h>
#include <wx/frame.h>
#include <wx/glcanvas.h>
#include <wx/menu.h>
#include <wx/notebook.h>
#include <wx/panel.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/timer.h>
//*)

class Tadzhik_control_panel_wxFrame: public wxFrame, public TadzhikControllerDelegate
{
    public:

        TadzhikController printerController;

        Tadzhik_control_panel_wxFrame(wxWindow* parent,wxWindowID id = -1);
        virtual ~Tadzhik_control_panel_wxFrame();
        void displayPath();

    //TadzhikControllerDelegate
        void logString(string &str);
        void handleTelemetry(TATelemetry &telemetry);

    private:

        //(*Handlers(Tadzhik_control_panel_wxFrame)
        void OnQuit(wxCommandEvent& event);
        void OnAbout(wxCommandEvent& event);
        void OnGCodeTextCtrlText(wxCommandEvent& event);
        void OnMakePathButtonClick(wxCommandEvent& event);
        void OnSmoothPathButtonClick(wxCommandEvent& event);
        void OnConnectButtomClick(wxCommandEvent& event);
        void OnDisconnectButtonClick(wxCommandEvent& event);
        void OnSendButtonClick(wxCommandEvent& event);
        void OnSerialTimerTrigger(wxTimerEvent& event);
        void OnButton2Click(wxCommandEvent& event);
        void OnStartButtomClick(wxCommandEvent& event);
        void OnSetVelocityButtonClick(wxCommandEvent& event);
        void OnStartPrintheadButtonClick(wxCommandEvent& event);
        void OnStopPrintheadButtonClick(wxCommandEvent& event);
        //*)

        //(*Identifiers(Tadzhik_control_panel_wxFrame)
        static const long ID_GLCANVAS1;
        static const long ID_TEXTCTRL1;
        static const long ID_PANEL2;
        static const long ID_TEXTCTRL2;
        static const long ID_PANEL3;
        static const long ID_NOTEBOOK1;
        static const long ID_BUTTON1;
        static const long ID_BUTTON2;
        static const long ID_BUTTON3;
        static const long ID_STATICTEXT1;
        static const long ID_TEXTCTRL3;
        static const long ID_CHOICE1;
        static const long ID_BUTTON4;
        static const long ID_BUTTON5;
        static const long ID_TEXTCTRL4;
        static const long ID_BUTTON6;
        static const long ID_BUTTON7;
        static const long ID_BUTTON8;
        static const long ID_BUTTON9;
        static const long ID_STATICTEXT3;
        static const long ID_STATICTEXT2;
        static const long ID_STATICTEXT4;
        static const long ID_TEXTCTRL5;
        static const long ID_BUTTON10;
        static const long ID_BUTTON11;
        static const long ID_BUTTON12;
        static const long ID_PANEL1;
        static const long idMenuOpen;
        static const long idMenuSave;
        static const long idMenuQuit;
        static const long idMenuAbout;
        static const long ID_TIMER1;
        //*)

        //(*Declarations(Tadzhik_control_panel_wxFrame)
        wxButton* Button2;
        wxButton* Button3;
        wxButton* ConnectButtom;
        wxButton* DisconnectButton;
        wxButton* MakePathButton;
        wxButton* SendButton;
        wxButton* SetVelocityButton;
        wxButton* SmoothPathButton;
        wxButton* StartButtom;
        wxButton* StartPrintheadButton;
        wxButton* StopButton;
        wxButton* StopPrintheadButton;
        wxChoice* portChoice;
        wxGLCanvas* GLCanvas1;
        wxMenuItem* MenuItem3;
        wxMenuItem* MenuItem4;
        wxNotebook* Notebook1;
        wxPanel* Panel1;
        wxPanel* Panel2;
        wxPanel* Panel3;
        wxStaticText* NormVText;
        wxStaticText* StaticText1;
        wxStaticText* StaticText2;
        wxStaticText* XYZText;
        wxTextCtrl* CommandTextCtrl;
        wxTextCtrl* GCodeTextCtrl;
        wxTextCtrl* LogTextCtrl;
        wxTextCtrl* PathTextCtrl;
        wxTextCtrl* VelocityTextCtrl;
        wxTimer SerialTimer;
        //*)

        void OnOpen(wxCommandEvent& event);
        void OnSave(wxCommandEvent& event);

        DECLARE_EVENT_TABLE()
};

#endif // TADZHIK_CONTROL_PANEL_WXMAIN_H
