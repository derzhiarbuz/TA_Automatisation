/***************************************************************
 * Name:      Tadzhik_control_panel_wxMain.cpp
 * Purpose:   Code for Application Frame
 * Author:    Me ()
 * Created:   2018-05-13
 * Copyright: Me ()
 * License:
 **************************************************************/

#include "wx_pch.h"
#include "Tadzhik_control_panel_wxMain.h"
#include <wx/msgdlg.h>
#include <fstream>

//(*InternalHeaders(Tadzhik_control_panel_wxFrame)
#include <wx/font.h>
#include <wx/intl.h>
#include <wx/string.h>
//*)

//helper functions
enum wxbuildinfoformat {
    short_f, long_f };

wxString wxbuildinfo(wxbuildinfoformat format)
{
    wxString wxbuild(wxVERSION_STRING);

    if (format == long_f )
    {
#if defined(__WXMSW__)
        wxbuild << _T("-Windows");
#elif defined(__UNIX__)
        wxbuild << _T("-Linux");
#endif

#if wxUSE_UNICODE
        wxbuild << _T("-Unicode build");
#else
        wxbuild << _T("-ANSI build");
#endif // wxUSE_UNICODE
    }

    return wxbuild;
}

//(*IdInit(Tadzhik_control_panel_wxFrame)
const long Tadzhik_control_panel_wxFrame::ID_GLCANVAS1 = wxNewId();
const long Tadzhik_control_panel_wxFrame::ID_TEXTCTRL1 = wxNewId();
const long Tadzhik_control_panel_wxFrame::ID_PANEL2 = wxNewId();
const long Tadzhik_control_panel_wxFrame::ID_TEXTCTRL2 = wxNewId();
const long Tadzhik_control_panel_wxFrame::ID_PANEL3 = wxNewId();
const long Tadzhik_control_panel_wxFrame::ID_NOTEBOOK1 = wxNewId();
const long Tadzhik_control_panel_wxFrame::ID_BUTTON1 = wxNewId();
const long Tadzhik_control_panel_wxFrame::ID_BUTTON2 = wxNewId();
const long Tadzhik_control_panel_wxFrame::ID_BUTTON3 = wxNewId();
const long Tadzhik_control_panel_wxFrame::ID_STATICTEXT1 = wxNewId();
const long Tadzhik_control_panel_wxFrame::ID_TEXTCTRL3 = wxNewId();
const long Tadzhik_control_panel_wxFrame::ID_CHOICE1 = wxNewId();
const long Tadzhik_control_panel_wxFrame::ID_BUTTON4 = wxNewId();
const long Tadzhik_control_panel_wxFrame::ID_BUTTON5 = wxNewId();
const long Tadzhik_control_panel_wxFrame::ID_TEXTCTRL4 = wxNewId();
const long Tadzhik_control_panel_wxFrame::ID_BUTTON6 = wxNewId();
const long Tadzhik_control_panel_wxFrame::ID_BUTTON7 = wxNewId();
const long Tadzhik_control_panel_wxFrame::ID_BUTTON8 = wxNewId();
const long Tadzhik_control_panel_wxFrame::ID_BUTTON9 = wxNewId();
const long Tadzhik_control_panel_wxFrame::ID_STATICTEXT3 = wxNewId();
const long Tadzhik_control_panel_wxFrame::ID_STATICTEXT2 = wxNewId();
const long Tadzhik_control_panel_wxFrame::ID_STATICTEXT4 = wxNewId();
const long Tadzhik_control_panel_wxFrame::ID_TEXTCTRL5 = wxNewId();
const long Tadzhik_control_panel_wxFrame::ID_BUTTON10 = wxNewId();
const long Tadzhik_control_panel_wxFrame::ID_BUTTON11 = wxNewId();
const long Tadzhik_control_panel_wxFrame::ID_BUTTON12 = wxNewId();
const long Tadzhik_control_panel_wxFrame::ID_PANEL1 = wxNewId();
const long Tadzhik_control_panel_wxFrame::idMenuOpen = wxNewId();
const long Tadzhik_control_panel_wxFrame::idMenuSave = wxNewId();
const long Tadzhik_control_panel_wxFrame::idMenuQuit = wxNewId();
const long Tadzhik_control_panel_wxFrame::idMenuAbout = wxNewId();
const long Tadzhik_control_panel_wxFrame::ID_TIMER1 = wxNewId();
//*)

BEGIN_EVENT_TABLE(Tadzhik_control_panel_wxFrame,wxFrame)
    //(*EventTable(Tadzhik_control_panel_wxFrame)
    //*)
END_EVENT_TABLE()

Tadzhik_control_panel_wxFrame::Tadzhik_control_panel_wxFrame(wxWindow* parent,wxWindowID id)
{
    printerController.delegate = this;
    //(*Initialize(Tadzhik_control_panel_wxFrame)
    wxMenu* Menu1;
    wxMenu* Menu2;
    wxMenuBar* MenuBar1;
    wxMenuItem* MenuItem1;
    wxMenuItem* MenuItem2;

    Create(parent, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxDEFAULT_FRAME_STYLE, _T("wxID_ANY"));
    SetClientSize(wxSize(1200,700));
    Move(wxPoint(70,15));
    Panel1 = new wxPanel(this, ID_PANEL1, wxPoint(24,24), wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL1"));
    int GLCanvasAttributes_1[] = {
    	WX_GL_RGBA,
    	WX_GL_DOUBLEBUFFER,
    	WX_GL_DEPTH_SIZE,      16,
    	WX_GL_STENCIL_SIZE,    0,
    	0, 0 };
    #if wxCHECK_VERSION(3,0,0)
    	GLCanvas1 = new wxGLCanvas(Panel1, ID_GLCANVAS1, GLCanvasAttributes_1, wxPoint(8,8), wxSize(584,488), 0, _T("ID_GLCANVAS1"));
    #else
    	GLCanvas1 = new wxGLCanvas(Panel1, ID_GLCANVAS1, wxPoint(8,8), wxSize(584,488), 0, _T("ID_GLCANVAS1"), GLCanvasAttributes_1);
    #endif // wxCHECK_VERSION
    Notebook1 = new wxNotebook(Panel1, ID_NOTEBOOK1, wxPoint(600,0), wxSize(288,496), 0, _T("ID_NOTEBOOK1"));
    Panel2 = new wxPanel(Notebook1, ID_PANEL2, wxPoint(50,74), wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL2"));
    GCodeTextCtrl = new wxTextCtrl(Panel2, ID_TEXTCTRL1, wxEmptyString, wxPoint(0,0), wxSize(288,472), wxTE_MULTILINE, wxDefaultValidator, _T("ID_TEXTCTRL1"));
    Panel3 = new wxPanel(Notebook1, ID_PANEL3, wxPoint(88,11), wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL3"));
    PathTextCtrl = new wxTextCtrl(Panel3, ID_TEXTCTRL2, wxEmptyString, wxPoint(0,0), wxSize(288,472), wxTE_MULTILINE, wxDefaultValidator, _T("ID_TEXTCTRL2"));
    Notebook1->AddPage(Panel2, _("G-code"), false);
    Notebook1->AddPage(Panel3, _("Path"), false);
    MakePathButton = new wxButton(Panel1, ID_BUTTON1, _("Make Path"), wxPoint(608,504), wxSize(88,34), 0, wxDefaultValidator, _T("ID_BUTTON1"));
    SmoothPathButton = new wxButton(Panel1, ID_BUTTON2, _("Smooth"), wxPoint(704,504), wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON2"));
    Button3 = new wxButton(Panel1, ID_BUTTON3, _("Show 3D"), wxPoint(800,504), wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON3"));
    StaticText1 = new wxStaticText(Panel1, ID_STATICTEXT1, _("Log"), wxPoint(912,8), wxDefaultSize, 0, _T("ID_STATICTEXT1"));
    LogTextCtrl = new wxTextCtrl(Panel1, ID_TEXTCTRL3, wxEmptyString, wxPoint(904,24), wxSize(288,472), wxTE_MULTILINE, wxDefaultValidator, _T("ID_TEXTCTRL3"));
    portChoice = new wxChoice(Panel1, ID_CHOICE1, wxPoint(904,504), wxSize(104,32), 0, 0, 0, wxDefaultValidator, _T("ID_CHOICE1"));
    portChoice->Append(_("COM1"));
    portChoice->Append(_("COM2"));
    portChoice->Append(_("COM3"));
    portChoice->Append(_("COM4"));
    portChoice->Append(_("COM5"));
    portChoice->Append(_("COM6"));
    portChoice->Append(_("COM7"));
    portChoice->Append(_("COM8"));
    portChoice->Append(_("COM9"));
    portChoice->Append(_("COM10"));
    portChoice->Append(_("COM11"));
    portChoice->Append(_("COM12"));
    portChoice->Append(_("COM13"));
    portChoice->Append(_("COM14"));
    portChoice->SetSelection( portChoice->Append(_("COM15")) );
    portChoice->Append(_("COM16"));
    portChoice->Append(_("COM17"));
    portChoice->Append(_("COM18"));
    portChoice->Append(_("COM19"));
    portChoice->Append(_("COM20"));
    ConnectButtom = new wxButton(Panel1, ID_BUTTON4, _("Connect"), wxPoint(1016,504), wxSize(80,34), 0, wxDefaultValidator, _T("ID_BUTTON4"));
    DisconnectButton = new wxButton(Panel1, ID_BUTTON5, _("Disconnect"), wxPoint(1104,504), wxSize(88,34), 0, wxDefaultValidator, _T("ID_BUTTON5"));
    CommandTextCtrl = new wxTextCtrl(Panel1, ID_TEXTCTRL4, wxEmptyString, wxPoint(904,552), wxSize(184,21), 0, wxDefaultValidator, _T("ID_TEXTCTRL4"));
    SendButton = new wxButton(Panel1, ID_BUTTON6, _("Send"), wxPoint(1104,544), wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON6"));
    StartButtom = new wxButton(Panel1, ID_BUTTON7, _("Start"), wxPoint(608,544), wxSize(88,34), 0, wxDefaultValidator, _T("ID_BUTTON7"));
    wxFont StartButtomFont(10,wxFONTFAMILY_SWISS,wxFONTSTYLE_NORMAL,wxFONTWEIGHT_NORMAL,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    StartButtom->SetFont(StartButtomFont);
    StopButton = new wxButton(Panel1, ID_BUTTON8, _("Stop"), wxPoint(704,544), wxSize(88,34), 0, wxDefaultValidator, _T("ID_BUTTON8"));
    wxFont StopButtonFont(10,wxFONTFAMILY_SWISS,wxFONTSTYLE_NORMAL,wxFONTWEIGHT_NORMAL,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    StopButton->SetFont(StopButtonFont);
    Button2 = new wxButton(Panel1, ID_BUTTON9, _("Pause"), wxPoint(800,544), wxSize(88,34), 0, wxDefaultValidator, _T("ID_BUTTON9"));
    XYZText = new wxStaticText(Panel1, ID_STATICTEXT3, _("XYZ ="), wxPoint(16,504), wxSize(256,32), 0, _T("ID_STATICTEXT3"));
    XYZText->SetForegroundColour(wxColour(0,0,160));
    wxFont XYZTextFont(20,wxFONTFAMILY_DEFAULT,wxFONTSTYLE_NORMAL,wxFONTWEIGHT_BOLD,false,_T("Rasa"),wxFONTENCODING_DEFAULT);
    XYZText->SetFont(XYZTextFont);
    NormVText = new wxStaticText(Panel1, ID_STATICTEXT2, _("||V|| ="), wxPoint(16,544), wxSize(256,32), 0, _T("ID_STATICTEXT2"));
    NormVText->SetForegroundColour(wxColour(0,0,160));
    wxFont NormVTextFont(20,wxFONTFAMILY_DEFAULT,wxFONTSTYLE_NORMAL,wxFONTWEIGHT_BOLD,false,_T("Rasa"),wxFONTENCODING_DEFAULT);
    NormVText->SetFont(NormVTextFont);
    StaticText2 = new wxStaticText(Panel1, ID_STATICTEXT4, _("Set velocity (mm/s):"), wxPoint(904,592), wxDefaultSize, 0, _T("ID_STATICTEXT4"));
    VelocityTextCtrl = new wxTextCtrl(Panel1, ID_TEXTCTRL5, _("30"), wxPoint(1040,592), wxSize(56,21), 0, wxDefaultValidator, _T("ID_TEXTCTRL5"));
    SetVelocityButton = new wxButton(Panel1, ID_BUTTON10, _("Set"), wxPoint(1104,584), wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON10"));
    StartPrintheadButton = new wxButton(Panel1, ID_BUTTON11, _("Printhead ON"), wxPoint(912,624), wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON11"));
    StopPrintheadButton = new wxButton(Panel1, ID_BUTTON12, _("Printhead OFF"), wxPoint(1056,624), wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON12"));
    MenuBar1 = new wxMenuBar();
    Menu1 = new wxMenu();
    MenuItem3 = new wxMenuItem(Menu1, idMenuOpen, _("Open"), wxEmptyString, wxITEM_NORMAL);
    Menu1->Append(MenuItem3);
    MenuItem4 = new wxMenuItem(Menu1, idMenuSave, _("Save Path"), wxEmptyString, wxITEM_NORMAL);
    Menu1->Append(MenuItem4);
    MenuItem1 = new wxMenuItem(Menu1, idMenuQuit, _("Quit\tAlt-F4"), _("Quit the application"), wxITEM_NORMAL);
    Menu1->Append(MenuItem1);
    MenuBar1->Append(Menu1, _("&File"));
    Menu2 = new wxMenu();
    MenuItem2 = new wxMenuItem(Menu2, idMenuAbout, _("About\tF1"), _("Show info about this application"), wxITEM_NORMAL);
    Menu2->Append(MenuItem2);
    MenuBar1->Append(Menu2, _("Help"));
    SetMenuBar(MenuBar1);
    SerialTimer.SetOwner(this, ID_TIMER1);
    SerialTimer.Start(1, false);

    Connect(ID_BUTTON1,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&Tadzhik_control_panel_wxFrame::OnMakePathButtonClick);
    Connect(ID_BUTTON2,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&Tadzhik_control_panel_wxFrame::OnSmoothPathButtonClick);
    Connect(ID_BUTTON4,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&Tadzhik_control_panel_wxFrame::OnConnectButtomClick);
    Connect(ID_BUTTON5,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&Tadzhik_control_panel_wxFrame::OnDisconnectButtonClick);
    Connect(ID_BUTTON6,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&Tadzhik_control_panel_wxFrame::OnSendButtonClick);
    Connect(ID_BUTTON7,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&Tadzhik_control_panel_wxFrame::OnStartButtomClick);
    Connect(ID_BUTTON10,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&Tadzhik_control_panel_wxFrame::OnSetVelocityButtonClick);
    Connect(ID_BUTTON11,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&Tadzhik_control_panel_wxFrame::OnStartPrintheadButtonClick);
    Connect(ID_BUTTON12,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&Tadzhik_control_panel_wxFrame::OnStopPrintheadButtonClick);
    Connect(idMenuQuit,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&Tadzhik_control_panel_wxFrame::OnQuit);
    Connect(idMenuAbout,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&Tadzhik_control_panel_wxFrame::OnAbout);
    Connect(ID_TIMER1,wxEVT_TIMER,(wxObjectEventFunction)&Tadzhik_control_panel_wxFrame::OnSerialTimerTrigger);
    //*)
    Connect(idMenuOpen,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&Tadzhik_control_panel_wxFrame::OnOpen);
    Connect(idMenuSave,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&Tadzhik_control_panel_wxFrame::OnSave);
}

Tadzhik_control_panel_wxFrame::~Tadzhik_control_panel_wxFrame()
{
    printerController.delegate = 0;
    //(*Destroy(Tadzhik_control_panel_wxFrame)
    //*)
}

void Tadzhik_control_panel_wxFrame::OnQuit(wxCommandEvent& event)
{
    Close();
}

void Tadzhik_control_panel_wxFrame::OnAbout(wxCommandEvent& event)
{
    wxString msg = wxbuildinfo(long_f);
    wxMessageBox(msg, _("Welcome to..."));
}

void Tadzhik_control_panel_wxFrame::OnOpen(wxCommandEvent& event)
{
        wxFileDialog
        openFileDialog(this, _("Open G-Code file"), "", "",
                       "G-Code files (*.gcode)|*.gcode", wxFD_OPEN|wxFD_FILE_MUST_EXIST);
    if (openFileDialog.ShowModal() == wxID_CANCEL) {
        return;     // the user changed idea...
    }

    if(!GCodeTextCtrl->LoadFile(openFileDialog.GetPath())) {
        wxLogError("Cannot open '%s'.", openFileDialog.GetPath());
    }

    printerController.convertGCodeToPath(openFileDialog.GetPath().ToStdString());
}

void Tadzhik_control_panel_wxFrame::OnSave(wxCommandEvent& event)
{
        wxFileDialog openFileDialog(this, _("Open G-Code file"), "", "",
                       "G-Code files (*.gcode)|*.gcode", wxFD_SAVE);

    if (openFileDialog.ShowModal() == wxID_CANCEL) {
        return;     // the user changed idea...
    }

    if(!PathTextCtrl->SaveFile(openFileDialog.GetPath())) {
        wxLogError("Cannot save '%s'.", openFileDialog.GetPath());
    }

    printerController.convertGCodeToPath(openFileDialog.GetPath().ToStdString());
}

void Tadzhik_control_panel_wxFrame::displayPath() {
    PathTextCtrl->Hide();
    PathTextCtrl->Clear();
    for(TALine line : printerController.path) {
        (*PathTextCtrl)<<"G"<<(wxString::Format(wxT("%d"),line.G))<<" X"<<(wxString::Format(wxT("%0.2f"),line.toX))<<" Y"<<(wxString::Format(wxT("%0.2f"),line.toY))<<" Z"<<(wxString::Format(wxT("%0.2f"),line.toZ))<<"\n";
       // (*PathTextCtrl)<<(wxString::Format(wxT("%5d"),line.N))<<" G"<<(wxString::Format(wxT("%d"),line.G))<<" X"<<(wxString::Format(wxT("%0.2f"),line.toX))<<" Y"<<(wxString::Format(wxT("%0.2f"),line.toY))<<" Z"<<(wxString::Format(wxT("%0.2f"),line.toZ))<<"\n";
    }
    PathTextCtrl->Show();
}


void Tadzhik_control_panel_wxFrame::OnMakePathButtonClick(wxCommandEvent& event)
{
    this->displayPath();
}

void Tadzhik_control_panel_wxFrame::OnSmoothPathButtonClick(wxCommandEvent& event)
{
    printerController.smoothPath(5.);
    this->displayPath();
}

void Tadzhik_control_panel_wxFrame::OnConnectButtomClick(wxCommandEvent& event)
{
    (*LogTextCtrl)<<"Connecting to printer...\n";
    printerController.connectToPrinter(portChoice->GetSelection()+1, 115200);
    if(printerController.connectedToPrinter())
        (*LogTextCtrl)<<"Connected\n";
    else
        (*LogTextCtrl)<<"Connection failed!\n";
}

void Tadzhik_control_panel_wxFrame::OnDisconnectButtonClick(wxCommandEvent& event)
{
    printerController.disconnect();
    (*LogTextCtrl)<<"Disconnected\n";
}

void Tadzhik_control_panel_wxFrame::OnSendButtonClick(wxCommandEvent& event)
{
    if(CommandTextCtrl->GetLineText(0).size() <= 0) return;

    if(!printerController.connectedToPrinter()) {
        (*LogTextCtrl)<<"No connection\n";
    }
    else if(!printerController.sendCommand(CommandTextCtrl->GetLineText(0).ToStdString()))
        (*LogTextCtrl)<<"Not a command\n";
}

void Tadzhik_control_panel_wxFrame::logString(string &str) {
    (*LogTextCtrl)<<str<<"\n";
}

void Tadzhik_control_panel_wxFrame::handleTelemetry(TATelemetry &telemetry) {
   XYZText->SetLabel((wxString::Format(wxT("XYZ = (%0.2f, %0.2f, %0.2f)"),telemetry.X, telemetry.Y, telemetry.Z)));
   float normV;
   normV = sqrt(telemetry.V[0]*telemetry.V[0] + telemetry.V[1]*telemetry.V[1]);
   NormVText->SetLabel((wxString::Format(wxT("||V|| = %0.2f"), normV)));
}

void Tadzhik_control_panel_wxFrame::OnSerialTimerTrigger(wxTimerEvent& event)
{
    printerController.communicationLoop();
}

void Tadzhik_control_panel_wxFrame::OnStartButtomClick(wxCommandEvent& event)
{
    printerController.start();
}

void Tadzhik_control_panel_wxFrame::OnSetVelocityButtonClick(wxCommandEvent& event)
{
    long vel;
    if(VelocityTextCtrl->GetLineText(0).ToLong(&vel)) {
        printerController.setSpeed(vel);
    }
}

void Tadzhik_control_panel_wxFrame::OnStartPrintheadButtonClick(wxCommandEvent& event)
{
    printerController.startPrinthead();
}

void Tadzhik_control_panel_wxFrame::OnStopPrintheadButtonClick(wxCommandEvent& event)
{
    printerController.stopPrinthead();
}
