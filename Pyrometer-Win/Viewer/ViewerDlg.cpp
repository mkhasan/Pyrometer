
// ViewerDlg.cpp : implementation file
//

#include "stdafx.h"
#include "Viewer.h"
#include "ViewerDlg.h"
#include "afxdialogex.h"
//#include <atlstr.h>  

#include "USB2SERIAL_W32.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif






CString ComPortName;
int sensorId = -1;

uint8_t nodeList[20];
int nNodes=1;


bool quit = false;
int value = -1;
unsigned long value1 = 0;
unsigned long maxMissing=0;
unsigned long maxFrame = 0;

extern uint8_t nodeList[20];
extern int nNodes;

// CAboutDlg dialog used for App About

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

// Implementation
protected:
	DECLARE_MESSAGE_MAP()
	virtual void OnCancel();
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CViewerDlg dialog



CViewerDlg::CViewerDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_VIEWER_DIALOG, pParent)
	, port(_T("COM4"))
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CViewerDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CViewerDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BTN_SHOW, &CViewerDlg::OnBnClickedBtnShow)
	ON_WM_TIMER()
	ON_STN_CLICKED(IDC_STATIC_TEMP, &CViewerDlg::OnStnClickedStaticTemp)
	ON_STN_CLICKED(IDC_TEMP3, &CViewerDlg::OnStnClickedTemp3)
	ON_STN_CLICKED(IDC_MSG, &CViewerDlg::OnStnClickedMsg)
	ON_STN_CLICKED(IDC_TEMP, &CViewerDlg::OnStnClickedTemp)
	ON_STN_CLICKED(IDC_TEMP2, &CViewerDlg::OnStnClickedTemp2)
END_MESSAGE_MAP()


// CViewerDlg message handlers

BOOL CViewerDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// Add "About..." menu item to system menu.

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

	// TODO: Add extra initialization here

	CFont *m_Font1 = new CFont;
	m_Font1->CreatePointFont(300, _T("Arial Bold"));
	CStatic * m_Label = (CStatic *)GetDlgItem(IDC_MSG);
	m_Label->SetFont(m_Font1);

	m_Label->SetWindowTextW(_T(""));

	CFont *m_Font2 = new CFont;
	m_Font2->CreatePointFont(200, _T("Arial Bold"));

	GetDlgItem(IDC_STATIC_TEMP)->SetFont(m_Font2);
	GetDlgItem(IDC_STATIC_TEMP)->SetWindowTextW(_T("Temperature is"));

	GetDlgItem(IDC_ID)->SetWindowTextW(_T("15"));
	GetDlgItem(IDC_COM)->SetWindowTextW(port);
	
	//OnBnClickedBtnShow();

	GetDlgItem(IDC_TEMP)->ShowWindow(SW_HIDE);
	GetDlgItem(IDC_TEMP3)->ShowWindow(SW_HIDE);
	GetDlgItem(IDC_STATIC_SENDER)->ShowWindow(SW_HIDE);
	return TRUE;  // return TRUE  unless you set the focus to a control
}

void CViewerDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CViewerDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// The system calls this function to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CViewerDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



void CViewerDlg::OnBnClickedBtnShow()
{
	// TODO: Add your control notification handler code here

	CString idStr;
	GetDlgItem(IDC_ID)->GetWindowTextW(idStr);
	sensorId = _ttoi(idStr);

	nodeList[0] = (uint8_t)(sensorId & 0xFF);

	if (!(sensorId > 0 && sensorId < 0xFF)) {
		AfxMessageBox(_T("Error: Invalid sensor id"));
		exit(0);
	}

	GetDlgItem(IDC_ID)->EnableWindow(false);
	GetDlgItem(IDC_BTN_SHOW)->EnableWindow(false);
	GetDlgItem(IDC_COM)->EnableWindow(false);

	

	hThread = NULL;
	//static DWORD   dwThread;
	
	GetDlgItem(IDC_COM)->GetWindowTextW(port);

	ComPortName = port;

	value = -1;
		 
	hThread = CreateThread(
		NULL,                   // default security attributes
		0,                      // use default stack size  
		USB2SERIAL_W32,       // thread function name
		NULL,          // argument to thread function 
		0,                      // use default creation flags 
		NULL);   // returns the thread identifier 


	if (hThread == NULL) {

		AfxMessageBox(_T("Error"));
		exit(0);
	}

	//GetDlgItem(IDC_TEMP3)->ShowWindow(SW_HIDE);

	SetTimer(1, 1000, NULL);


}


extern sensorRecord sensor;
void CViewerDlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: Add your message handler code here and/or call default
	double val1 = -1.0;
	double val2 = -1.0;
	CString str;
	static int ret = 0;
	int dir = 1;
	int i=0;
	unsigned int count, sender;
	if (nIDEvent == 1) {
		// handle timer event

		if (sensor.ownPtr != NULL) {
		
			GetDlgItem(IDC_STATIC_TEMP)->ShowWindow(SW_SHOW);
			dir = 1;
			if (SensorRead(sensor.ComNr, &count, &sender, &val1, &val2) == SENSOR_TRUE) {
				str.Format(_T("%.2f"), val1);
				GetDlgItem(IDC_MSG)->SetWindowTextW(str);


				str.Format(_T("id is is %d"), count);
				GetDlgItem(IDC_TEMP)->SetWindowTextW(str);

				str.Format(_T("Sender is %d"), sender);
				GetDlgItem(IDC_STATIC_SENDER)->SetWindowTextW(str);

				//if (value1 > 0)
					//value1 = 100;
				str.Format(_T("in count is %ld"), value1);
				GetDlgItem(IDC_TEMP3)->SetWindowTextW(str);
				dir = -1;


			}


			ret += dir;
			str.Format(_T("frame came %ld"), maxFrame);
			GetDlgItem(IDC_TEMP3)->SetWindowTextW(str);

				
		}
	}
	CDialogEx::OnTimer(nIDEvent);
}


void CViewerDlg::OnStnClickedStaticTemp()
{
	// TODO: Add your control notification handler code here
}


void CAboutDlg::OnCancel()
{
	// TODO: Add your specialized code here and/or call the base class
	quit = true;
	CDialogEx::OnCancel();
}


void CViewerDlg::OnStnClickedTemp3()
{
	// TODO: Add your control notification handler code here
}


void CViewerDlg::OnStnClickedMsg()
{
	// TODO: Add your control notification handler code here
}


void CViewerDlg::OnStnClickedTemp()
{
	// TODO: Add your control notification handler code here
}


void CViewerDlg::OnStnClickedTemp2()
{
	// TODO: Add your control notification handler code here
}
