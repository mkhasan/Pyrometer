
// ViewerDlg.h : header file
//

#pragma once


// CViewerDlg dialog
class CViewerDlg : public CDialogEx
{
// Construction
public:
	CViewerDlg(CWnd* pParent = NULL);	// standard constructor

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_VIEWER_DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support


	CString port;
	HANDLE  hThread;

// Implementation
protected:
	HICON m_hIcon;

	// Generated message map functions
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedBtnShow();
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	afx_msg void OnStnClickedStaticTemp();
	afx_msg void OnStnClickedTemp3();
	afx_msg void OnStnClickedMsg();
	afx_msg void OnStnClickedTemp();
	afx_msg void OnStnClickedTemp2();
};
