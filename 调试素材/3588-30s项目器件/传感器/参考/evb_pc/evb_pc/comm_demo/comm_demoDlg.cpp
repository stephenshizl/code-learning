// comm_demoDlg.cpp : 实现文件
//

#include "stdafx.h"
#include "comm_demo.h"
#include "comm_demoDlg.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

#if 0
#if defined(MAG_CALI_SUPPORT)
#pragma comment(lib, "mag_cali.lib")
extern "C" struct mag_lib_interface_t MAG_LIB_API_INTERFACE;
#endif

#if defined(QMC_CALI_2D_SUPPORT)
extern "C"
{
	int process(float *mMagneticData);
};
#endif

#if defined(QMC_CALI_3D_SUPPORT)
static int mag_cali_count = 0;
static int m_magR;
static unsigned char mag_cali_flag=0;
static short mag_ofsx=0,mag_ofsy=0,mag_ofsz=0,Mag_AvgR=0;
#endif
#endif

//#ifdef _DEBUG
#define DEBUG_CONSOLE
//#endif
#ifdef DEBUG_CONSOLE
#define QST_PRINTF		_cprintf
#else
#define QST_PRINTF
#endif


#ifdef DEBUG_CONSOLE
static BOOL console_flag = FALSE;

static void InitConsoleWindow()
{    
	if(console_flag == FALSE)
	{
		AllocConsole();    
		HANDLE handle = GetStdHandle(STD_OUTPUT_HANDLE);    
		int hCrt = _open_osfhandle((long)handle,_O_TEXT);    
		FILE * hf = _fdopen( hCrt, "w" );    
		*stdout = *hf;
		console_flag = TRUE;
		QST_PRINTF("InitConsoleWindow \n");
	}
}

static void CloseConsoleWindow()
{
	if(console_flag == TRUE)
	{
		FreeConsole();
		console_flag = FALSE;
		QST_PRINTF("CloseConsoleWindow \n");
	}
}
#endif

/*
int64_t qst_time_in_nanosec(void) 
{
	SYSTEMTIME sys;
	GetLocalTime(&sys);
	return (sys.wSecond * 1000LL + sys.wMilliseconds) * 1000000LL;
}
*/

/*
unsigned char qmaX981_reg_data[][6] = 
{
	{0x00,0x01,0x9e,0xfe,0x7c,0x12},

};

int qmaX981_calc_raw(void)
{
	unsigned char databuf[6] = { 0 };
	int rawData[3];
	int len = (sizeof(qmaX981_reg_data) / (sizeof(unsigned char) * 6));
	int i;

	for (i = 0; i < len; i++)
	{
		databuf[0] = qmaX981_reg_data[i][0];
		databuf[1] = qmaX981_reg_data[i][1];
		databuf[2] = qmaX981_reg_data[i][2];
		databuf[3] = qmaX981_reg_data[i][3];
		databuf[4] = qmaX981_reg_data[i][4];
		databuf[5] = qmaX981_reg_data[i][5];

		rawData[0] = (short)(((unsigned short)databuf[1] << 8) | (databuf[0]));
		rawData[1] = (short)(((unsigned short)databuf[3] << 8) | (databuf[2]));
		rawData[2] = (short)(((unsigned short)databuf[5] << 8) | (databuf[4]));
		rawData[0] = rawData[0] >> 2;
		rawData[1] = rawData[1] >> 2;
		rawData[2] = rawData[2] >> 2;
		QST_PRINTF("	raw:	%d	%d	%d\n", rawData[0], rawData[1], rawData[2]);
	}

	return QMAX981_SUCCESS;
}
*/

class CAboutDlg : public CDialog
{
public:
	CAboutDlg();

// 对话框数据
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

// 实现
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialog(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialog)
END_MESSAGE_MAP()


// Ccomm_demoDlg 对话框

Ccomm_demoDlg::Ccomm_demoDlg(CWnd* pParent /*=NULL*/)
	: CDialog(Ccomm_demoDlg::IDD, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void Ccomm_demoDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_CHARTCTRL_1, m_ChartCtrl);
	DDX_Control(pDX, IDC_CHARTCTRL_2, m_ChartCtrl_2);
	DDX_Control(pDX, IDC_CHARTCTRL_3, m_ChartCtrl_3);
}

BEGIN_MESSAGE_MAP(Ccomm_demoDlg, CDialog)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	//}}AFX_MSG_MAP
	ON_BN_CLICKED(IDC_BT_OPEN_COMM, &Ccomm_demoDlg::OnBnClickedBtOpenDevice)
	ON_BN_CLICKED(IDOK, &Ccomm_demoDlg::OnBnClickedOk)
	ON_EN_CHANGE(IDC_EDIT_PORT, &Ccomm_demoDlg::OnEnChangeCommPort)
	ON_WM_DEVICECHANGE()
	ON_WM_TIMER()
	ON_WM_CLOSE()
	ON_WM_DESTROY()
	ON_WM_CTLCOLOR()
	ON_BN_CLICKED(IDC_BUTTON_REG_WRITE, &Ccomm_demoDlg::OnBnClickedButtonRegWrite)
	ON_BN_CLICKED(IDC_BUTTON_REG_READ, &Ccomm_demoDlg::OnBnClickedButtonRegRead)
	ON_EN_CHANGE(IDC_EDIT_SLAVE, &Ccomm_demoDlg::OnEnChangeEditSlave)
	//ON_CBN_SELCHANGE(IDC_COMBO_COM_LIST, &Ccomm_demoDlg::OnCbnSelchangeComboComList)
	ON_BN_CLICKED(IDC_BUTTON_CALI, &Ccomm_demoDlg::OnBnClickedButtonCali)
//	ON_BN_CLICKED(IDC_CHECK_PROTOCOL, &Ccomm_demoDlg::OnBnClickedCheckProtocol)
ON_BN_CLICKED(IDC_BUTTON_IO_WRITE, &Ccomm_demoDlg::OnBnClickedButtonIoWrite)
ON_BN_CLICKED(IDC_BT_START, &Ccomm_demoDlg::OnBnClickedBtStart)
END_MESSAGE_MAP()


// Ccomm_demoDlg 消息处理程序

BOOL Ccomm_demoDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	// 将“关于...”菜单项添加到系统菜单中。

	// IDM_ABOUTBOX 必须在系统命令范围内。
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		CString strAboutMenu;
		strAboutMenu.LoadString(IDS_ABOUTBOX);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 设置此对话框的图标。当应用程序主窗口不是对话框时，框架将自动
	//  执行此操作
	SetIcon(m_hIcon, TRUE);			// 设置大图标
	SetIcon(m_hIcon, FALSE);		// 设置小图标

	// TODO: 在此添加额外的初始化代码
#ifdef DEBUG_CONSOLE
	InitConsoleWindow();
#endif
	app_init_para();
	master_type = usb_open_device();
	usb_close_device();
	app_chart_init();
	app_refresh_ui(QST_SENSOR_NONE);

#if defined(MAG_CALI_SUPPORT)
	MAG_LIB_API_INTERFACE.initLib(&qmcInfo);
#endif
	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}

void Ccomm_demoDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialog::OnSysCommand(nID, lParam);
	}
}

// 如果向对话框添加最小化按钮，则需要下面的代码
//  来绘制该图标。对于使用文档/视图模型的 MFC 应用程序，
//  这将由框架自动完成。

void Ccomm_demoDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 用于绘制的设备上下文

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 使图标在工作区矩形中居中
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 绘制图标
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialog::OnPaint();
	}
}

//当用户拖动最小化窗口时系统调用此函数取得光标
//显示。
HCURSOR Ccomm_demoDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}


void Ccomm_demoDlg::app_init_para(void)
{
	sampleRate = 16;
	protocol_type = USB_I2C;
	sensor_type = QST_SENSOR_NONE;
	mChartInit = FALSE;
	log_flag = FALSE;
	edit_slave = 0x00;
	step = 0;

	master_type = DEVICE_NONE;
	master_connect = FALSE;
	master_start = FALSE;
	master_status = FALSE;

	InitBtwzFilter();
#if defined(QST_MAG_CALI_SUPPORT)
	struct magChipInfo mag_info;
	mag_info.deviceid = 0x80;
	mag_info.hwGyro = 0;
	mag_info.layout = 0;

	mag_cali_p = &MAG_LIB_API_INTERFACE;
	mag_cali_p->initLib(&mag_info);
	mag_cali_p->enableLib(1);
#endif
}

void Ccomm_demoDlg::app_reset_para(void)
{
	sensor_type = QST_SENSOR_NONE;

	master_connect = FALSE;
	master_start = FALSE;
	master_status = FALSE;
}

void Ccomm_demoDlg::app_init_ui(void)
{
	CString sampleRateStr;
	//CEdit* pEditArea;
	//CStatic *pStaticArea;

	// set sample rate
	sampleRateStr.Empty();
	sampleRateStr.Format(L"%d", sampleRate);
	GetDlgItem(IDC_EDIT_SAMPLERATE)->SetWindowTextW(sampleRateStr);
	sampleRateStr.ReleaseBuffer();

	GetDlgItem(IDC_STATIC_STEPCOUNTER)->ShowWindow(SW_HIDE);
	GetDlgItem(IDC_BUTTON_CALI)->ShowWindow(SW_HIDE);
	
	//m_textFont.CreatePointFont(200, L"宋体");
	//pStaticArea = (CStatic *)GetDlgItem(IDC_STATIC_PORT);
	//pEditArea = (CEdit*)GetDlgItem(IDC_EDIT_PORT);
	if((master_type == DEVICE_CH341A)||(master_type == DEVICE_STM32F103))
	{
		GetDlgItem(IDC_STATIC_PORT)->SetWindowTextW(_T("Device:"));
		if(master_type == DEVICE_CH341A)
			GetDlgItem(IDC_EDIT_PORT)->SetWindowTextW(_T(" CH341A"));
		else if(master_type == DEVICE_STM32F103)
			GetDlgItem(IDC_EDIT_PORT)->SetWindowTextW(_T(" STM32"));
		GetDlgItem(IDC_EDIT_PORT)->EnableWindow(0);
	}
	else
	{	
		GetDlgItem(IDC_STATIC_PORT)->SetWindowTextW(_T("Serial:"));
		GetDlgItem(IDC_EDIT_PORT)->SetWindowTextW(_T(""));
		GetDlgItem(IDC_EDIT_PORT)->EnableWindow(1);
	}
	GetDlgItem(IDC_BT_OPEN_COMM)->SetWindowTextW(_T("Open"));
}

void Ccomm_demoDlg::app_refresh_ui(int type)
{
	CString uiStr;

	GetDlgItem(IDC_BUTTON_CALI)->ShowWindow(SW_HIDE);
#if defined(QMAX981_STEPCOUNTER)
	GetDlgItem(IDC_STATIC_STEPCOUNTER)->ShowWindow(SW_SHOW);
#endif
	if((master_type == DEVICE_CH341A)||(master_type == DEVICE_STM32F103))
	{
		GetDlgItem(IDC_STATIC_PORT)->SetWindowTextW(_T("Device:"));
		if(master_type == DEVICE_CH341A)
			GetDlgItem(IDC_EDIT_PORT)->SetWindowTextW(_T(" CH341A"));
		else if(master_type == DEVICE_STM32F103)
			GetDlgItem(IDC_EDIT_PORT)->SetWindowTextW(_T(" STM32"));
		GetDlgItem(IDC_EDIT_PORT)->EnableWindow(0);
	}
	else if(master_type == DEVICE_CP2102)
	{
		GetDlgItem(IDC_EDIT_PORT)->EnableWindow(0);
	}
	else
	{	
		GetDlgItem(IDC_STATIC_PORT)->SetWindowTextW(_T("Serial:"));
		GetDlgItem(IDC_EDIT_PORT)->SetWindowTextW(_T(""));
		GetDlgItem(IDC_EDIT_PORT)->EnableWindow(1);
	}

	if(master_connect)
	{
		GetDlgItem(IDC_BT_OPEN_COMM)->SetWindowTextW(_T("Close"));
		GetDlgItem(IDC_BT_START)->EnableWindow(1);

		GetDlgItem(IDC_EDIT_SAMPLERATE)->GetWindowTextW(uiStr);
		sampleRate = _wtoi(uiStr.GetBuffer(uiStr.GetLength()));

		GetDlgItem(IDC_EDIT_SAMPLERATE)->EnableWindow(FALSE);
		GetDlgItem(IDC_CHECK_SAVEDATA)->EnableWindow(FALSE);
		GetDlgItem(IDC_CHECK_PROTOCOL)->EnableWindow(FALSE);
	}
	else
	{
		GetDlgItem(IDC_BT_OPEN_COMM)->SetWindowTextW(_T("Open"));
		GetDlgItem(IDC_BT_START)->EnableWindow(0);

		uiStr.Empty();
		uiStr.Format(L"%d", sampleRate);
		GetDlgItem(IDC_EDIT_SAMPLERATE)->SetWindowTextW(uiStr);
		GetDlgItem(IDC_EDIT_SAMPLERATE)->EnableWindow(TRUE);
		GetDlgItem(IDC_CHECK_SAVEDATA)->EnableWindow(TRUE);
		GetDlgItem(IDC_CHECK_PROTOCOL)->EnableWindow(TRUE);
	}
	
	if(master_start)
	{
		GetDlgItem(IDC_BT_START)->SetWindowTextW(_T("Stop"));
	}
	else
	{
		GetDlgItem(IDC_BT_START)->SetWindowTextW(_T("Start"));
	}

	uiStr.Empty();
	uiStr.Format(L"%x", edit_slave);
	GetDlgItem(IDC_EDIT_SLAVE)->SetWindowTextW(uiStr.GetString());
	uiStr.ReleaseBuffer();

	m_ChartCtrl.EnableRefresh(true);
	//m_ChartCtrl_2.EnableRefresh(true);	
	//m_ChartCtrl_3.EnableRefresh(true);
	m_ChartCtrl_2.ShowWindow(SW_HIDE);
	m_ChartCtrl_3.ShowWindow(SW_HIDE);
}

void Ccomm_demoDlg::app_chart_init(void)
{
	CChartAxis *pAxis= NULL; 
	//CChartAxisLabel *pLabel=NULL;
	//TChartString str_dis;

	if(mChartInit == TRUE)
		return;
	mChartInit = TRUE;
	pAxis = m_ChartCtrl.CreateStandardAxis(CChartCtrl::BottomAxis);
	pAxis->SetAutomatic(true);
	pAxis->GetLabel()->SetText(_T("Sample"));

	pAxis = m_ChartCtrl.CreateStandardAxis(CChartCtrl::LeftAxis);
	pAxis->SetAutomatic(true); 
	pAxis->SetAutomaticMode(CChartAxis::FullAutomatic);
	pAxis->GetLabel()->SetText(_T("Data"));
	pAxis->SetGap(3.0);

	m_ChartCtrl.GetTitle()->AddString(_T("QST sensor"));
	m_ChartCtrl.GetTitle()->SetColor(RGB(255,100,110));

	pLineSerie1 = m_ChartCtrl.CreateLineSerie();
	pLineSerie1->SetSeriesOrdering(poNoOrdering);//设置为无序  
	pLineSerie1->SetColor(RGB(255,0,0));
	pLineSerie1->SetName(_T("line 1"));
	
	pLineSerie2 = m_ChartCtrl.CreateLineSerie();
	pLineSerie2->SetSeriesOrdering(poNoOrdering);//设置为无序  
	pLineSerie2->SetColor(RGB(0,255,0));
	pLineSerie2->SetName(_T("line 2"));
	
	pLineSerie3 = m_ChartCtrl.CreateLineSerie();
	pLineSerie3->SetSeriesOrdering(poNoOrdering);//设置为无序  
	pLineSerie3->SetColor(RGB(0,0,255));
	pLineSerie3->SetName(_T("line 3"));
	
	pLineSerie4 = m_ChartCtrl.CreateLineSerie();
	pLineSerie4->SetSeriesOrdering(poNoOrdering);//设置为无序  
	pLineSerie4->SetColor(RGB(255,100,0));
	pLineSerie4->SetName(_T("line 4"));
	
	pLineSerie5 = m_ChartCtrl.CreateLineSerie();
	pLineSerie5->SetSeriesOrdering(poNoOrdering);//设置为无序  
	pLineSerie5->SetColor(RGB(0,255,255));
	pLineSerie5->SetName(_T("line 5"));
	
	pLineSerie6 = m_ChartCtrl.CreateLineSerie();
	pLineSerie6->SetSeriesOrdering(poNoOrdering);//设置为无序  
	pLineSerie6->SetColor(RGB(0,0,0));
	pLineSerie6->SetName(_T("line 6"));

	memset(sensor_data, 0, sizeof(sensor_data));
	for (int i=0; i<QST_CHART_MAX_POINT; i++)
	{
		b_data[i] = i;
		line1_data[i] = 0.0;
		line2_data[i] = 0.0;
		line3_data[i] = 0.0;
		line4_data[i] = 0.0;
		line5_data[i] = 0.0;
		line6_data[i] = 0.0;
	}
	m_ChartCtrl.EnableRefresh(true);
	m_ChartCtrl_2.ShowWindow(SW_HIDE);
	m_ChartCtrl_3.ShowWindow(SW_HIDE);
	
	chart_index = 1;
}

void Ccomm_demoDlg::app_data_update(void)
{
	CString sRawData;
	CStatic *pRawDataEdit;
	char str_buf[256]={0};

#if !defined(CHART_REFRESH_DISABLE)
	if(sensor_type != QST_SENSOR_NONE)
	{
		if(chart_index < QST_CHART_MAX_POINT)
		{
			b_data[chart_index] = chart_index;
			line1_data[chart_index] = sensor_data[0];
			line2_data[chart_index] = sensor_data[1];
			line3_data[chart_index] = sensor_data[2];
			line4_data[chart_index] = sensor_data[3];
			line5_data[chart_index] = sensor_data[4];
			line6_data[chart_index] = sensor_data[5];
			if(line_set&QST_LINE1_ENABLE)
				pLineSerie1->SetPoints(b_data, line1_data, chart_index);
			if(line_set&QST_LINE2_ENABLE)
				pLineSerie2->SetPoints(b_data, line2_data, chart_index);
			if(line_set&QST_LINE3_ENABLE)
				pLineSerie3->SetPoints(b_data, line3_data, chart_index);
			if(line_set&QST_LINE4_ENABLE)
				pLineSerie4->SetPoints(b_data, line4_data, chart_index);
			if(line_set&QST_LINE5_ENABLE)
				pLineSerie5->SetPoints(b_data, line5_data, chart_index);
			if(line_set&QST_LINE6_ENABLE)
				pLineSerie6->SetPoints(b_data, line6_data, chart_index);
		}
		else
		{
			for(int i=0; i<QST_CHART_MAX_POINT-1; i++)
			{
				b_data[i] = b_data[i+1];
				line1_data[i] = line1_data[i+1];
				line2_data[i] = line2_data[i+1];
				line3_data[i] = line3_data[i+1];				
				line4_data[i] = line4_data[i+1];
				line5_data[i] = line5_data[i+1];
				line6_data[i] = line6_data[i+1];
			}
			b_data[QST_CHART_MAX_POINT-1] = chart_index;
			line1_data[QST_CHART_MAX_POINT-1] = sensor_data[0];
			line2_data[QST_CHART_MAX_POINT-1] = sensor_data[1];
			line3_data[QST_CHART_MAX_POINT-1] = sensor_data[2];
			line4_data[QST_CHART_MAX_POINT-1] = sensor_data[3];
			line5_data[QST_CHART_MAX_POINT-1] = sensor_data[4];
			line6_data[QST_CHART_MAX_POINT-1] = sensor_data[5];
			if(line_set&QST_LINE1_ENABLE)
				pLineSerie1->SetPoints(b_data, line1_data, QST_CHART_MAX_POINT);
			if(line_set&QST_LINE2_ENABLE)
				pLineSerie2->SetPoints(b_data, line2_data, QST_CHART_MAX_POINT);
			if(line_set&QST_LINE3_ENABLE)
				pLineSerie3->SetPoints(b_data, line3_data, QST_CHART_MAX_POINT);
			if(line_set&QST_LINE4_ENABLE)
				pLineSerie4->SetPoints(b_data, line4_data, QST_CHART_MAX_POINT);
			if(line_set&QST_LINE5_ENABLE)
				pLineSerie5->SetPoints(b_data, line5_data, QST_CHART_MAX_POINT);
			if(line_set&QST_LINE6_ENABLE)
				pLineSerie6->SetPoints(b_data, line6_data, QST_CHART_MAX_POINT);
		}
	}
	chart_index++;
#endif

	if(QST_SENSOR_ACCEL & sensor_type)
	{
		if(log_flag)
		{
			memset(str_buf, 0, sizeof(str_buf));
			sprintf_s(str_buf, "%f	%f	%f \r\n",sensor_data[0],sensor_data[1],sensor_data[2]);
			log_file.Write(str_buf, strlen(str_buf));
		}
		sRawData.Empty();
		sRawData.Format(L"Accelerator:\n x: %f\n y: %f \n z: %f \n", sensor_data[0],sensor_data[1],sensor_data[2]);
		pRawDataEdit = (CStatic*)GetDlgItem(IDC_STATIC_ACCDATA);
		pRawDataEdit->SetWindowTextW(sRawData.GetString());
#if defined(QMAX981_STEPCOUNTER)
		sRawData.Empty();
		sRawData.Format(L"Step:\n %d", step);
		pRawDataEdit = (CStatic*)GetDlgItem(IDC_STATIC_STEPCOUNTER);
		pRawDataEdit->SetWindowTextW(sRawData.GetString());
#endif
	}
	if(QST_SENSOR_MAG & sensor_type)
	{
		if(log_flag)
		{
			memset(str_buf, 0, sizeof(str_buf));
			sprintf_s(str_buf, "QMC:	%f	%f	%f \r\n", sensor_data[3],sensor_data[4],sensor_data[5]);
			log_file.Write(str_buf, strlen(str_buf));
		}
		sRawData.Empty();
		if(mag_accuracy == 3)
			sRawData.Format(L"Magnetic(Cali):\n x: %f\n y: %f \n z: %f", sensor_data[3],sensor_data[4],sensor_data[5]);
		else
			sRawData.Format(L"Magnetic(not Cali):\n x: %f\n y: %f \n z: %f \n", sensor_data[3],sensor_data[4],sensor_data[5]);
			
		pRawDataEdit = (CStatic*)GetDlgItem(IDC_STATIC_MAGDATA);
		pRawDataEdit->SetWindowTextW(sRawData.GetBuffer(sRawData.GetLength()));

#if defined(QMC_CALI_2D_SUPPORT)
		sRawData.Empty();
		sRawData.Format(L"Orientation:\n %f", (atan2(sensor_data[1], sensor_data[0])*180/3.1415926)+180);
		pRawDataEdit = (CStatic*)GetDlgItem(IDC_STATIC_STEPCOUNTER);
		pRawDataEdit->SetWindowTextW(sRawData.GetString());
#endif
	}

	if(QST_SENSOR_GYRO & sensor_type)
	{
		if(log_flag)
		{
			memset(str_buf, 0, sizeof(str_buf));
			sprintf_s(str_buf, "Gyro:	%f	%f	%f \r\n", sensor_data[6],sensor_data[7],sensor_data[8]);
			log_file.Write(str_buf, strlen(str_buf));
		}
		sRawData.Empty();
		//sRawData.Format(L"陀螺仪:\n x: %f\n y: %f \n z: %f \n", gyro_xyz[0]*PI/180,gyro_xyz[1]*PI/180,gyro_xyz[2]*PI/180);
		sRawData.Format(L"Gyroscope:\n x: %f\n y: %f \n z: %f \n", sensor_data[6],sensor_data[7],sensor_data[8]);
		pRawDataEdit = (CStatic*)GetDlgItem(IDC_STATIC_PRESSDATA);
		pRawDataEdit->SetWindowTextW(sRawData.GetBuffer(sRawData.GetLength()));
	}
	else if(QST_SENSOR_PRESS & sensor_type)
	{
		if(log_flag)
		{
			memset(str_buf, 0, sizeof(str_buf));
			GetLocalTime(&qst_time);
			sprintf_s(str_buf, "[%02d:%02d:%02d.%03d] PRESS:	%f	%f \r\n", 
				qst_time.wHour,qst_time.wMinute,qst_time.wSecond,qst_time.wMilliseconds,
				sensor_data[0], sensor_data[1]);
			log_file.Write(str_buf, strlen(str_buf));
		}
		sRawData.Empty();
		sRawData.Format(L"Press: %f\nTemp: %f ", sensor_data[0], sensor_data[1]);
		pRawDataEdit = (CStatic*)GetDlgItem(IDC_STATIC_PRESSDATA);
		pRawDataEdit->SetWindowTextW(sRawData.GetBuffer(sRawData.GetLength()));
	}
	if(QST_SENSOR_ACCGYRO & sensor_type)
	{
		if(log_flag)
		{
			memset(str_buf, 0, sizeof(str_buf));
			sprintf_s(str_buf, "%f	%f	%f	%f	%f	%f \r\n",sensor_data[0],sensor_data[1],sensor_data[2],
															sensor_data[6],sensor_data[7],sensor_data[8]);
			log_file.Write(str_buf, strlen(str_buf));
		}
		sRawData.Empty();
		sRawData.Format(L"Accelerator:\n x: %f\n y: %f \n z: %f \n", sensor_data[0],sensor_data[1],sensor_data[2]);
		pRawDataEdit = (CStatic*)GetDlgItem(IDC_STATIC_ACCDATA);
		pRawDataEdit->SetWindowTextW(sRawData.GetString());
		sRawData.Empty();
		sRawData.Format(L"Gyroscope:\n x: %f\n y: %f \n z: %f \n", sensor_data[6],sensor_data[7],sensor_data[8]);
		pRawDataEdit = (CStatic*)GetDlgItem(IDC_STATIC_PRESSDATA);
		pRawDataEdit->SetWindowTextW(sRawData.GetBuffer(sRawData.GetLength()));
	}
	sRawData.ReleaseBuffer();
}


/*
void Ccomm_demoDlg::enum_com_port(void)
{
	CString port_name;
	CString list_str;
	int port, index;

	mPortList = (CComboBox*)GetDlgItem(IDC_COMBO_COM_LIST);
	mPortList->ResetContent();
	index = 0;
	port = 0;
	for(port=0; port<256; port++)
	{
		port_name.Empty();
		port_name.Format(L"\\\\.\\COM%d", port);
		if(hCom == INVALID_HANDLE_VALUE)
		{
			hCom = CreateFile(port_name.GetString(),
				GENERIC_READ | GENERIC_WRITE,
				0,
				NULL,
				OPEN_EXISTING,
				FILE_ATTRIBUTE_NORMAL,
				NULL);
		}
		
		if(hCom != INVALID_HANDLE_VALUE)
		{
			list_str.Empty();
			list_str.Format(L"COM%d", port);
			mPortList->AddString(list_str.GetString());
			mComlist[index++] = port;
			master_status = CloseHandle(hCom);
			hCom = INVALID_HANDLE_VALUE;
		}
	}
}

BOOL Ccomm_demoDlg::open_comm(int prot)
{
	DWORD len=0;
	unsigned char	buf_wr[32];
	CString port_name;
	COMMTIMEOUTS timeouts;
	int retry_count = 0;

	port_name.Empty();
	port_name.Format(L"\\\\.\\COM%d", prot);
	if(hCom == INVALID_HANDLE_VALUE)
	{
		hCom = CreateFile(port_name.GetString(),
			GENERIC_READ | GENERIC_WRITE,
			0,
			NULL,
			OPEN_EXISTING,
			FILE_ATTRIBUTE_NORMAL,
			NULL);
	}

	if(hCom == INVALID_HANDLE_VALUE)
	{
		QST_PRINTF("CreateFile Fail!\n");
		AfxMessageBox(_T("Open serial port fail!"), MB_OK, MB_ICONINFORMATION);
		return false;
	}
	else
	{
		QST_PRINTF("CreateFile Success\n");
		SetupComm(hCom, 1024, 1024);
		GetCommState(hCom, &dcb);

		dcb.BaudRate = 9600;	//9600;
		dcb.ByteSize = 8;
		dcb.Parity = NOPARITY;
		dcb.StopBits = ONESTOPBIT;
		if (!SetCommState(hCom, &dcb))
		{
			QST_PRINTF("Serial parameter fail\n");
			AfxMessageBox(_T("Serial parameter fail"), MB_OK, MB_ICONINFORMATION);
			return FALSE;
		}
		timeouts.ReadIntervalTimeout = 300;  
		timeouts.ReadTotalTimeoutMultiplier = 5;  
		timeouts.ReadTotalTimeoutConstant = 3;  
		timeouts.WriteTotalTimeoutMultiplier = 300;  
		timeouts.WriteTotalTimeoutConstant = 3;  
		if (!SetCommTimeouts(hCom, &timeouts))
		{
			QST_PRINTF("setting port time-outs error! \n");
			return FALSE;
		}
		//EscapeCommFunction(hCom, CLRDTR);
		Sleep(20);
		master_status = PurgeComm(hCom, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);
		Sleep(20);
		if(!master_status)
		{
			QST_PRINTF("PurgeComm error! \n");
			return FALSE;
		}
		QST_PRINTF("Serial Ok\n");
		buf_wr[0] = 0x49;
		buf_wr[1] = 0x50;
		master_status = WriteFile(hCom, buf_wr, 2, &len, NULL);
		if(master_status&&(len==2))
		{
			retry_count = 0;
			while(retry_count<1000)
			{
				master_status = ReadFile(hCom, buf_wr, 1, &len, NULL);
				if(master_status)
				{
					QST_PRINTF("read master_status=%d, data=0x%x \n",master_status, buf_wr[0]);
					break;
				}
				retry_count++;
			}
			if(buf_wr[0] == 0x40)
			{
				QST_PRINTF("CP2102 usb to i2c controler communication OK! \n");
			}
		}
		else
		{
			QST_PRINTF("WriteFile error \n");			
			return FALSE;
		}

		return TRUE;
	}
}
*/


void Ccomm_demoDlg::app_exit(void)
{
	KillTimer(SENSOR_TIMER_ID_1);
	app_close_file();
	usb_close_device();
	com_close_device();
	app_reset_para();
	app_refresh_ui(QST_SENSOR_NONE);
}

void Ccomm_demoDlg::app_open_file(void)
{
	CButton  *pCheckBox;

	pCheckBox = (CButton  *)GetDlgItem(IDC_CHECK_SAVEDATA);
	QST_PRINTF("Save data check : %d \n", pCheckBox->GetCheck());
	if((log_flag == FALSE)&&(pCheckBox->GetCheck()))
	{
		SYSTEMTIME st;
		CString file_name;
		GetLocalTime(&st);	 
		QST_PRINTF("%d-%d-%d-%02d-%02d-%02d", st.wYear,st.wMonth,st.wDay,st.wHour,st.wMinute,st.wSecond);
		file_name.Format(L"%d%d%d-%02d%02d%02d.txt", st.wYear,st.wMonth,st.wDay,st.wHour,st.wMinute,st.wSecond);
		log_flag = log_file.Open(file_name.GetString(), CFile::modeCreate | CFile::modeWrite);
		QST_PRINTF("log_flag=%d  %d\n", log_flag, log_file.m_hFile);
	}
}


void Ccomm_demoDlg::app_close_file(void)
{
	if((log_flag == TRUE)&&(log_file.m_hFile!=CFile::hFileNull))
	{
		log_file.Close();
		log_flag = FALSE;
	}
}

void Ccomm_demoDlg::app_write_file(char *str_buf, unsigned int len)
{
	if(log_flag == TRUE)
	{
		log_file.Write(str_buf, len);
	}
}

void Ccomm_demoDlg::OnBnClickedBtOpenDevice()
{
	BOOL init_flag = FALSE;
	unsigned char dev_addr[4] = {0x00, 0x00, 0x00, 0x00};
	int index = 0;
	// TODO: 在此添加控件通知处理程序代码

	if(master_connect == FALSE)
	{
		if((master_type==DEVICE_CH341A)||(master_type==DEVICE_STM32F103))
		{
			if(IsDlgButtonChecked(IDC_CHECK_PROTOCOL))
			{
				protocol_type = USB_SPI;
				master_connect = spi_init(master_type, SPI_SubMode_0);
			}
			else
			{
				protocol_type = USB_I2C;
				master_connect = i2c_init(master_type, 400*1000);
			}
		}
		else if((master_type == DEVICE_NONE)||(master_type == DEVICE_CP2102))
		{
			master_type = com_open_device(mComNum);
			if(master_type==DEVICE_CP2102)
			{
				master_connect = TRUE;
			}
		}
		if(master_connect)
			app_open_file();

		app_refresh_ui(sensor_type);
	}
	else		// close port
	{
		app_exit();
	}
}

void Ccomm_demoDlg::OnBnClickedBtStart()
{
	// TODO: 在此添加控件通知处理程序代码
	if (master_start == FALSE)
	{
		if(master_connect)
		{
			sensor_type = QST_SENSOR_NONE;
			line_set = QST_LINE_NONE;
#if defined(QST_ACCELEMRTER_SUPPORT)
			if(qmaX981_init())
			{
				sensor_type |= QST_SENSOR_ACCEL;
				acc_type = QST_ACCEL_QMAX981;
				edit_slave = qmaX981_get_slave();
				device_register_irq(qmaX981_irq_hdlr);
			}
			else if(qma6100_init())
			{
				sensor_type |= QST_SENSOR_ACCEL;
				acc_type = QST_ACCEL_QMA6100;
				edit_slave = qma6100_get_slave();
				device_register_irq(qma6100_irq_hdlr);
			}
			else if(lis3dh_init())
			{
				sensor_type |= QST_SENSOR_ACCEL;
				acc_type = QST_ACCEL_LIS3DH;
				edit_slave = lis3dh_get_slave();
			}
			else if(bma25x_init())
			{
				sensor_type |= QST_SENSOR_ACCEL;
				acc_type = QST_ACCEL_BMA25X;
				edit_slave = bma25x_get_slave();
			}
			else
			{
				acc_type = QST_ACCEL_NONE;
			}

			if(acc_type > QST_ACCEL_NONE)
				line_set |= (QST_LINE1_ENABLE|QST_LINE2_ENABLE|QST_LINE3_ENABLE);
#endif
#if defined(QST_MAGNETIC_SUPPORT)
			if(qmcX983_init())
			{
				sensor_type |= QST_SENSOR_MAG;
				mag_type = QST_MAG_QMC7983;
			}			
			else if(qmc6308_init())
			{
				sensor_type |= QST_SENSOR_MAG;
				mag_type = QST_MAG_QMC6308;
			}			
			else if(Qmc5883_InitConfig())
			{
				sensor_type |= QST_SENSOR_MAG;
				mag_type = QST_MAG_QMC5883L;
			}
			else
			{
				mag_type = QST_MAG_NONE;
			}

			if(mag_type > QST_MAG_NONE)
				//line_set |= (QST_LINE1_ENABLE|QST_LINE2_ENABLE|QST_LINE3_ENABLE);
				//line_set |= (QST_LINE4_ENABLE|QST_LINE5_ENABLE|QST_LINE6_ENABLE);
#endif
#if defined(QST_PRESSURE_SUPPORT)
			if(qmp6988_init(&qmp6988))
			{
				sensor_type |= QST_SENSOR_PRESS;
				press_type = QST_PRESS_QMP6988;
			}
			else if(bmp280_init())
			{
				sensor_type |= QST_SENSOR_PRESS;
				press_type = QST_PRESS_BMP280;
			}
			else
			{
				press_type = QST_PRESS_NONE;
			}
			if(press_type > QST_PRESS_NONE)
				line_set |= QST_LINE1_ENABLE;
#endif
#if defined(QST_ACCGYRO_SUPPORT)
			accgyro_type = QST_ACCGYRO_NONE;
			if(accgyro_type == QST_ACCGYRO_NONE)
			{
				if(protocol_type == USB_SPI)
					spi_config(4*1000*1000, 3);

				if(qmi8610_init())
				{
					sensor_type |= QST_SENSOR_ACCGYRO;
					accgyro_type = QST_ACCGYRO_QMI8610;
				}
				else if(Qmi8658_init())
				{
					sensor_type |= QST_SENSOR_ACCGYRO;
					accgyro_type = QST_ACCGYRO_QMI8658;
				}
			}
			if(accgyro_type == QST_ACCGYRO_NONE)
			{
				if(bmi160_init())
				{
					sensor_type |= QST_SENSOR_ACCGYRO;
					accgyro_type = QST_ACCGYRO_BMI160;
				}
				else if(mpu6050_init())
				{
					sensor_type |= QST_SENSOR_ACCGYRO;
					accgyro_type = QST_ACCGYRO_MPU6050;
				}
			}

			if(accgyro_type > QST_ACCGYRO_NONE)
				line_set |= (QST_LINE1_ENABLE|QST_LINE2_ENABLE|QST_LINE3_ENABLE);
#endif
			Sleep(300);
			if(sensor_type != QST_SENSOR_NONE)
			{
				master_start = TRUE;
				app_refresh_ui(sensor_type);
				SetTimer(SENSOR_TIMER_ID_1, sampleRate, NULL);
			}
		}
	}
	else
	{
		master_start = FALSE;
		app_refresh_ui(sensor_type);
		KillTimer(SENSOR_TIMER_ID_1);
	}
}

void Ccomm_demoDlg::OnBnClickedOk()
{
	// TODO: 在此添加控件通知处理程序代码
	OnOK();
}


void Ccomm_demoDlg::OnEnChangeCommPort()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialog::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
	CString sPort;

	GetDlgItem(IDC_EDIT_PORT)->GetWindowTextW(sPort);
	wchar_t *p = sPort.GetBuffer(sPort.GetLength()); 
	mComNum = _wtoi(p);
	sPort.ReleaseBuffer();
	QST_PRINTF("OnEnChangeCommPort mComNum=%d \n", mComNum);
}

BOOL Ccomm_demoDlg::OnDeviceChange(UINT nEventType, DWORD dwData)
{
	QST_PRINTF("nEventType = %d dwData=%ld \n", nEventType, dwData);
	switch (nEventType)  
	{  
	case DBT_DEVICEREMOVECOMPLETE://移除设备
		break;
	case DBT_DEVICEARRIVAL://添加设备  
		break;  

	default:  
		break;  
	}

	usb_close_device();
	com_close_device();
	app_reset_para();
	master_type = usb_open_device();
	usb_close_device();
	app_refresh_ui(sensor_type);

	return TRUE;
}

void Ccomm_demoDlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	//sensor_handle_int();
	if(nIDEvent == SENSOR_TIMER_ID_1)
	{
#if defined(QST_ACCELEMRTER_SUPPORT)
		if(sensor_type & QST_SENSOR_ACCEL)
		{
			if(acc_type == QST_ACCEL_QMAX981)
			{
				qmaX981_read_xyz(&sensor_data[0]);
	#if defined(QMAX981_STEPCOUNTER)
				step = qmaX981_read_stepcounter();
	#endif
			}
			else if(acc_type == QST_ACCEL_QMA6100)
			{
				qma6100_read_acc_xyz(&sensor_data[0]);
	#if defined(QMA6100_STEPCOUNTER)
				step = qma6100_read_stepcounter();
	#endif
			}
			else if(acc_type == QST_ACCEL_LIS3DH)
			{
				lis3dh_read_acc_xyz(&sensor_data[0]);
			}
			else if(acc_type == QST_ACCEL_BMA25X)
			{
				bma25x_read_xyz(&sensor_data[0]);
			}
#if defined(QST_MAG_CALI_SUPPORT)
			struct magCaliDataInPut accIn;
			accIn.status = 3;
			accIn.timeStamp = 0;
			accIn.x = sensor_data[0];
			accIn.y = sensor_data[1];
			accIn.z = sensor_data[2];
			mag_cali_p->caliApiSetAccData(&accIn);
#endif
		}
#endif
#if defined(QST_MAGNETIC_SUPPORT)
		if(sensor_type & QST_SENSOR_MAG)
		{
			if(mag_type == QST_MAG_QMC7983)
			{
				qmcX983_read_mag_xyz(&sensor_data[3]);
			}
			else if(mag_type == QST_MAG_QMC6308)
			{
				qmc6308_read_mag_xyz(&sensor_data[3]);
			}
			else if(mag_type == QST_MAG_QMC5883L)
			{
				Qmc5883_GetData(&sensor_data[3]);				
			}
#if defined(QST_MAG_CALI_SUPPORT)
			struct magCaliDataInPut magIn;
			struct magCaliDataOutPut magOut;

			magIn.status = 0;
			magIn.timeStamp = 0;
			magIn.x = sensor_data[3];
			magIn.y = sensor_data[4];
			magIn.z = sensor_data[5];
			mag_cali_p->doCaliApi(&magIn, &magOut);
			sensor_data[3] = magOut.x;
			sensor_data[4] = magOut.y;
			sensor_data[5] = magOut.z;
			mag_accuracy = magOut.status;
#endif
		}
#endif
#if defined(QST_PRESSURE_SUPPORT)
		if(sensor_type & QST_SENSOR_PRESS)
		{
			if(press_type == QST_PRESS_QMP6988)
			{
				qmp6988_calc_pressure(&qmp6988, &sensor_data[1], &sensor_data[0]);
				sensor_data[2] = ExeBtwzFilter(sensor_data[0]);				
				line_set |= QST_LINE3_ENABLE;
			}
			else if(press_type == QST_PRESS_BMP280)
			{
				bmp280_calc_press(&sensor_data[0], &sensor_data[1]);
			}
		}
#endif
#if defined(QST_ACCGYRO_SUPPORT)
		if(sensor_type & QST_SENSOR_ACCGYRO)
		{
			if(accgyro_type == QST_ACCGYRO_QMI8610)
			{
				qmi8610_read_xyz(&sensor_data[0], &sensor_data[6], NULL);
			}
			else if(accgyro_type == QST_ACCGYRO_QMI8658)
			{
				Qmi8658_read_xyz(&sensor_data[0], &sensor_data[6], NULL);
			}
			else if(accgyro_type == QST_ACCGYRO_BMI160)
			{
				bmi160_read_xyz(&sensor_data[0], &sensor_data[6]);
			}
			else if(accgyro_type == QST_ACCGYRO_MPU6050)
			{
				mpu6050_read_data(&sensor_data[0], &sensor_data[6], &sensor_data[10]);
			}
		}
#endif
		app_data_update();
	}
}


void Ccomm_demoDlg::OnClose()
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	QST_PRINTF("close dlg! \n");

	app_exit();
#ifdef DEBUG_CONSOLE
	CloseConsoleWindow();
#endif
	CDialog::OnClose();
}


void Ccomm_demoDlg::OnDestroy()
{
	QST_PRINTF("destory dlg! \n");

	app_exit();
#ifdef DEBUG_CONSOLE
	CloseConsoleWindow();
#endif
	CDialog::OnDestroy();

	// TODO: 在此处添加消息处理程序代码
}

#if 0
HBRUSH Ccomm_demoDlg::OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor)
{
	HBRUSH hbr = CDialog::OnCtlColor(pDC, pWnd, nCtlColor);

	// TODO:  在此更改 DC 的任何特性
	//QST_PRINTF("nCtlColor=%d \n", nCtlColor);
	//if(nCtlColor == CTLCOLOR_STATIC)
	//{
	//	pDC->SetTextColor(RGB(0,255,0));
	//}
	if(pWnd->GetDlgCtrlID() == IDC_STATIC_STEPCOUNTER)
	{
		pDC->SetTextColor(RGB(0,128,252));
		pDC->SetBkMode(TRANSPARENT);
	}
	// TODO:  如果默认的不是所需画笔，则返回另一个画笔
	return hbr;
}
#endif

void Ccomm_demoDlg::OnBnClickedButtonRegWrite()
{
	// TODO: 在此添加控件通知处理程序代码
	CEdit *pInput;
	CString str_input;
	unsigned char reg_addr, reg_value;
	int len, index;
	char buff_out[10];

	memset(buff_out, 0, sizeof(buff_out));
	str_input.Empty();
	pInput = (CEdit*)GetDlgItem(IDC_EDIT_REG_ADDR);
	pInput->GetWindowTextW(str_input);
	len = str_input.GetLength();
	if(len > 2/*sizeof(buff_out)-1*/)
	{
		AfxMessageBox(_T("Input error!"), MB_OK, MB_ICONINFORMATION);
		str_input.Empty();
		pInput->SetWindowTextW(str_input.GetString());
		return;
	}
	WideCharToMultiByte(CP_ACP, 0, str_input, len,  buff_out, len, NULL, NULL);
	reg_addr = 0;
	index = 0;
	while(len > 0)
	{
		len--;
		if(buff_out[len] >='0' && buff_out[len]<='9')
		{
			reg_addr += (buff_out[len]-'0')*((int)pow(16.00, index));
		}
		else if(buff_out[len] >='A' && buff_out[len]<='F')
		{
			reg_addr += (buff_out[len]-'A'+10)*((int)pow(16.00, index));
		}
		else if(buff_out[len] >='a' && buff_out[len]<='f')
		{
			reg_addr += (buff_out[len]-'a'+10)*((int)pow(16.00, index));
		}
		else
		{
			AfxMessageBox(_T("Input error!"), MB_OK, MB_ICONINFORMATION);
			str_input.Empty();
			pInput->SetWindowTextW(str_input.GetString());
			return;
		}
		index++;
	}
	QST_PRINTF("reg_addr = 0x%x\n", reg_addr);

	memset(buff_out, 0, sizeof(buff_out));
	str_input.Empty();
	pInput = (CEdit*)GetDlgItem(IDC_EDIT_REG_VALUE);
	pInput->GetWindowTextW(str_input);
	len = str_input.GetLength();
	if(len > 2/*sizeof(buff_out)-1*/)
	{
		AfxMessageBox(_T("Input error!"), MB_OK, MB_ICONINFORMATION);
		str_input.Empty();
		pInput->SetWindowTextW(str_input.GetString());
		return;
	}
	WideCharToMultiByte(CP_ACP, 0, str_input, len,  buff_out, len, NULL, NULL);
	reg_value = 0;
	index = 0;
	while(len > 0)
	{
		len--;
		if(buff_out[len] >='0' && buff_out[len]<='9')
		{
			reg_value += (buff_out[len]-'0')*((int)pow(16.00, index));
		}
		else if(buff_out[len] >='A' && buff_out[len]<='F')
		{
			reg_value += (buff_out[len]-'A'+10)*((int)pow(16.00, index));
		}
		else if(buff_out[len] >='a' && buff_out[len]<='f')
		{
			reg_value += (buff_out[len]-'a'+10)*((int)pow(16.00, index));
		}
		else
		{
			AfxMessageBox(_T("Input error!"), MB_OK, MB_ICONINFORMATION);
			str_input.Empty();
			pInput->SetWindowTextW(str_input.GetString());
			return;
		}
		index++;
	}
	QST_PRINTF("reg_value = 0x%x\n", reg_value);
	if(protocol_type == USB_I2C)
	{
		i2c_write_reg(edit_slave, reg_addr, reg_value);
	}
	else if(protocol_type == USB_SPI)
	{
		if(sensor_type & QST_SENSOR_ACCEL)
		{
			if(acc_type == QST_ACCEL_QMA6100)
			{			
				qma6100_writereg(reg_addr, reg_value);
			}
			else if(acc_type == QST_ACCEL_QMAX981)
			{
				qmaX981_writereg(reg_addr, reg_value);
			}
			else if(acc_type == QST_ACCEL_LIS3DH)
			{
				lis3dh_write_reg(reg_addr, reg_value);
			}
		}		
		if(sensor_type & QST_SENSOR_MAG)
		{			
			if(mag_type == QST_MAG_QMC7983)
			{
				qmcX983_WriteReg(reg_addr, reg_value);
			}
			else if(mag_type == QST_MAG_QMC6308)
			{
				qmc6308_write_reg(reg_addr, reg_value);
			}
			else if(mag_type == QST_MAG_QMC5883L)
			{
				Qmc5883_WriteReg(reg_addr, reg_value);
			}
		}		
		if(sensor_type & QST_SENSOR_PRESS)
		{
			if(press_type == QST_PRESS_QMP6988)
			{
				qmp6988_WriteReg(0x70,reg_addr, reg_value);
			}
		}		
		if(sensor_type & QST_SENSOR_ACCGYRO)
		{
			if(accgyro_type == QST_ACCGYRO_QMI8610)
			{
				qmi8610_write_reg(reg_addr, reg_value);
			}
			else if(accgyro_type == QST_ACCGYRO_QMI8658)
			{
				Qmi8658_write_reg(reg_addr, reg_value);
			}
			else if(accgyro_type == QST_ACCGYRO_BMI160)
			{
				bmi160_write_reg(reg_addr, reg_value);
			}
		}
	}
}

void Ccomm_demoDlg::OnBnClickedButtonRegRead()
{
	// TODO: 在此添加控件通知处理程序代码
	CEdit *pInput;
	CString str_input;
	unsigned char reg_addr, reg_value;
	int len, index;
	char buff_out[10];

	memset(buff_out, 0, sizeof(buff_out));
	str_input.Empty();
	pInput = (CEdit*)GetDlgItem(IDC_EDIT_REG_ADDR);
	pInput->GetWindowTextW(str_input);
	len = str_input.GetLength();
	if(len > 2/*sizeof(buff_out)-1*/)
	{
		AfxMessageBox(_T("Input error!"), MB_OK, MB_ICONINFORMATION);
		return;
	}
	WideCharToMultiByte(CP_ACP, 0, str_input, len,  buff_out, len, NULL, NULL);
	reg_addr = 0;
	index = 0;
	while(len > 0)
	{
		len--;
		if(buff_out[len] >='0' && buff_out[len]<='9')
		{
			reg_addr += (buff_out[len]-'0')*((int)pow(16.00, index));
		}
		else if(buff_out[len] >='A' && buff_out[len]<='F')
		{
			reg_addr += (buff_out[len]-'A'+10)*((int)pow(16.00, index));
		}
		else if(buff_out[len] >='a' && buff_out[len]<='f')
		{
			reg_addr += (buff_out[len]-'a'+10)*((int)pow(16.00, index));
		}
		else
		{
			AfxMessageBox(_T("Input error!"), MB_OK, MB_ICONINFORMATION);
			str_input.Empty();
			pInput->SetWindowTextW(str_input.GetString());
			return;
		}
		index++;
	}

	if(protocol_type == USB_I2C)
	{
		i2c_read_reg(edit_slave, reg_addr, &reg_value, 1);
	}
	else if(protocol_type == USB_SPI)
	{
		if(sensor_type & QST_SENSOR_ACCEL)
		{
			if(acc_type == QST_ACCEL_QMA6100)
			{			
				qma6100_readreg(reg_addr, &reg_value, 1);
			}
			else if(acc_type == QST_ACCEL_QMAX981)
			{
				qmaX981_readreg(reg_addr, &reg_value, 1);
			}
			else if(acc_type == QST_ACCEL_LIS3DH)
			{
				lis3dh_read_reg(reg_addr, &reg_value, 1);
			}
		}		
		if(sensor_type & QST_SENSOR_MAG)
		{			
			if(mag_type == QST_MAG_QMC7983)
			{
				qmcX983_ReadData(reg_addr, &reg_value, 1);
			}
			else if(mag_type == QST_MAG_QMC6308)
			{
				qmc6308_read_block(reg_addr, &reg_value, 1);
			}
			else if(mag_type == QST_MAG_QMC5883L)
			{
				Qmc5883_ReadReg(reg_addr, &reg_value, 1);
			}
		}		
		if(sensor_type & QST_SENSOR_PRESS)
		{
			if(press_type == QST_PRESS_QMP6988)
			{
				qmp6988_ReadData(0x70, reg_addr, &reg_value, 1);
			}
		}		
		if(sensor_type & QST_SENSOR_ACCGYRO)
		{
			if(accgyro_type == QST_ACCGYRO_QMI8610)
			{
				qmi8610_read_reg(reg_addr, &reg_value, 1);
			}
			else if(accgyro_type == QST_ACCGYRO_QMI8658)
			{
				Qmi8658_read_reg(reg_addr, &reg_value, 1);
			}
			else if(accgyro_type == QST_ACCGYRO_BMI160)
			{
				bmi160_read_reg(reg_addr, &reg_value, 1);
			}
		}
	}

	QST_PRINTF("reg_addr = 0x%x\n", reg_addr);
	QST_PRINTF("reg_value = 0x%x\n", reg_value);
	memset(buff_out, 0, sizeof(buff_out));
	wsprintf((wchar_t*)buff_out, L"%02x", reg_value);
	pInput = (CEdit*)GetDlgItem(IDC_EDIT_REG_VALUE);
	pInput->SetWindowTextW((wchar_t*)buff_out);
}

void Ccomm_demoDlg::OnEnChangeEditSlave()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialog::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
	CEdit *pInput;
	CString str_input;
	unsigned char reg_addr = 0;
	int len, index;
	char buff_out[10];

	memset(buff_out, 0, sizeof(buff_out));
	str_input.Empty();
	pInput = (CEdit*)GetDlgItem(IDC_EDIT_SLAVE);
	pInput->GetWindowTextW(str_input);
	len = str_input.GetLength();
	if(len > 2/*sizeof(buff_out)-1*/)
	{
		AfxMessageBox(_T("Slave addr too long"), MB_OK, MB_ICONINFORMATION);
		str_input.Empty();
		pInput->SetWindowTextW(str_input.GetString());
		return;
	}
	WideCharToMultiByte(CP_ACP, 0, str_input, len,  buff_out, len, NULL, NULL);
	reg_addr = 0;
	index = 0;
	while(len > 0)
	{
		len--;
		if(buff_out[len] >='0' && buff_out[len]<='9')
		{
			reg_addr += (buff_out[len]-'0')*((int)pow(16.00, index));
		}
		else if(buff_out[len] >='A' && buff_out[len]<='F')
		{
			reg_addr += (buff_out[len]-'A'+10)*((int)pow(16.00, index));
		}
		else if(buff_out[len] >='a' && buff_out[len]<='f')
		{
			reg_addr += (buff_out[len]-'a'+10)*((int)pow(16.00, index));
		}
		else
		{
			AfxMessageBox(_T("Input error!"), MB_OK, MB_ICONINFORMATION);
			str_input.Empty();
			pInput->SetWindowTextW(str_input.GetString());
			reg_addr = 0;
			return;
		}
		index++;
	}
	edit_slave = reg_addr;
	QST_PRINTF("Edit slave_addr=0x%02x\n", edit_slave);
}


void Ccomm_demoDlg::OnBnClickedButtonCali()
{
	// TODO: 在此添加控件通知处理程序代码
	QST_PRINTF("Cali button clicked! \n");
}


//void Ccomm_demoDlg::OnBnClickedCheckProtocol()
//{
//	// TODO: 在此添加控件通知处理程序代码
//}


void Ccomm_demoDlg::OnBnClickedButtonIoWrite()
{
	// TODO: 在此添加控件通知处理程序代码
	CEdit *pInput;
	CString str_input;
	unsigned char io_value;
	int len, index;
	char buff_out[10];

	if(master_connect == TRUE)
	{
		memset(buff_out, 0, sizeof(buff_out));
		str_input.Empty();
		pInput = (CEdit*)GetDlgItem(IDC_EDIT_IO);
		pInput->GetWindowTextW(str_input);
		len = str_input.GetLength();
		if(len > 2/*sizeof(buff_out)-1*/)
		{
			AfxMessageBox(_T("Input error!"), MB_OK, MB_ICONINFORMATION);
			str_input.Empty();
			pInput->SetWindowTextW(str_input.GetString());
			return;
		}
		WideCharToMultiByte(CP_ACP, 0, str_input, len,  buff_out, len, NULL, NULL);
		io_value = 0;
		index = 0;
		while(len > 0)
		{
			len--;
			if(buff_out[len] >='0' && buff_out[len]<='9')
			{
				io_value += (buff_out[len]-'0')*((int)pow(16.00, index));
			}
			else if(buff_out[len] >='A' && buff_out[len]<='F')
			{
				io_value += (buff_out[len]-'A'+10)*((int)pow(16.00, index));
			}
			else if(buff_out[len] >='a' && buff_out[len]<='f')
			{
				io_value += (buff_out[len]-'a'+10)*((int)pow(16.00, index));
			}
			else
			{
				AfxMessageBox(_T("Input error!"), MB_OK, MB_ICONINFORMATION);
				str_input.Empty();
				pInput->SetWindowTextW(str_input.GetString());
				return;
			}
			index++;
		}
		QST_PRINTF("io_value = 0x%x\n", io_value);
		device_write_io(io_value);
	}
}

//system("start explorer http://www.qstcorp.com/");
//ShellExecute(this->m_hWnd,L"open",L"mailto:zhiqiang_yang@qstcorp.com?subject=hello qst",L"",L"", SW_SHOW); 

