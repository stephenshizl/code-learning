// comm_demoDlg.h : 头文件
//

#pragma once
#include "afxwin.h"
#include <stdio.h>
#include <stdint.h>
#include <Dbt.h>
#include <io.h>
#include <fcntl.h> 
#include <time.h>
#include <math.h>
#include "conio.h"

#include "USBIOX.H"
#include "USB2UARTSPIIICDLL.h"
#include "ChartCtrl.h"
#include "ChartAxisLabel.h"
#include "ChartLineSerie.h"
#include "ChartBarSerie.h"
#include "MyFilter.h"
#include "usb_device.h"
// acc
#include "qmaX981.h"
#include "qma6100.h"
#include "lis3dh.h"
#include "bma25x.h"
// mag
#include "qmcX983.h"
#include "qmc6308.h"
#include "qmc5883.h"
// accgyro
#include "qmi8610.h"
#include "qmi8658.h"
#include "bmi160.h"
#include "mpu6050.h"
// press
#include "qmp6988.h"
#include "bmp280.h"

//typedef unsigned char uint8;
//typedef char int8;
//typedef unsigned short uint16;
//typedef short int16;
//typedef unsigned int uint32;
//typedef int int32;

#define SENSOR_TIMER_ID_1			1
#define SENSOR_TIMER_ID_2			2
#define SENSOR_TIMER_ID_3			3
#define QST_CHART_MAX_POINT			50		// draw chart
#define PI	3.1415926535897932f

#define QST_ACCELEMRTER_SUPPORT
#define QST_MAGNETIC_SUPPORT
//#define QST_MAG_CALI_SUPPORT
//#define QST_PRESSURE_SUPPORT
//#define QST_ACCGYRO_SUPPORT

#if defined(QST_MAG_CALI_SUPPORT)
#define CHART_REFRESH_DISABLE
#include "mag_calibration_lib.h"
extern "C" struct mag_lib_interface_t MAG_LIB_API_INTERFACE;
#endif

typedef struct
{
    char sign[3];
    unsigned char map[3];
}qst_convert;

typedef enum
{
	QST_SENSOR_NONE,
	QST_SENSOR_ACCEL = 1,
	QST_SENSOR_MAG = 1<<1,
	QST_SENSOR_GYRO = 1<<2,
	QST_SENSOR_PRESS = 1<<3,
	QST_SENSOR_LIGHT = 1<<4,	
	QST_SENSOR_ACCGYRO = 1<<5,

	QST_SENSOR_TOTAL
} qst_sensor_type;

typedef enum
{
	QST_ACCEL_NONE,
	QST_ACCEL_QMAX981,
	QST_ACCEL_QMA6100,
	QST_ACCEL_LIS3DH,
	QST_ACCEL_BMA25X,

	QST_ACCEL_TOTAL
} qst_acc_type;

typedef enum
{
	QST_MAG_NONE,
	QST_MAG_QMC7983,
	QST_MAG_QMC6308,
	QST_MAG_QMC6310,
	QST_MAG_QMC5883L,

	QST_MAG_TOTAL
} qst_mag_type;

typedef enum
{
	QST_GYRO_NONE,

	QST_GYRO_TOTAL
} qst_gyro_type;

typedef enum
{
	QST_ACCGYRO_NONE,
	QST_ACCGYRO_QMI8610,
	QST_ACCGYRO_QMI8658,
	QST_ACCGYRO_BMI160,
	QST_ACCGYRO_MPU6050,

	QST_ACCGYRO_TOTAL
} qst_accgyro_type;

typedef enum
{
	QST_PRESS_NONE,
	QST_PRESS_QMP6988,
	QST_PRESS_BMP280,

	QST_PRESS_TOTAL
} qst_press_type;


typedef enum
{
	QST_LINE_NONE,
	QST_LINE1_ENABLE = 1<<0,
	QST_LINE2_ENABLE = 1<<1,
	QST_LINE3_ENABLE = 1<<2,
	QST_LINE4_ENABLE = 1<<3,
	QST_LINE5_ENABLE = 1<<4,	
	QST_LINE6_ENABLE = 1<<5,

	QST_LINE_TOTAL = 0xff
} chart_line_set;

typedef struct
{
	union
	{
		float	sensor_data[10];
		struct
		{
			float acc_x;
			float acc_y;
			float acc_z;
			float gyr_x;
			float gyr_y;
			float gyr_z;
			float mag_x;
			float mag_y;
			float mag_z;
			float tempearture;
			float press;
		};
	};
	unsigned int step;
} sensor_vec_t;


// Ccomm_demoDlg 
class Ccomm_demoDlg : public CDialog
{
// 构造
public:
	Ccomm_demoDlg(CWnd* pParent = NULL);	// 标准构造函数

// 对话框数据
	enum { IDD = IDD_COMM_DEMO_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 支持


// 实现
protected:
	HICON m_hIcon;

	// 生成的消息映射函数
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedBtOpenDevice();
	afx_msg void OnBnClickedOk();
	//BOOL open_comm(int prot);

	void app_init_para(void);
	void app_reset_para(void);
	void app_init_ui(void);
	void app_refresh_ui(int type);
	void app_data_update(void);
	void app_chart_init(void);
	void app_exit(void);
	// log file
	void app_open_file(void);
	void app_close_file(void);
	void app_write_file(char *str_buf, unsigned int len);
// log file

private:
	SYSTEMTIME		qst_time;
	int				sampleRate;
	int				mag_accuracy;
 	int			 	mComNum;
	//sensor_vec_t	out_data;
	float			sensor_data[12];		// sensor_data[9]	: press   sensor_data[10]: tempearture   
	unsigned int	step;

#if defined(MAG_CALI_SUPPORT)
	struct magChipInfo qmcInfo;
#endif
#if defined(QMC_CALI_3D_SUPPORT)
	_QMC7983		cali_data;
#endif
	qmp6988_data	qmp6988;

	// master control
	int 			sensor_type; 
	qst_acc_type	acc_type;
	qst_mag_type	mag_type;
	qst_press_type	press_type;
	qst_accgyro_type	accgyro_type;
	
	unsigned char	edit_slave;
 	int				master_type;
	int				protocol_type;
	BOOL			master_status;
	BOOL			master_connect;
	BOOL			master_start;

	// draw chart
	CChartCtrl m_ChartCtrl;
	CChartCtrl m_ChartCtrl_2;
	CChartCtrl m_ChartCtrl_3;
	BOOL mChartInit;
	// line
	int				line_set;
	unsigned int	chart_index;
	CChartLineSerie *pLineSerie1;
	CChartLineSerie *pLineSerie2;
	CChartLineSerie *pLineSerie3;
	CChartLineSerie *pLineSerie4;
	CChartLineSerie *pLineSerie5;
	CChartLineSerie *pLineSerie6;
	double			b_data[QST_CHART_MAX_POINT];
	double			line1_data[QST_CHART_MAX_POINT];
	double			line2_data[QST_CHART_MAX_POINT];
	double			line3_data[QST_CHART_MAX_POINT];
	double			line4_data[QST_CHART_MAX_POINT];
	double			line5_data[QST_CHART_MAX_POINT];
	double			line6_data[QST_CHART_MAX_POINT];
	// draw chart

	//CFont m_textFont;
	//write file
	CFile			log_file;
	BOOL			log_flag;
	// write
#if defined(QST_MAG_CALI_SUPPORT)
	struct mag_lib_interface_t	*mag_cali_p;
#endif

 public:
	afx_msg void OnEnChangeCommPort();
	afx_msg BOOL OnDeviceChange(UINT nEventType, DWORD dwData);
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	afx_msg void OnClose();
	afx_msg void OnDestroy();
	//afx_msg HBRUSH OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor);
	afx_msg void OnBnClickedButtonRegWrite();
	afx_msg void OnBnClickedButtonRegRead();
	afx_msg void OnBnClickedButtonCic();
	afx_msg void OnEnChangeEditSlave();
	afx_msg void OnBnClickedButtonCali();
 	afx_msg void OnBnClickedButtonIoWrite();
	afx_msg void OnBnClickedBtStart();
};
