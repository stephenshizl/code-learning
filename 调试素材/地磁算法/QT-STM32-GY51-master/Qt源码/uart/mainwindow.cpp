#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    port = new QSerialPort;


    setWindowState(Qt::WindowMaximized);
}
MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::Read_Data()
{
    QByteArray arry;
    arry = port->readAll();
    if(!arry.isEmpty())
    {
        ui->textEdit->append(tr(arry));
        ui->textEdit->setText(ui->textEdit->toPlainText().remove('\n'));
    }
}

void MainWindow::on_pushButton_2_clicked()
{
    ui->textEdit->clear();
}

void MainWindow::paintEvent(QPaintEvent *)
{
    int width = this->width();//窗口宽度
    int height = this->height();//窗口高度

    QPainter painter(this);
    painter.translate(width/2, height/2);//原点移到中间

    QPixmap pix;//图形映射
    pix.load(":/compass/0.jpg");

    painter.rotate(-angle_current); //坐标轴逆时针旋转angle_current度
    painter.drawPixmap(-250, -250, 500, 500, pix);//绘制指南针
    painter.rotate(angle_current); //坐标轴还原

    QPen pen;
    pen.setColor(Qt::blue);
    painter.setPen(pen);
    painter.setBrush(QBrush(Qt::blue));

    painter.drawRect(-2,-150,3,100);
    //painter.translate(0,0); //使原点复原
 }
void MainWindow:: timerEvent(QTimerEvent *event)
{
    if(event->timerId() == this->timerID)
    {
        //每50ms取最新的30个字符
        QString str = ui->textEdit->toPlainText().right(30);
        ui->textEdit->clear();

        //解决包尾出现在包头前面的问题
        while((str.indexOf('@')>str.indexOf('#'))&&str.contains('@')&&str.contains('#'))
            str = str.remove(0,str.indexOf('@'));

        //解析数据包中数据
        if(str.contains('@')&&str.contains('#'))
        angle_current = str.mid(str.indexOf('@')+1,str.indexOf('#')-str.indexOf('@')-1).toInt();

        ui->label_2->setText(tr("角度：%1°").arg(angle_current));
        this->update();
    }
}


void MainWindow::on_pushButton_3_clicked()
{
    port->close();
    port->setPortName("COM3");

    port->open(QIODevice::ReadWrite);
    qDebug()<<(tr("Port open : %1\n").arg(port->isOpen()));
    port->setBaudRate(9600);
    port->setDataBits(QSerialPort::Data8);
    port->setParity(QSerialPort::NoParity);
    port->setStopBits(QSerialPort::QSerialPort::OneStop);
    port->setFlowControl(QSerialPort::NoFlowControl);

    connect(port, SIGNAL(readyRead()), this, SLOT(Read_Data()));

    timerID = startTimer(50);
}
