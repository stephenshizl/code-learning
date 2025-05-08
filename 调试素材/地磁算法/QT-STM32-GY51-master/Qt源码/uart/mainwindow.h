#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPort>
#include <QObject>
#include <QDebug>
#include <QByteArray>
#include <QPainter>
#include <QPixmap>
#include <QPen>
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    QSerialPort* port;
    int timerID;
    int angle_current = 0;

private:
    Ui::MainWindow *ui;
public slots:
    void Read_Data();
    void on_pushButton_2_clicked();
protected:
    void paintEvent(QPaintEvent *);
    void timerEvent(QTimerEvent *event);
private slots:
    void on_pushButton_3_clicked();
};

#endif // MAINWINDOW_H
