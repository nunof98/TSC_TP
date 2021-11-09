#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QFile>
#include <QFileDialog>
#include <QMessageBox>


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->statusbar->addPermanentWidget(&m_status);

    // Estado inicial destes dois widgets
    showStatus("Serialport disconnected");
    ui->lcdNumber->display("00.00");


    // Conexões dos elementos da interface com os slots definidos na classe
    connect(&m_serial, &QSerialPort::readyRead, this, &MainWindow::receiveData);

    connect(ui->actionConnect, &QAction::triggered, this, &MainWindow::connectSerial);
    connect(ui->actionDisconnect, &QAction::triggered, this, &MainWindow::disconnectSerial);
    connect(ui->actionConfigure, &QAction::triggered, this, &MainWindow::configureSerial);
    connect(ui->btn_saveFile, &QPushButton::clicked, this, &MainWindow::saveFile);
    connect(ui->btn_clear, &QPushButton::clicked, ui->textEdit, &QTextEdit::clear);
    connect(ui->btn_start_stop, &QPushButton::clicked, this, &MainWindow::sendData);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::showStatus(QString status)
{
    m_status.setText(status);
}


void MainWindow::connectSerial()
{
    SerialDialog::Settings settings = m_dialog.settings();
    // Configurar objecto QSerialPort com as settigns escolhidas no SerialDialog
    m_serial.setPortName(settings.name);
    m_serial.setParity(settings.parity);
    m_serial.setBaudRate(settings.baudrate);
    m_serial.setDataBits(settings.dataBits);
    m_serial.setStopBits(settings.stopBits);

    if(m_serial.open(QIODevice::ReadWrite))
    {
        qDebug() << "Serial port opened: " << m_serial.portName();
        ui->actionConnect->setEnabled(false);
        ui->actionDisconnect->setEnabled(true);
        ui->actionConfigure->setEnabled(false);

        showStatus("Serialport Connected to " + m_serial.portName());
    }
    // Caso haja um erro a abrir conexão, abre uma caixa com a mensagem de erro
    else
    {
        QMessageBox::critical(this, "Error", m_serial.errorString());
        showStatus("Serialport disconnected");
    }
}

void MainWindow::disconnectSerial()
{
    if(m_serial.isOpen())
    {
        m_serial.close();

        ui->actionConnect->setEnabled(true);
        ui->actionDisconnect->setEnabled(false);
        ui->actionConfigure->setEnabled(true);

        showStatus("Serialport disconnected");
    }
}

void MainWindow::configureSerial()
{
    m_dialog.show();
}

void MainWindow::receiveData()
{
    if(m_serial.canReadLine())
    {
        QByteArray data = m_serial.readAll();
        qDebug() << data;

        ui->textEdit->append(data);

        if (data.contains("="))
        {
            // Assumindo que Arduino envia dados no formato seguinte:
            // rpm = 15.00\n\r
            QString str = QString(data);
            str.remove(QRegExp("[ \n\r]"));
            QList<QString> list = str.split('=', QString::SkipEmptyParts);
            // Index 0 - "rpm" / Index 1 -"15.00"

            if(list.size() == 2)
            {
                float rpm = list.at(1).toFloat();
                QString rpmString = QString("%1").arg(rpm, 5, 'f', 2, '0');
                ui->lcdNumber->display(rpmString);
            }
        }
    }
}

void MainWindow::sendData()
{
    // Envia pela porta série o que estiver escrito linha de texto
    if(m_serial.isOpen())
    {
        QString rpm = "";
        double data;
        if(m_flag)
        {
            data = ui->doubleSpinBox_rpm->value();
            ui->btn_start_stop->setText("STOP");
            m_flag = false;
        }
        else
        {
            data = 0;
            ui->btn_start_stop->setText("START");
            m_flag = true;
        }

        rpm = QString::number(data) + "\n";
        m_serial.write(rpm.toUtf8());
    }
    else
        QMessageBox::critical(this, "Error", "Serial port isn't connected");
}

void MainWindow::saveFile()
{
    // Abre diálogo para escolher onde guardar ficheiro
    QString fileName = QFileDialog::getSaveFileName(this, "Save File", "", "Text file (*.txt)");
    QFile file(fileName);
    if (!file.open(QIODevice::ReadWrite))
        return;

    // Guarda texto da consola no ficheiro
    QTextStream in(&file);
    in << ui->textEdit->toPlainText();
}
