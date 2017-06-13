/*
 * Copyright (C) 2016 Markus Ippy, Bastian Gschrey, Jan
 *
 * Digital Gauges for Apexi Power FC for RX7 on Raspberry Pi
 *
 *
 * This software comes under the GPL (GNU Public License)
 * You may freely copy,distribute etc. this as long as the source code
 * is made available for FREE.
 *
 * No warranty is made or implied. You use this program at your own risk.
 */

/*
  \file serial.h
  \
  \author Bastian Gschrey & Markus Ippy
 */

#ifndef SERIAL_H
#define SERIAL_H

#include <QObject>
#include <QModbusDataUnit>

class SerialPort;
class DashBoard;
class Decoder;
class AppSettings;
class QModbusClient;
class QModbusReply;

class Serial : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QStringList portsNames READ portsNames WRITE setPortsNames NOTIFY sig_portsNamesChanged)

public:
    ~Serial();
    explicit Serial(QObject *parent = 0);

    Q_INVOKABLE void clear() const;
    Q_INVOKABLE void initSerialPort();
    Q_INVOKABLE void openConnection(const QString &portName, const int &ecuSelect, const int &interfaceSelect, const int &loggingSelect);
    Q_INVOKABLE void closeConnection();
    Q_INVOKABLE void startLogging(const QString &logfilenameSelect, const int &loggeron);
    Q_INVOKABLE void stopLogging(const QString &logfilenameSelect, const int &loggeron);
    Q_INVOKABLE void Auxcalc (const QString &unitaux1,const int &an1V0,const int &an2V5,const QString &unitaux2,const int &an3V0,const int &an4V5,const QString &unitaux3,const int &an5V0,const int &an6V5,const QString &unitaux4,const int &an7V0,const int &an8V5);
    Q_INVOKABLE void gopro();
    Q_INVOKABLE void goProSettings(const int &goProSelect, const QString &goPropass);
public slots:
    void getPorts();
    void getEcus();
    //void getLogging();

public:
    void writeRequestPFC(QByteArray);


    //Adaptronic Streaming Comms


    void AdaptronicStopStream();

    QStringList portsNames() const { return m_portsNames; }

private:
    SerialPort *m_serialport;
    Decoder *m_decoder;
    DashBoard *m_dashBoard;
    AppSettings *m_appSettings;
    QStringList m_portsNames;
    QStringList *m_ecuList;
    QModbusReply *lastRequest;
    QModbusClient *modbusDevice;



    //QModbusReply reply;


    QModbusDataUnit readRequest() const;

signals:
    void sig_finished();
    void sig_error(QString err);
    void sig_ApexiDataAvailable(QByteArray);
    void sig_portsNamesChanged(QStringList portsNames);



public slots:
    void readyToRead();
    void AdaptronicStartStream();



        //void openConnection(SerialSetting::Settings p); //open serial connection with settings
    void sendRequest(int requestIndex);

    void setPortsNames(QStringList portsNames)
    {
        if (m_portsNames == portsNames)
            return;

        m_portsNames = portsNames;
        emit sig_portsNamesChanged(portsNames);
    }
    void readData(QByteArray serialdata);


};

#endif // SERIAL_H
