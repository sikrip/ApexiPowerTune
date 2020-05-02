#include "Apexi.h"
#include "dashboard.h"
#include "connect.h"
#include "ApexuFuelMap.h"
#include <iostream>
#include <iomanip>
#include <QTime>
#include <QTimer>
#include <QDebug>
#include <QBitArray>
#include <QByteArrayMatcher>
#include <QFile>
#include <QTextStream>

/**
 * The raw bytes of the PFC responses
 */
QByteArray rawmessagedata;

/**
 * The calculated value of AN1-2
 */
qreal AN1AN2calc;

/**
 * The calculated value of AN1-2
 */
qreal AN3AN4calc;

/**
 * Depicts what is measured via AN1-2 (ex AFR)
 */
QString Auxname1;

/**
 * Depicts what is measured via AN3-4 (ex AFR)
 */
QString Auxname2;

/**
 * The value of the calculation for AN1-2 at volt 0
 */
float auxval1;

/**
 * The value of the calculation for AN1-2 at volt 5
 */
float auxval2;

/**
 * The value of the calculation for AN3-4 at volt 0
 */
float auxval3;

/**
 * The value of the calculation for AN3-4 at volt 5
 */
float auxval4;

/**
 * This is used to decide if a reconnect attempt will be performed.
 * As it is, only one reconnect attempt will be done.
 */
int reconnect = 0;

/**
 * The name of the port used to connect to PFC
 */
QString port;

/**
 * The model of the connected car.
 * 1 => Mazda
 * 2 => Nissan, Subaru, Honda
 * 3 => Toyota, Mitsubishi
 */
int Model = 0;

/**
 * Holds the number of expected bytes of the next PFC response.
 */
int expectedbytes;

/**
 * The PFC protocol to be used.
 * 0 => New protocol
 * 2 => Old protocol
 *
 * TODO looks like only protocol 0 is used
 */
int Protocol = 0;

int requestIndex = 0; // ID for requested data type Power FC
const int FIRST_INIT_REQUEST = 0;
const int SENSOR_STR_REQUEST = 1;
const int SECOND_INIT_REQUEST = 2;
const int FUEL_MAP_REQUEST_1 = 3;
const int FUEL_MAP_REQUEST_2 = 4;
const int FUEL_MAP_REQUEST_3 = 5;
const int FUEL_MAP_REQUEST_4 = 6;
const int FUEL_MAP_REQUEST_5 = 7;
const int FUEL_MAP_REQUEST_6 = 8;
const int FUEL_MAP_REQUEST_7 = 9;
const int FUEL_MAP_REQUEST_8 = 10;
// Live data requests
const int ADV_DATA_REQUEST = 11;
const int MAP_IDX_REQUEST = 12;
const int SENSOR_DATA_REQUEST = 13;
const int BASIC_DATA_REQUEST = 14;
const int AUX_REQUEST = 15;

// TODO find why this is a state variable
qreal advboost;

double mul[80] = FC_INFO_MUL;  // required values for calculation from raw to readable values for Advanced Sensor info
double add[] = FC_INFO_ADD;

/**
 * Limits the number of write requests to the fuel map.
 * A value lower than 8 will not write the entire fuel map.
 */
const int FUEL_MAP_MAX_WRITE_REQUESTS = 3;
const int MAX_AUTOTUNE_RPM_IDX = 6; // ~3250 rpm
const int MAX_AUTOTUNE_LOAD_IDX = 10; // ~7600 load
const double MIN_AUTOTUNE_WATER_TEMP = 75;
const double MIN_AUTOTUNE_RPM = 500;

int logLevel = 1; // 0: off, 1: connect, disconnect etc, 2: all

// Used logging messages in fixed intervals
long logSamplesCount = 0;
const int LOG_INTERVAL = 1000;

Apexi::Apexi(QObject *parent)
        : QObject(parent), m_dashboard(Q_NULLPTR) {
}

Apexi::Apexi(DashBoard *dashboard, QObject *parent)
        : QObject(parent), m_dashboard(dashboard) {
}

void Apexi::SetProtocol(const int &protocolselect) {
    Protocol = protocolselect;
}

void Apexi::initSerialPort() {
    if (logLevel>0) {
        cout << "Initializing serial port\n";
    }
    m_serialport = new SerialPort(this);
    connect(this->m_serialport, SIGNAL(readyRead()), this, SLOT(readyToRead()));
    connect(m_serialport, static_cast<void (QSerialPort::*)(QSerialPort::SerialPortError)>(&QSerialPort::error),
            this, &Apexi::handleError);
    connect(m_serialport, &QSerialPort::bytesWritten, this, &Apexi::handleBytesWritten);
    connect(&m_timer, &QTimer::timeout, this, &Apexi::handleTimeout);
    m_readData.clear();
}

//function for flushing all serial buffers
void Apexi::clear() {
    if (logLevel>0) {
        cout << "Clearing serial port\n";
    }
    m_serialport->clear();
}

//function to open serial port
void Apexi::openConnection(const QString &portName) {
    if (logLevel>0) {
        cout << "Opening connection\n";
    }
    port = portName;
    initSerialPort();
    m_serialport->setPortName(port);
    m_serialport->setBaudRate(QSerialPort::Baud57600);
    m_serialport->setParity(QSerialPort::NoParity);
    m_serialport->setDataBits(QSerialPort::Data8);
    m_serialport->setStopBits(QSerialPort::OneStop);
    m_serialport->setFlowControl(QSerialPort::NoFlowControl);;

    if (m_serialport->open(QIODevice::ReadWrite) == false) {
        if (logLevel>0) {
            cout << "Failed to open serial communication: " << m_serialport->errorString().toStdString() << endl;
        }
        m_dashboard->setSerialStat(m_serialport->errorString());
        Apexi::closeConnection();
    } else {
        if (logLevel>0) {
            cout << "Connected to Serial Port\n";
        }
        m_dashboard->setSerialStat(QString("Connected to Serialport"));
        requestIndex = FIRST_INIT_REQUEST;
        Apexi::sendPfcReadRequest();
    }
}

void Apexi::closeConnection() {
    if (logLevel>0) {
        cout << "Closing connection\n";
    }
    disconnect(this->m_serialport, SIGNAL(readyRead()), this, SLOT(readyToRead()));
    disconnect(m_serialport, static_cast<void (QSerialPort::*)(QSerialPort::SerialPortError)>(&QSerialPort::error),
               this, &Apexi::handleError);
    disconnect(m_serialport, &QSerialPort::bytesWritten, this, &Apexi::handleBytesWritten);
    disconnect(&m_timer, &QTimer::timeout, this, &Apexi::handleTimeout);
    m_serialport->close();
}

void Apexi::retryconnect() {
    if (logLevel>0) {
        cout << "Retry connection\n";
    }
    Apexi::openConnection(port);
}

void Apexi::handleTimeout() {
    if (logLevel>0) {
        cout << "Handling timeout\n";
    }
    m_dashboard->setTimeoutStat(QString("Is Timeout : Y"));
    m_timer.stop();
    m_serialport->close();
    if (m_serialport->open(QIODevice::ReadWrite) == false) {
        if (logLevel>0) {
            cout << "Failed to open serial communication: " << m_serialport->errorString().toStdString() << endl;
        }
        m_dashboard->setSerialStat(m_serialport->errorString());
    } else {
        if (logLevel>0) {
            cout << "Connected to Serial Port\n";
        }
        m_dashboard->setSerialStat(QString("Connected to Serialport"));
    }

    requestIndex = SECOND_INIT_REQUEST;
    m_readData.clear();

    Apexi::sendPfcReadRequest();
}

void Apexi::handleError(QSerialPort::SerialPortError serialPortError) {
    if (logLevel>0) {
        cout << "Handling error " << m_serialport->errorString().toStdString() << endl;
    }
    if (serialPortError == QSerialPort::ReadError) {
        QString fileName = "Errors.txt";
        QFile mFile(fileName);
        if (!mFile.open(QFile::Append | QFile::Text)) {
        }
        QTextStream out(&mFile);
        out << "Serial Error " << (m_serialport->errorString()) << endl;
        mFile.close();
        m_dashboard->setSerialStat(m_serialport->errorString());
    }
}

void Apexi::readyToRead() {
    if (logLevel>1) {
        cout << "readyToRead callback." << endl;
    }
    m_readData = m_serialport->readAll();
    /*
    //Test enable raw message log
      QString fileName = "RawMessage.txt";
      QFile mFile(fileName);
      if(!mFile.open(QFile::Append | QFile::Text)){
      }
      QTextStream out(&mFile);
      out << m_readData.toHex() <<endl;
      mFile.close();
      // Test End
    */
    m_dashboard->setRecvData(QString("Receive Data : " + m_readData.toHex()));
    Apexi::decodeResponseAndSendNextRequest(m_readData);
}

/**
 * Responsible to decode the provided PFC response and send the next request.
 *
 * @param buffer the raw PFC response
 */
void Apexi::decodeResponseAndSendNextRequest(const QByteArray &buffer) {
    if (logLevel>1) {
        cout << "decodeResponseAndSendNextRequest" << endl;
    }
    m_buffer.append(buffer);
    QByteArray startpattern = m_writeData.left(1);
    QByteArrayMatcher startmatcher(startpattern);

    // No idea what is going on here!
    int pos = 0;
    while ((pos = startmatcher.indexIn(m_buffer, pos)) != -1) {
        m_dashboard->setRunStat(m_buffer.toHex());
        if (pos != 0) {
            m_buffer.remove(0, pos);
            if (m_buffer.length() > expectedbytes) {
                m_buffer.remove(expectedbytes, m_buffer.length());
            }
        }
        if (pos == 0) {
            break;
        }
    }

    if (m_buffer.length() == expectedbytes) {
        m_dashboard->setTimeoutStat(QString("Is Timeout : N"));

        m_apexiMsg = m_buffer;
        m_buffer.clear();
        m_timer.stop();

        // Decode current data
        decodePfcData(m_apexiMsg);
        m_apexiMsg.clear();

        if (handleNextFuelMapWriteRequest(FUEL_MAP_MAX_WRITE_REQUESTS)) {
            if (logLevel>0 && getCurrentFuelMapWriteRequest() == 1) {
                    cout << "\nWriting fuel map..." << endl;
                    logFuelData(10);
            }
            // Fuel map should be updated; live data acquisition will be stopped until the map is sent to PFC
            QByteArray writePacket = QByteArray::fromRawData(getNextFuelMapWritePacket(), MAP_WRITE_PACKET_LENGTH);
            if (logLevel>1) {
                cout << "Sending map write packet: " << writePacket.toHex().toStdString() << endl;
            }
            Apexi::writeRequestPFC(writePacket);
            //TODO should verify that the ack packet is actually received
            expectedbytes = 3; // ack packet (0xF2 0x02 0x0B) is expected
            m_timer.start(700);
        } else {
            // Decide the next request to be sent to PFC
            if (requestIndex < AUX_REQUEST) {
                // Once go through all requests (init, sensor strings, init, fuel map, adv data, map idx, sensor data, basic data, aux)
                requestIndex++;
            } else {
                // then cycle through live data requests ADV_DATA_REQUEST..AUX_REQUEST (adv data, map idx, sensor data, basic data, aux)
                requestIndex = ADV_DATA_REQUEST;
                // New cycle of live data, so update the afr logs with the previous data
                updateAutoTuneLogs();
            }
            Apexi::sendPfcReadRequest();
        }
    }
}

void Apexi::updateAutoTuneLogs() {
    const int rpmIdx = packageMap[0]; // col MapN
    const int loadIdx = packageMap[1];// row MapP
    const double speed = (double) m_dashboard->speed();
    const double rpm = (double) m_dashboard->rpm(); // packageBasic[3];
    const double waterTemp = (double) m_dashboard->Watertemp(); // packageBasic[7];
    const double afr = (double) AN3AN4calc; // wideband is connected to An3-AN4
    const double tps = (double) m_dashboard->ThrottleV();

    const bool shouldUpdateAfr = rpmIdx <= MAX_AUTOTUNE_RPM_IDX && loadIdx < MAX_AUTOTUNE_LOAD_IDX &&
                                 waterTemp >= MIN_AUTOTUNE_WATER_TEMP && rpm > MIN_AUTOTUNE_RPM;

    if (logLevel > 0 /* && logSamplesCount++ % LOG_INTERVAL */) {
        cout << QTime::currentTime().toString("hh:mm:ss.zzz").toStdString()
             << " Updating fuel data:" << shouldUpdateAfr << " Water temp:" << waterTemp
             << " RpmIdx:" << rpmIdx << " LoadIdx:" <<  loadIdx
             << " Rpm:" << rpm << " Speed:" << speed
             << " AFR:" << afr << " Tps:" << tps << endl;
    }

    if (shouldUpdateAfr) {
        updateAFRData(rpmIdx, loadIdx, afr);
    }
}

void Apexi::decodePfcData(QByteArray rawmessagedata) {
    if (logLevel>1) {
        cout << "PFC response packet: " << rawmessagedata.toHex().toStdString() << endl;
    }
    if (rawmessagedata.length()) {
        //Power FC Decode
        quint8 requesttype = rawmessagedata[0];
        switch (requesttype) {
            case ID::Advance:
                Apexi::decodeAdv(rawmessagedata);
                break;
            case ID::SensorStrings:
                Apexi::decodeSensorStrings(rawmessagedata);
                break;
            case ID::SensorData:
                Apexi::decodeSensor(rawmessagedata);
                break;
            case ID::OldSensorData:
                Apexi::decodeSensor(rawmessagedata);
                break;
            case ID::AuxData:
                Apexi::decodeAux(rawmessagedata);
                break;
            case ID::MapIndex:
                Apexi::decodeMapIndices(rawmessagedata);
                break;
            case ID::OldMapIndex:
                Apexi::decodeMapIndices(rawmessagedata);
                break;
            case ID::BasicData:
                Apexi::decodeBasic(rawmessagedata);
                break;
            case ID::OldBasicData:
                Apexi::decodeBasic(rawmessagedata);
                break;
            case ID::Init:
                Apexi::decodeInit(rawmessagedata);
                if (reconnect == 0) {
                    qDebug() << "reconnect";
                    reconnect = 1;
                    requestIndex = FIRST_INIT_REQUEST;
                    Apexi::closeConnection();
                    QTimer::singleShot(2000, this, SLOT(retryconnect()));
                }
                break;
            case ID::FuelMapBatch1:
                readFuelMap(1, rawmessagedata.data());
                break;
            case ID::FuelMapBatch2:
                readFuelMap(2, rawmessagedata.data());
                break;
            case ID::FuelMapBatch3:
                readFuelMap(3, rawmessagedata.data());
                break;
            case ID::FuelMapBatch4:
                readFuelMap(4, rawmessagedata.data());
                break;
            case ID::FuelMapBatch5:
                readFuelMap(5, rawmessagedata.data());
                break;
            case ID::FuelMapBatch6:
                readFuelMap(6, rawmessagedata.data());
                break;
            case ID::FuelMapBatch7:
                readFuelMap(7, rawmessagedata.data());
                break;
            case ID::FuelMapBatch8:
                readFuelMap(8, rawmessagedata.data());
                if (logLevel>0) {
                    cout << "== Read the following fuel map ==\n";
                    for (int r = 0; r < 20; r++) {
                        for (int c = 0; c < 20; c++) {
                            cout << getCurrentFuel(r, c);
                            if (c < 19) {
                                cout << ",";
                            }
                        }
                        cout << "\n";
                    }
                }
                break;
                /*
            case ID::Version:
                Apexi::decodeVersion(rawmessagedata);
                break;
            */
            default:
                break;
        }
    }
    rawmessagedata.clear();
}

void Apexi::handleBytesWritten(qint64 bytes) {
    m_bytesWritten += bytes;
    if (m_bytesWritten == m_writeData.size()) {
        m_bytesWritten = 0;
    }
}

void Apexi::writeRequestPFC(QByteArray p_request) {
    m_writeData = p_request;
    qint64 bytesWritten = m_serialport->write(p_request);
    // m_dashboard->setSerialStat(QString("Sending Request " + p_request.toHex()));

    if (bytesWritten == -1) {
        m_dashboard->setSerialStat(m_serialport->errorString());
    } else if (bytesWritten != m_writeData.size()) {
        m_dashboard->setSerialStat(m_serialport->errorString());
    }

}

void Apexi::sendPfcReadRequest() {
    if (logLevel>1) {
        cout << "sendPfcReadRequest with requestIndex:" << requestIndex << endl;
    }
    if (Protocol == 0) { //New Apexi Structure
        switch (requestIndex) {
            case FIRST_INIT_REQUEST:
                //Init Platform (This returns the Platform String )
                Apexi::writeRequestPFC(QByteArray::fromHex("F3020A"));
                expectedbytes = 11;
                break;
            case SENSOR_STR_REQUEST:
                //Apexi::getSensorStrings();
                Apexi::writeRequestPFC(QByteArray::fromHex("DD0220"));
                expectedbytes = 83;
                break;
            case SECOND_INIT_REQUEST:
                //Init Platform (This returns the Platform String )
                Apexi::writeRequestPFC(QByteArray::fromHex("F3020A"));
                expectedbytes = 11;
                break;
            case FUEL_MAP_REQUEST_1:
                // Request fuel map (request 1 of 8)
                Apexi::writeRequestPFC(QByteArray::fromHex("B0024D"));
                expectedbytes = 103; // 1(id) + 1(num) + 100(payload) + 1(checksum);
                break;
            case FUEL_MAP_REQUEST_2:
                // Request fuel map (request 2 of 8)
                Apexi::writeRequestPFC(QByteArray::fromHex("B1024C"));
                expectedbytes = 103; // 1(id) + 1(num) + 100(payload) + 1(checksum);
                break;
            case FUEL_MAP_REQUEST_3:
                // Request fuel map (request 3 of 8)
                Apexi::writeRequestPFC(QByteArray::fromHex("B2024B"));
                expectedbytes = 103; // 1(id) + 1(num) + 100(payload) + 1(checksum);
                break;
            case FUEL_MAP_REQUEST_4:
                // Request fuel map (request 4 of 8)
                Apexi::writeRequestPFC(QByteArray::fromHex("B3024A"));
                expectedbytes = 103; // 1(id) + 1(num) + 100(payload) + 1(checksum);
                break;
            case FUEL_MAP_REQUEST_5:
                // Request fuel map (request 5 of 8)
                Apexi::writeRequestPFC(QByteArray::fromHex("B40249"));
                expectedbytes = 103; // 1(id) + 1(num) + 100(payload) + 1(checksum);
                break;
            case FUEL_MAP_REQUEST_6:
                // Request fuel map (request 6 of 8)
                Apexi::writeRequestPFC(QByteArray::fromHex("B50248"));
                expectedbytes = 103; // 1(id) + 1(num) + 100(payload) + 1(checksum);
                break;
            case FUEL_MAP_REQUEST_7:
                // Request fuel map (request 7 of 8)
                Apexi::writeRequestPFC(QByteArray::fromHex("B60247"));
                expectedbytes = 103; // 1(id) + 1(num) + 100(payload) + 1(checksum);
                break;
            case FUEL_MAP_REQUEST_8:
                // Request fuel map (request 8 of 8)
                Apexi::writeRequestPFC(QByteArray::fromHex("B70246"));
                expectedbytes = 103; // 1(id) + 1(num) + 100(payload) + 1(checksum);
                break;
                // Live Data
            case ADV_DATA_REQUEST:
                //Apexi::getAdvData();
                Apexi::writeRequestPFC(QByteArray::fromHex("F0020D"));
                expectedbytes = 33;
                break;
            case MAP_IDX_REQUEST:
                //Apexi::getMapIndices();
                Apexi::writeRequestPFC(QByteArray::fromHex("DB0222"));
                expectedbytes = 5;
                break;
            case SENSOR_DATA_REQUEST:
                //Apexi::getSensorData();
                Apexi::writeRequestPFC(QByteArray::fromHex("DE021F"));
                expectedbytes = 21;
                break;
            case BASIC_DATA_REQUEST:
                //Apexi::getBasic();
                Apexi::writeRequestPFC(QByteArray::fromHex("DA0223"));
                expectedbytes = 23;
                break;
            case AUX_REQUEST:
                //Apexi::getAux();
                Apexi::writeRequestPFC(QByteArray::fromHex("0002FD"));
                expectedbytes = 7;
                break;
        }
    }
    if (Protocol == 1) {
        switch (requestIndex) {
            // Old Apexi Structure
            case FIRST_INIT_REQUEST:
                //Init Platform (This returns the Platform String )
                Apexi::writeRequestPFC(QByteArray::fromHex("F3020A"));
                expectedbytes = 11;
                break;
            case SENSOR_STR_REQUEST:
                //Apexi::getSensorStrings();
                Apexi::writeRequestPFC(QByteArray::fromHex("690294"));
                expectedbytes = 83;
                break;
            case SECOND_INIT_REQUEST:
                //Apexi::getAdvData();
                Apexi::writeRequestPFC(QByteArray::fromHex("F0020D"));
                expectedbytes = 33;
                break;
            case FUEL_MAP_REQUEST_1:
                // Request fuel map (request 1 of 8)
                Apexi::writeRequestPFC(QByteArray::fromHex("B0024D"));
                expectedbytes = 103; // 1(id) + 1(num) + 100(payload) + 1(checksum);
                break;
            case FUEL_MAP_REQUEST_2:
                // Request fuel map (request 2 of 8)
                Apexi::writeRequestPFC(QByteArray::fromHex("B1024C"));
                expectedbytes = 103; // 1(id) + 1(num) + 100(payload) + 1(checksum);
                break;
            case FUEL_MAP_REQUEST_3:
                // Request fuel map (request 3 of 8)
                Apexi::writeRequestPFC(QByteArray::fromHex("B2024B"));
                expectedbytes = 103; // 1(id) + 1(num) + 100(payload) + 1(checksum);
                break;
            case FUEL_MAP_REQUEST_4:
                // Request fuel map (request 4 of 8)
                Apexi::writeRequestPFC(QByteArray::fromHex("B3024A"));
                expectedbytes = 103; // 1(id) + 1(num) + 100(payload) + 1(checksum);
                break;
            case FUEL_MAP_REQUEST_5:
                // Request fuel map (request 5 of 8)
                Apexi::writeRequestPFC(QByteArray::fromHex("B40249"));
                expectedbytes = 103; // 1(id) + 1(num) + 100(payload) + 1(checksum);
                break;
            case FUEL_MAP_REQUEST_6:
                // Request fuel map (request 6 of 8)
                Apexi::writeRequestPFC(QByteArray::fromHex("B50248"));
                expectedbytes = 103; // 1(id) + 1(num) + 100(payload) + 1(checksum);
                break;
            case FUEL_MAP_REQUEST_7:
                // Request fuel map (request 7 of 8)
                Apexi::writeRequestPFC(QByteArray::fromHex("B60247"));
                expectedbytes = 103; // 1(id) + 1(num) + 100(payload) + 1(checksum);
                break;
            case FUEL_MAP_REQUEST_8:
                // Request fuel map (request 8 of 8)
                Apexi::writeRequestPFC(QByteArray::fromHex("B70246"));
                expectedbytes = 103; // 1(id) + 1(num) + 100(payload) + 1(checksum);
                break;
                // Live Data
            case ADV_DATA_REQUEST:
                //Apexi::getMapIndices();
                Apexi::writeRequestPFC(QByteArray::fromHex("680295"));
                expectedbytes = 5;
                break;
            case MAP_IDX_REQUEST:
                //Apexi::getSensorData();
                Apexi::writeRequestPFC(QByteArray::fromHex("6A0293"));
                expectedbytes = 21;
                break;
            case SENSOR_DATA_REQUEST:
                //Apexi::getBasic();
                Apexi::writeRequestPFC(QByteArray::fromHex("660297"));
                expectedbytes = 23;
                break;
            case BASIC_DATA_REQUEST:
                //Apexi::getAux();
                Apexi::writeRequestPFC(QByteArray::fromHex("0002FD"));
                expectedbytes = 7;
                break;
        }
    }
    m_timer.start(700); //Set timout to 700 mseconds 
}

void Apexi::Auxcalc(const QString &unitaux1, const qreal &an1V0, const qreal &an2V5, const QString &unitaux2,
                    const qreal &an3V0, const qreal &an4V5) {
    qreal aux1min = an1V0;
    qreal aux2max = an2V5;
    qreal aux3min = an3V0;
    qreal aux4max = an4V5;
    QString Auxunit1 = unitaux1;
    QString Auxunit2 = unitaux2;

    Apexi::calculatorAux(aux1min, aux2max, aux3min, aux4max, Auxunit1, Auxunit2);
}

void Apexi::decodeAdv(QByteArray rawmessagedata) {
    fc_adv_info_t *info = reinterpret_cast<fc_adv_info_t *>(rawmessagedata.data());
    if (Model == 1) {

        packageADV[0] = info->RPM + add[0];
        packageADV[1] = info->Intakepress;
        packageADV[2] = info->PressureV * 0.001; //value in V
        packageADV[3] = info->ThrottleV * 0.001; //value in V
        packageADV[4] = info->Primaryinp * 0.001;
        packageADV[5] = info->Fuelc;
        packageADV[6] = info->Leadingign - 25;
        packageADV[7] = info->Trailingign - 25;
        packageADV[8] = info->Fueltemp + add[8];
        packageADV[9] = info->Moilp;     //Value lower by 10 compared to FC Edit
        packageADV[10] = info->Boosttp / 2.56;    // (FC edit shows just raw value
        packageADV[11] = info->Boostwg / 2.56;     // (FC edit shows just raw value
        packageADV[12] = info->Watertemp - 80;
        packageADV[13] = info->Intaketemp - 80;
        packageADV[14] = info->Knock;
        packageADV[15] = info->BatteryV * 0.1;
        packageADV[16] = info->Speed;
        packageADV[17] = info->Iscvduty * 0.001;
        packageADV[18] = info->O2volt;
        packageADV[19] = info->na1;
        packageADV[20] = info->Secinjpulse * 0.001;
        packageADV[21] = info->na2;

        m_dashboard->setrpm(packageADV[0]);
        m_dashboard->setIntakepress(packageADV[1]);
        m_dashboard->setPressureV(packageADV[2]);
        m_dashboard->setThrottleV(packageADV[3]);
        m_dashboard->setPrimaryinp(packageADV[4]);
        m_dashboard->setFuelc(packageADV[5]);
        m_dashboard->setLeadingign(packageADV[6]);
        m_dashboard->setTrailingign(packageADV[7]);
        m_dashboard->setFueltemp(packageADV[8]);
        m_dashboard->setMoilp(packageADV[9]);
        m_dashboard->setBoosttp(packageADV[10]);
        m_dashboard->setBoostwg(packageADV[11]);
        m_dashboard->setWatertemp(packageADV[12]);
        m_dashboard->setIntaketemp(packageADV[13]);
        m_dashboard->setKnock(packageADV[14]);
        m_dashboard->setBatteryV(packageADV[15]);
        m_dashboard->setSpeed(packageADV[16]);
        m_dashboard->setIscvduty(packageADV[17]);
        m_dashboard->setO2volt(packageADV[18]);
        m_dashboard->setna1(packageADV[19]);
        m_dashboard->setSecinjpulse(packageADV[20]);
        m_dashboard->setna2(packageADV[21]);


        //    //qDebug() << "Time passed since last call"<< startTime.msecsTo(QTime::currentTime());
        //odometer += ((startTime.msecsTo(QTime::currentTime())) * ((packageADV[16]) / 3600000)); // Odometer
        //m_dashboard->setOdo(odometer);
        // startTime.restart();


    }
    // Nissan and Subaru
    if (Model == 2) {
        fc_adv_info_t2 *info = reinterpret_cast<fc_adv_info_t2 *>(rawmessagedata.data());

        packageADV2[0] = info->RPM;
        packageADV2[1] = info->EngLoad;
        packageADV2[2] = info->MAF1V * 0.001;
        packageADV2[3] = info->MAF2V * 0.001;
        packageADV2[4] = info->injms * 0.004;
        packageADV2[5] = info->Inj; //fc edit shows raw byte
        packageADV2[6] = info->Ign;
        packageADV2[7] = info->Dwell;
        packageADV2[8] = -760 + info->BoostPres;
        if (packageADV2[8] >= 0x8000)
            packageADV2[8] = (packageADV[8] - 0x8000) * 0.01;
        else
            packageADV2[8] = (1.0 / 2560 + 0.001) * packageADV[8];
        packageADV2[9] = info->BoostDuty * 0.005;
        packageADV2[10] = info->Watertemp - 80;
        packageADV2[11] = info->Intaketemp - 80;
        packageADV2[12] = info->Knock;
        packageADV2[13] = info->BatteryV * 0.1;
        packageADV2[14] = info->Speed;
        packageADV2[15] = info->MAFactivity * 0.16;
        packageADV2[16] = info->O2volt * 0.005;
        packageADV2[17] = info->O2volt_2 * 0.005;
        packageADV2[18] = info->ThrottleV * 0.001;

        m_dashboard->setrpm(packageADV2[0]);
        m_dashboard->setEngLoad(packageADV2[1]);
        m_dashboard->setMAF1V(packageADV2[2]);
        m_dashboard->setMAF2V(packageADV2[3]);
        m_dashboard->setinjms(packageADV2[4]);
        //m_dashboard->setFue(packageADV2[5]);
        m_dashboard->setIgn(packageADV2[6]);
        m_dashboard->setDwell(packageADV2[7]);
        m_dashboard->setBoostPres(packageADV2[8]);
        m_dashboard->setBoostDuty(packageADV2[9]);
        m_dashboard->setWatertemp(packageADV2[10]);
        m_dashboard->setIntaketemp(packageADV2[11]);
        m_dashboard->setKnock(packageADV2[12]);
        m_dashboard->setBatteryV(packageADV2[13]);
        m_dashboard->setSpeed(packageADV2[14]);
        m_dashboard->setMAFactivity(packageADV2[15]);
        m_dashboard->setO2volt(packageADV2[16]);
        m_dashboard->setO2volt_2(packageADV2[17]);
        m_dashboard->setThrottleV((packageADV2[18] * 100));
        m_dashboard->setTPS((packageADV2[18] * 100) / 4.38);

    }
    //Toyota
    if (Model == 3) {
        fc_adv_info_t3 *info = reinterpret_cast<fc_adv_info_t3 *>(rawmessagedata.data());
        int checkboost = (unsigned char) rawmessagedata[17];
        packageADV3[0] = mul[0] * info->RPM3 + add[0];
        //previousRev_rpm[buf_currentIndex] = packageADV[0];
        packageADV3[1] = info->Intakepress3;
        packageADV3[2] = info->PressureV3 * 0.001;
        packageADV3[3] = info->ThrottleV3 * 0.001;
        packageADV3[4] = info->Primaryinp3;
        packageADV3[5] = info->Fuelc3;
        packageADV3[6] = info->Ign3;
        packageADV3[7] = info->Dwell3;
        if (checkboost == 128) {
            int convert = (unsigned char) rawmessagedata[16];
            advboost = convert * 0.01;
        } else {
            packageADV3[8] = info->BoostPres3 - 760;
            advboost = packageADV3[8];
        }
        packageADV3[9] = mul[9] * info->BoostDuty3 + add[9];
        packageADV3[10] = info->Watertemp3 - 80;
        packageADV3[11] = info->Intaketemp3 - 80;
        packageADV3[12] = info->Knock3;
        packageADV3[13] = info->BatteryV3 * 0.1;
        packageADV3[14] = info->Speed3;

        // packageADV3[14] *= speed_correction;
        //previousSpeed_kph[buf_currentIndex] = packageADV[14];
        //        packageADV3[15] = mul[15] * info->Iscvduty + add[15];
        packageADV3[16] = mul[16] * info->O2volt3 + add[16];
        //        packageADV3[17] = mul[17] * info->SuctionAirTemp + add[17];
        //        packageADV3[18] = mul[18] * info->ThrottleV_2 + add[18];
        packageADV3[19] = mul[19] * info->na13 + add[19];
        packageADV3[20] = 0;
        packageADV3[21] = 0;


        m_dashboard->setrpm(packageADV3[0]);
        m_dashboard->setIntakepress(packageADV3[1]);
        m_dashboard->setPressureV(packageADV3[2]);
        m_dashboard->setThrottleV(packageADV3[3]);
        m_dashboard->setPrimaryinp(packageADV3[4]);
        m_dashboard->setFuelc(packageADV3[5]);
        m_dashboard->setLeadingign(packageADV3[6]);
        m_dashboard->setTrailingign(packageADV3[7]);
        m_dashboard->setpim(advboost);
        m_dashboard->setWatertemp(packageADV3[10]);
        m_dashboard->setIntaketemp(packageADV3[11]);
        m_dashboard->setKnock(packageADV3[12]);
        m_dashboard->setBatteryV(packageADV3[13]);
        m_dashboard->setSpeed(packageADV3[14]);
    }
}

void Apexi::decodeSensor(QByteArray rawmessagedata) {
    fc_sens_info_t *info = reinterpret_cast<fc_sens_info_t *>(rawmessagedata.data());

    packageSens[0] = info->sens1 * 0.01;
    packageSens[1] = info->sens2 * 0.01;
    packageSens[2] = info->sens3 * 0.01;
    packageSens[3] = info->sens4 * 0.01;
    packageSens[4] = info->sens5 * 0.01;
    packageSens[5] = info->sens6 * 0.01;
    packageSens[6] = info->sens7 * 0.01;
    packageSens[7] = info->sens8 * 0.01;

    QBitArray flagArray(16);
    for (int i = 0; i < 16; i++)
        flagArray.setBit(i, info->flags >> i & 1);


    m_dashboard->setsens1(packageSens[0]);
    m_dashboard->setsens2(packageSens[1]);
    m_dashboard->setsens3(packageSens[2]);
    m_dashboard->setsens4(packageSens[3]);
    m_dashboard->setsens5(packageSens[4]);
    m_dashboard->setsens6(packageSens[5]);
    m_dashboard->setsens7(packageSens[6]);
    m_dashboard->setsens8(packageSens[7]);

    //Bit Flags for Sensors
    m_dashboard->setFlag1(flagArray[0]);
    m_dashboard->setFlag2(flagArray[1]);
    m_dashboard->setFlag3(flagArray[2]);
    m_dashboard->setFlag4(flagArray[3]);
    m_dashboard->setFlag5(flagArray[4]);
    m_dashboard->setFlag6(flagArray[5]);
    m_dashboard->setFlag7(flagArray[6]);
    m_dashboard->setFlag8(flagArray[7]);
    m_dashboard->setFlag9(flagArray[8]);
    m_dashboard->setFlag10(flagArray[9]);
    m_dashboard->setFlag11(flagArray[10]);
    m_dashboard->setFlag12(flagArray[11]);
    m_dashboard->setFlag13(flagArray[12]);
    m_dashboard->setFlag14(flagArray[13]);
    m_dashboard->setFlag15(flagArray[14]);
    m_dashboard->setFlag16(flagArray[15]);
}

void Apexi::decodeAux(QByteArray rawmessagedata) {
    fc_aux_info_t *info = reinterpret_cast<fc_aux_info_t *>(rawmessagedata.data());


    packageAux[0] = mul[29] * info->AN1 + add[29];
    packageAux[1] = mul[29] * info->AN2 + add[29];
    packageAux[2] = mul[29] * info->AN3 + add[29];
    packageAux[3] = mul[29] * info->AN4 + add[29];

    //qDebug()<< "AN1" << packageAux[0] ;
    //qDebug()<< "AN2" << packageAux[1] ;
    //qDebug()<< "AN3" << packageAux[2] ;
    //qDebug()<< "AN4" << packageAux[3] ;
    //Analog1
    AN1AN2calc = (((((auxval2 - auxval1) * 0.2) * (packageAux[0] - packageAux[1]))) + auxval1);
    AN3AN4calc = ((((auxval4 - auxval3) * 0.2) * (packageAux[2] - packageAux[3])) + auxval3);
    m_dashboard->setauxcalc1(AN1AN2calc);
    m_dashboard->setauxcalc2(AN3AN4calc);
    //qDebug()<< "AN1-AN2" << AN1AN2calc ;
    //qDebug()<< "AN1-AN2" << AN3AN4calc ;
}

// Decodes map indices (MapN, MapP)
void Apexi::decodeMapIndices(QByteArray rawmessagedata) {
    fc_map_info_t *info = reinterpret_cast<fc_map_info_t *>(rawmessagedata.data());

    packageMap[0] = mul[0] * info->Map_N + add[0]; // rpm (column)
    packageMap[1] = mul[0] * info->Map_P + add[0]; // load (row)
}

void Apexi::decodeBasic(QByteArray rawmessagedata) {
    fc_Basic_info_t *info = reinterpret_cast<fc_Basic_info_t *>(rawmessagedata.data());

    qreal Boost;
    int checkboost = (unsigned char) rawmessagedata[13];
    packageBasic[0] = mul[15] * info->Basic_Injduty + add[0];
    packageBasic[1] = info->Basic_IGL;
    packageBasic[2] = info->Basic_IGT;
    packageBasic[3] = mul[0] * info->Basic_RPM + add[0];
    packageBasic[4] = mul[0] * info->Basic_KPH + add[0];
    packageBasic[5] = mul[0] * info->Basic_Boost;
    packageBasic[6] = mul[0] * info->Basic_Knock + add[0];
    packageBasic[7] = mul[0] * info->Basic_Watert + add[8];
    packageBasic[8] = mul[0] * info->Basic_Airt + add[8];
    packageBasic[9] = mul[15] * info->Basic_BattV + add[0];

    if (checkboost == 128) {
        int test = (unsigned char) rawmessagedata[12];
        Boost = test * 0.01;
    } else {
        Boost = (packageBasic[5] - 760);
    }

/*
    else{
       // qDebug()<< "Mdl1" << packageBasic[5];
        if (packageBasic[5] >= 0)
        {
            Boost = (packageBasic[5] -760) * 0.01;
        }
        else{
            Boost = packageBasic[5] -760; // while boost pressure is negative show pressure in mmhg
        }
      }
*/
    //m_dashboard->setInjDuty(packageBasic[0]);
    m_dashboard->setLeadingign(packageBasic[1]);
    m_dashboard->setTrailingign(packageBasic[2]);
    m_dashboard->setrpm(packageBasic[3]);
    m_dashboard->setSpeed(packageBasic[4]);
    m_dashboard->setBoostPres(Boost);
    m_dashboard->setKnock(packageBasic[6]);
    m_dashboard->setWatertemp(packageBasic[7]);
    m_dashboard->setIntaketemp(packageBasic[8]);
    m_dashboard->setBatteryV(packageBasic[9]);
/*
    QString fileName = "Basic.txt";
    QFile mFile(fileName);
    if(!mFile.open(QFile::Append | QFile::Text)){
    }
    QTextStream out(&mFile);
    out << rawmessagedata.toHex() <<endl;
    mFile.close();
*/
}

/*
  
void Apexi::decodeVersion(QByteArray rawmessagedata)
{
    //    ui->lineVersion->setText (QString(rawmessagedata).mid(2,5));
}
*/
void Apexi::decodeInit(QByteArray rawmessagedata) {
    const QString modelname = QString(rawmessagedata).mid(2, 8);
    //Mazda
    if (modelname == "13B1    " || modelname == "13B-REW " || modelname == "13B-REW2" || modelname == "13B-REW3" ||
        modelname == "13BT1PRO" || modelname == "13BR1PRO" || modelname == "13BR2PRO" || modelname == "13BR3PRO") {
        Model = 1;
    }
    //Nissan
    if (modelname == "NISSAN-L" || modelname == "CA18DET " || modelname == "SR20DE1 " || modelname == "SR20DE2 " ||
        modelname == "SR20DE3 " || modelname == "SR20DE4 " || modelname == "SR20DET1" || modelname == "SR20DET2" ||
        modelname == "SR20DET3" || modelname == "SR20DET4" || modelname == "SR20DET5" || modelname == "SR20DET6" ||
        modelname == "RB20DET " || modelname == "RB25DET " || modelname == "RB25DET2" || modelname == "RB26DETT" ||
        modelname == "VG30DETT" || modelname == "CA181PRO" || modelname == "SR2N1PRO" || modelname == "SR2N2PRO" ||
        modelname == "SR2N3PRO" || modelname == "SR2N4PRO" || modelname == "SR201PRO" || modelname == "SR202PRO" ||
        modelname == "SR203PRO" || modelname == "SR204PRO" || modelname == "SR205PRO" || modelname == "SR206PRO" ||
        modelname == "RB201PRO" || modelname == "RB251PRO" || modelname == "RB252PRO" || modelname == "RB261PRO" ||
        modelname == "RB262PRO" || modelname == "RB26Pro " || modelname == "RB26PRO " || modelname == "RB26PRO1" ||
        modelname == "RB25PRO2" || modelname == "CA18T1-D" || modelname == "SR20T1-D" || modelname == "SR20T2-D" ||
        modelname == "SR20T3-D" || modelname == "SR20T4-D" || modelname == "SR20T5-D" || modelname == "RB26_1-D" ||
        modelname == "RB26_2-D" || modelname == "VG30TT-D") {
        Model = 2;
    }
    //Toyota
    if (modelname == "TOYOTA-L" || modelname == "1ZZ-FE  " || modelname == "1ZZ-FET " || modelname == "2ZZ-GE  " ||
        modelname == "3S-GE   " || modelname == "3SGET   " || modelname == "1JZ-GTE " || modelname == "1JZGT-AT" ||
        modelname == "4A-G1   " || modelname == "4A-G2   " || modelname == "1JGT1PRO" || modelname == "TOYOTA-D" ||
        modelname == "4A-GE   " || modelname == "4A-GE1  " || modelname == "4A-GE2  " || modelname == "4A-GE3  " ||
        modelname == "4AGE1-TH" || modelname == "4AGE2-TH" || modelname == "4AGE3-TH" || modelname == "4E-FTE1 " ||
        modelname == "4E-FTE2 " || modelname == "1JZGT-D " || modelname == "3S-GE1  " || modelname == "3S-GE2  " ||
        modelname == "3S-GTE  " || modelname == "3S-GTE2 " || modelname == "3S-GTE3 " || modelname == "1JZ-GTE2" ||
        modelname == "1JZ-GTE3" || modelname == "2JZ-GTE1" || modelname == "2JZ-GTE2" || modelname == "4AGE1PRO" ||
        modelname == "4AGE2PRO" || modelname == "4AGE3PRO" || modelname == "4EFT1PRO" || modelname == "4EFT2PRO" ||
        modelname == "3SGE1PRO" || modelname == "3SGT1PRO" || modelname == "3SGT2PRO" || modelname == "3SGT3PRO" ||
        modelname == "1JGT2PRO" || modelname == "1JGT3PRO" || modelname == "2JGT1PRO" || modelname == "2JGT2PRO") {
        Model = 3;
    }
    //Subaru
    if (modelname == "EJ20G   " || modelname == "EJ20K   " || modelname == "EJ207   " || modelname == "EJ20R   " ||
        modelname == "EJ20GPRO") {
        Model = 2;
    }
    //Honda
    if (modelname == "D15B    " || modelname == "B16A1   " || modelname == "B16A-US " || modelname == "B16A2   " ||
        modelname == "B16A1-TH" || modelname == "B16B    " || modelname == "B16B2   " || modelname == "B16BT   " ||
        modelname == "B16B1-TH" || modelname == "B16B2-TH" || modelname == "B18C    " || modelname == "B18C-US " ||
        modelname == "B18C2   " || modelname == "B18CT   " || modelname == "B18C1-TH" || modelname == "B16A1PRO" ||
        modelname == "B16A2PRO" || modelname == "B16B1PRO" || modelname == "B16B2PRO" || modelname == "B18C1PRO" ||
        modelname == "H22A    ") {
        Model = 2;
    }
    //Mitsubishi
    if (modelname == "4G63    " || modelname == "4G63-US " || modelname == "4G63-3  " || modelname == "4G63-5  " ||
        modelname == "4G63-6  " || modelname == "4G63-7  " || modelname == "4G63D_US" || modelname == "4G63-D  " ||
        modelname == "4G63-D3 " || modelname == "4G63-D4 " || modelname == "4G63-D5 " || modelname == "4G63-D6 " ||
        modelname == "4G63-D7 ") {
        Model = 3;
    }
    Model = 3;
    if (logLevel > 0) {
        cout << "Decode Init. Platform: " << modelname.toStdString() << " Model: "
             << Model << " Ecu Idx: " << m_dashboard->ecu() << endl;
    }
    m_dashboard->setPlatform(modelname);
}

void Apexi::decodeSensorStrings(QByteArray rawmessagedata) {

    m_dashboard->setSensorString1(QString(rawmessagedata).mid(2, 4));
    m_dashboard->setSensorString2(QString(rawmessagedata).mid(6, 4));
    m_dashboard->setSensorString3(QString(rawmessagedata).mid(10, 4));
    m_dashboard->setSensorString4(QString(rawmessagedata).mid(14, 4));
    m_dashboard->setSensorString5(QString(rawmessagedata).mid(18, 4));
    m_dashboard->setSensorString6(QString(rawmessagedata).mid(22, 4));
    m_dashboard->setSensorString7(QString(rawmessagedata).mid(26, 4));
    m_dashboard->setSensorString8(QString(rawmessagedata).mid(30, 4));


    m_dashboard->setFlagString1(QString(rawmessagedata).mid(34, 3));
    m_dashboard->setFlagString2(QString(rawmessagedata).mid(37, 3));
    m_dashboard->setFlagString3(QString(rawmessagedata).mid(40, 3));
    m_dashboard->setFlagString4(QString(rawmessagedata).mid(43, 3));
    m_dashboard->setFlagString5(QString(rawmessagedata).mid(46, 3));
    m_dashboard->setFlagString6(QString(rawmessagedata).mid(49, 3));
    m_dashboard->setFlagString7(QString(rawmessagedata).mid(52, 3));
    m_dashboard->setFlagString8(QString(rawmessagedata).mid(55, 3));
    m_dashboard->setFlagString9(QString(rawmessagedata).mid(58, 3));
    m_dashboard->setFlagString10(QString(rawmessagedata).mid(61, 3));
    m_dashboard->setFlagString11(QString(rawmessagedata).mid(64, 3));
    m_dashboard->setFlagString12(QString(rawmessagedata).mid(67, 3));
    m_dashboard->setFlagString13(QString(rawmessagedata).mid(70, 3));
    m_dashboard->setFlagString14(QString(rawmessagedata).mid(73, 3));
    m_dashboard->setFlagString15(QString(rawmessagedata).mid(76, 3));
    m_dashboard->setFlagString16(QString(rawmessagedata).mid(79, 3));
}

void Apexi::calculatorAux(float aux1min, float aux2max, float aux3min, float aux4max, QString Auxunit1, QString Auxunit2) {
    auxval1 = aux1min;
    auxval2 = aux2max;
    auxval3 = aux3min;
    auxval4 = aux4max;
    Auxname1 = Auxunit1;
    Auxname2 = Auxunit2;
    qDebug() << Auxunit1 << auxval1 << auxval2 << Auxunit2 << auxval3 << auxval4;
}

void Apexi::writeDashfile(const QString &gauge1, const QString &gauge2, const QString &gauge3, const QString &gauge4,
                          const QString &gauge5, const QString &gauge6) {
//Creates the dashboard file for the Apexi Dash

    QString filename = "UserDashApexi.txt";
    QFile file(filename);
    if (file.open(QIODevice::ReadWrite | QIODevice::Truncate | QIODevice::Text)) {
        QTextStream stream(&file);
        stream << gauge1 << endl;
        stream << gauge2 << endl;
        stream << gauge3 << endl;
        stream << gauge4 << endl;
        stream << gauge5 << endl;
        stream << gauge6 << endl;
    }

    QString filename2 = "/home/pi/UserDashboards/UserDashApexi.txt";
    QFile file2(filename2);
    if (file2.open(QIODevice::ReadWrite | QIODevice::Truncate | QIODevice::Text)) {
        QTextStream stream(&file2);
        stream << gauge1 << endl;
        stream << gauge2 << endl;
        stream << gauge3 << endl;
        stream << gauge4 << endl;
        stream << gauge5 << endl;
        stream << gauge6 << endl;
    }
}
