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
 * Depicts what is measured via AN1-2.
 */
QString Auxname1;

/**
 * Depicts what is measured via AN3-4.
 */
QString Auxname2;

/**
 * Depicts what is measured via AN5-6.
 */
QString Auxname3;

/**
 * The maximum voltage of the aux ports.
 */
const double MAX_AUX_VOLT = 5.0;
/**
 * Transform raw int value to volt for datalogit black.
 */
const double AUX_BLACK_VOLT_TRANSFORM = 1.0 / 205.0;
/**
 * The value of the calculation for AN1-2 at volt 0
 */
float an1_2volt0;

/**
 * The value of the calculation for AN1-2 at volt 5
 */
float an1_2volt5;

/**
 * The value of the calculation for AN3-4 at volt 0
 */
float an3_4volt0;

/**
 * The value of the calculation for AN3-4 at volt 5
 */
float an3_4volt5;

/**
 * The value of the calculation for AN5-6 at volt 0
 */
float an5_6volt0 = 0;

/**
 * The value of the calculation for AN5-6 at volt 5
 */
float an5_6volt5 = 0;

/**
 * The value of the calculation for AN7-8 at volt 0
 */
float an7_8volt0 = 0;

/**
 * The value of the calculation for AN7-8 at volt 5
 */
float an7_8volt5 = 0;

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
 * '2' for black datalogit, '1' for white (old)
 */
char majorDatalogitVersion;
const char V2_DATALOGIT = '2';
const char V1_DATALOGIT = '1';

/**
 * Holds the number of expected bytes of the next PFC response.
 */
int expectedbytes;

int requestIndex = 0; // ID for requested data type Power FC
struct ReadPacket {
    QByteArray bytes;
    int responseSize;
};
const ReadPacket READ_REQUESTS[17] = {
    { QByteArray::fromHex("F3020A"),  11 }, // Platform (i.e ' 2ZZ-GE ')
    { QByteArray::fromHex("0102FC"),   8 }, // Datalogit Version (i.e 'V2.0.')
    { QByteArray::fromHex("F50208"),   8 }, // Platform version (i.e. '2.71A')
    { QByteArray::fromHex("DD0220"),  83 }, // Sensor labels (i.e. '2.71A')
    { QByteArray::fromHex("B0024D"), 103 }, // Fuel map (request 1 of 8)
    { QByteArray::fromHex("B1024C"), 103 }, // Fuel map (request 2 of 8)
    { QByteArray::fromHex("B2024B"), 103 }, // Fuel map (request 3 of 8)
    { QByteArray::fromHex("B3024A"), 103 }, // Fuel map (request 4 of 8)
    { QByteArray::fromHex("B40249"), 103 }, // Fuel map (request 5 of 8)
    { QByteArray::fromHex("B50248"), 103 }, // Fuel map (request 6 of 8)
    { QByteArray::fromHex("B60247"), 103 }, // Fuel map (request 7 of 8)
    { QByteArray::fromHex("B70246"), 103 }, // Fuel map (request 8 of 8)
    // Live data
    { QByteArray::fromHex("F0020D"),  33 }, // Advanced data
    { QByteArray::fromHex("DB0222"),   5 }, // Map indices
    { QByteArray::fromHex("DE021F"),  21 }, // Sensor data
    { QByteArray::fromHex("DA0223"),  23 }, // Basic data
    { QByteArray::fromHex("010300FB"),19 }  // Aux data (black)
};
// The request packet for aux data of the old datalogit
const ReadPacket V1_AUX_DATA = { QByteArray::fromHex("0002FD"), 7 };

const int MAX_REQUEST_IDX = 16;
const int INIT_REQUEST_IDX = 0;
const int FIRST_LIVE_DATA_REQUEST_IDX = 12;
const int AUX_DATA_REQUEST_IDX = 16;

// TODO find why this is a state variable
qreal advboost;

double mul[80] = FC_INFO_MUL;  // required values for calculation from raw to readable values for Advanced Sensor info
double add[] = FC_INFO_ADD;

/**
 * These two values depend on the TPS of the car and may need adjustment.
 */
const double MIN_TPS_VOLT = 0.56;
const double MAX_TPS_VOLT = 4.024;

/**
 * These values depend on the installed wideband sensor.
 */
const double MAX_AFR = 19.8;
const double MIN_AFR = 9.8;


/**
 * Limits the number of write requests to the fuel map.
 * A value lower than 8 will not write the entire fuel map.
 * Value 1 will write the first 50 values, ie first two coloums and the half of the third column.
 */
const int FUEL_MAP_MAX_WRITE_REQUESTS = 1;

/**
 * The following values decide when autotune will be active.
 */
const double MIN_AUTOTUNE_WATER_TEMP = 65;
const double MIN_AUTOTUNE_RPM = 500;
const double MAX_AUTOTUNE_RPM = 1100;
const double MAX_AUTOTUNE_TPS_CHANGE_RATE = 4; // volt / second
const double MIN_AUTOTUNE_TPS_CHANGE_RATE = -4;
const double MAX_AUTOTUNE_SPEED = 2; // km/h
const double MAX_AUTOTUNE_TPS_VOLT = 0.6;

/**
 * The "master switch" of the autotune.
 */
bool closedLoopEnabled = false;

// The last logged AFR value(-1 = uninitialized)
double loggedAFR = -1;
double lastTpsVolt = MIN_TPS_VOLT;
QTime lastLogTime = QTime::currentTime();

// Used for logging messages in fixed intervals
long logSamplesCount = 0;
// Log every this number of samples
const int LOG_SAMPLE_COUNT_INTERVAL = 10;

// Used to calculate and log the average value of aux3 (instead of logging each value)
// This is done for 'erratic' readings ex. oil pressure
double aux3Values[10];
int aux3CalcIdx = 0;
bool emitAux3Value = false;

Apexi::Apexi(QObject *parent)
        : QObject(parent), m_dashboard(Q_NULLPTR) {
}

Apexi::Apexi(DashBoard *dashboard, QObject *parent)
        : QObject(parent), m_dashboard(dashboard) {
}

void Apexi::initSerialPort() {
    if (LOG_LEVEL >= LOGGING_INFO) {
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
    if (LOG_LEVEL >= LOGGING_INFO) {
        cout << "Clearing serial port\n";
    }
    m_serialport->clear();
}

//function to open serial port
void Apexi::openConnection(const QString &portName) {
    cout << "Logging level:" << LOG_LEVEL << endl
         << "Log Interval:" << LOG_SAMPLE_COUNT_INTERVAL << endl
         << "Closed Loop:" << (closedLoopEnabled ? "Yes" : "No") << endl;
    if (LOG_LEVEL >= LOGGING_INFO) {
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
        if (LOG_LEVEL >= LOGGING_INFO) {
            cout << "Failed to open serial communication: " << m_serialport->errorString().toStdString() << endl;
        }
        m_dashboard->setSerialStat(m_serialport->errorString());
        Apexi::closeConnection();
    } else {
        if (LOG_LEVEL >= LOGGING_INFO) {
            cout << "Connected to Serial Port\n";
        }
        m_dashboard->setSerialStat(QString("Connected to Serialport"));
        requestIndex = INIT_REQUEST_IDX;
        Apexi::sendPfcReadRequest();
    }
}

void Apexi::closeConnection() {
    if (LOG_LEVEL >= LOGGING_INFO) {
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
    if (LOG_LEVEL >= LOGGING_INFO) {
        cout << "Retry connection\n";
    }
    Apexi::openConnection(port);
}

void Apexi::handleTimeout() {
    if (LOG_LEVEL >= LOGGING_INFO) {
        cout << "Handling timeout\n";
    }
    m_dashboard->setTimeoutStat(QString("Is Timeout : Y"));
    m_timer.stop();
    m_serialport->close();
    if (m_serialport->open(QIODevice::ReadWrite) == false) {
        if (LOG_LEVEL >= LOGGING_INFO) {
            cout << "Failed to open serial communication: " << m_serialport->errorString().toStdString() << endl;
        }
        m_dashboard->setSerialStat(m_serialport->errorString());
    } else {
        if (LOG_LEVEL >= LOGGING_INFO) {
            cout << "Connected to Serial Port\n";
        }
        m_dashboard->setSerialStat(QString("Connected to Serialport"));
    }

    requestIndex = INIT_REQUEST_IDX; // TODO or just continue with live data?
    m_readData.clear();

    Apexi::sendPfcReadRequest();
}

void Apexi::handleError(QSerialPort::SerialPortError serialPortError) {
    if (LOG_LEVEL >= LOGGING_INFO) {
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
    if (LOG_LEVEL >= LOGGING_DEBUG) {
        cout << "readyToRead callback." << endl;
    }
    m_readData = m_serialport->readAll();
    m_dashboard->setRecvData(QString("Receive Data : " + m_readData.toHex()));
    Apexi::decodeResponseAndSendNextRequest(m_readData);
}

/**
 * Responsible to decode the provided PFC response and send the next request.
 *
 * @param buffer the raw PFC response
 */
void Apexi::decodeResponseAndSendNextRequest(const QByteArray &buffer) {
    if (LOG_LEVEL >= LOGGING_DEBUG) {
        cout << "decodeResponseAndSendNextRequest: " << buffer.toHex().toStdString() << endl;
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

        if (closedLoopEnabled && isNotWOT() && handleNextFuelMapWriteRequest(FUEL_MAP_MAX_WRITE_REQUESTS)) {
            // It is safe to write the fuel to the PFC and
            // fuel map should be updated; live data acquisition will be stopped until the map is sent to PFC

            if (LOG_LEVEL >= LOGGING_INFO && getCurrentFuelMapWriteRequest() == 1) {
                cout << "\nWriting fuel map..." << endl;
                logFuelData(10);
            }
            QByteArray writePacket = QByteArray::fromRawData(getNextFuelMapWritePacket(), MAP_WRITE_PACKET_LENGTH);
            if (LOG_LEVEL >= LOGGING_DEBUG) {
                cout << "Sending map write packet: " << writePacket.toHex().toStdString() << endl;
            }
            Apexi::writeRequestPFC(writePacket);
            //TODO should verify that the ack packet is actually received
            expectedbytes = 3; // ack packet (0xF2 0x02 0x0B) is expected

            // Set closed loop status to 2 -> "Writing new map"
            m_dashboard->setClosedLoop(2);
            m_timer.start(700);
        } else {
            // Decide the next request to be sent to PFC
            if (requestIndex < MAX_REQUEST_IDX) {
                // Once go through all requests
                requestIndex++;
            } else {
                // then cycle through live data requests ADV_DATA_REQUEST..AUX_REQUEST (adv data, map idx, sensor data, basic data, aux)
                requestIndex = FIRST_LIVE_DATA_REQUEST_IDX;
                logSamplesCount++;
                // New cycle of live data, so update the afr logs with the previous data
                updateAutoTuneLogs();
            }
            Apexi::sendPfcReadRequest();
        }
    }
}

/**
 * Decides if we are NOT in WOT conditions.
 */
bool Apexi::isNotWOT() {
    const double tpsVolt = (double) m_dashboard->ThrottleV();
    return tpsVolt < MAX_AUTOTUNE_TPS_VOLT;
}

void Apexi::updateAutoTuneLogs() {
    const QTime now = QTime::currentTime();

    const int rpmIdx = packageMap[0]; // col MapN
    const int loadIdx = packageMap[1];// row MapP
    const double speed = (double) m_dashboard->speed();
    const double rpm = (double) m_dashboard->rpm(); // packageBasic[3];
    const double waterTemp = (double) m_dashboard->Watertemp(); // packageBasic[7];
    const double tpsVolt = (double) m_dashboard->ThrottleV();

    const double timeDeltaSeconds = lastLogTime.msecsTo(now) / 1000.0;
    const double tpsChangeRate = (tpsVolt - lastTpsVolt) / timeDeltaSeconds;
    lastLogTime = now;
    lastTpsVolt = tpsVolt;

    bool shouldUpdateAfr = true;
    if (!closedLoopEnabled) {
        // Update AFR only when the closed loop is enabled
        m_dashboard->setClosedLoop(0); // "Off: Closed Loop Disabled");
        shouldUpdateAfr = false;
    } else if (loggedAFR < MIN_AFR || loggedAFR > MAX_AFR) {
        // AFR value should be within some bounds
        m_dashboard->setClosedLoop(0); // "Off: AFR out of bounds");
        shouldUpdateAfr = false;
    } else if (waterTemp < MIN_AUTOTUNE_WATER_TEMP) {
        // Engine should be warmed up
        m_dashboard->setClosedLoop(0); // "Off: Engine not warmed up");
        shouldUpdateAfr = false;
    } else if (rpm < MIN_AUTOTUNE_RPM || rpm > MAX_AUTOTUNE_RPM) {
        // Engine is actually started and revving up to a certain RPM
        m_dashboard->setClosedLoop(0); // "Off: RPM to low or to high");
        shouldUpdateAfr = false;
    } else if (tpsChangeRate > MAX_AUTOTUNE_TPS_CHANGE_RATE || tpsChangeRate < MIN_AUTOTUNE_TPS_CHANGE_RATE){
        // Do not auto tune on sudden throttle changes (do not mess with accel enrich etc)
        m_dashboard->setClosedLoop(0); // "Off: Accel enrich or decel cut");
        shouldUpdateAfr = false;
    } else if(tpsVolt > MAX_AUTOTUNE_TPS_VOLT) {
        // Do not autotune near WOT
        m_dashboard->setClosedLoop(0); // "Off: WOT");
        shouldUpdateAfr = false;
    } else if (speed > MAX_AUTOTUNE_SPEED) {
        // Do not autotune when moving.
        m_dashboard->setClosedLoop(0); // "Off: Moving");
        shouldUpdateAfr = false;
    } else {
        m_dashboard->setClosedLoop(1); // "Active");
        shouldUpdateAfr = true;
    }

    if (shouldUpdateAfr) {
        updateAFRData(rpmIdx, loadIdx, loggedAFR);
    }

    if (LOG_LEVEL >= LOGGING_DEBUG && (logSamplesCount % LOG_SAMPLE_COUNT_INTERVAL) == 0) {
        cout << lastLogTime.toString("hh:mm:ss.zzz").toStdString()
             << setprecision(3) << fixed
             << ", ClosedLoopEnabled:" << (closedLoopEnabled ? "Yes" : "No")
             << ", AutoTuning:" << (shouldUpdateAfr ? "Yes" : "No")
             << ", WaterTemp:" << waterTemp
             << ", MapN:" << rpmIdx
             << ", MapP:" << loadIdx
             << ", Rpm:" << rpm
             << ", Speed:" << speed
             << ", AFR:" << loggedAFR
             << ", Tps:" << tpsVolt
             << ", TimeDelta:" << timeDeltaSeconds
             << ", TpsChangeRate:" << tpsChangeRate << endl;
    }
}

void Apexi::decodePfcData(QByteArray rawmessagedata) {
    if (LOG_LEVEL >= LOGGING_DEBUG) {
        cout << "PFC response packet: " << rawmessagedata.toHex().toStdString() << endl;
    }
    if (rawmessagedata.length()) {
        //Power FC Decode
        const quint8 requesttype = rawmessagedata[0];
        const quint8 responseLength = rawmessagedata[1];
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
            case ID::AuxDataBlack:
                if (responseLength == 0x12) {
                    // 01 12 => black datalogit aux
                    Apexi::decodeAuxBlack(rawmessagedata);
                } else if (responseLength == 0x07){
                    // 01 07 => datalogit version
                    Apexi::decodeDatalogitVersion(rawmessagedata);
                }
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
                    requestIndex = INIT_REQUEST_IDX;
                    Apexi::closeConnection();
                    QTimer::singleShot(2000, this, SLOT(retryconnect()));
                }
                break;
            case ID::Version:
                // TODO
                cout << "Platform Version:" << rawmessagedata.toStdString() << endl;
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
                if (LOG_LEVEL >= LOGGING_INFO) {
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
    if (bytesWritten == -1) {
        m_dashboard->setSerialStat(m_serialport->errorString());
    } else if (bytesWritten != m_writeData.size()) {
        m_dashboard->setSerialStat(m_serialport->errorString());
    }

}

void Apexi::sendPfcReadRequest() {
    // Using only New Apexi Structure (Protocol 0), Protocol 1 never used
    ReadPacket readPacket;
    if(requestIndex == AUX_DATA_REQUEST_IDX && majorDatalogitVersion == V1_DATALOGIT) {
        // Override request aux data for version 1 datalogit (white)
        readPacket = V1_AUX_DATA;
    } else {
        readPacket = READ_REQUESTS[requestIndex];
    }
    if (LOG_LEVEL >= LOGGING_DEBUG) {
        cout << "sendPfcReadRequest: " << requestIndex << "->" << readPacket.bytes.toHex().toStdString() << endl;
    }
    Apexi::writeRequestPFC(readPacket.bytes);
    expectedbytes = readPacket.responseSize;

    // Set timout to 700 millis
    m_timer.start(700);
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


        // Skipping because already set by basic info
        // m_dashboard->setrpm(packageADV3[0]);
        // m_dashboard->setLeadingign(packageADV3[6]);
        // m_dashboard->setTrailingign(packageADV3[7]);
        // m_dashboard->setWatertemp(packageADV3[10]);
        // m_dashboard->setIntaketemp(packageADV3[11]);
        // m_dashboard->setBatteryV(packageADV3[13]);
        // m_dashboard->setSpeed(packageADV3[14]);

        m_dashboard->setIntakepress(packageADV3[1]);
        m_dashboard->setPressureV(packageADV3[2]);
        m_dashboard->setThrottleV(packageADV3[3]);
        m_dashboard->setPrimaryinp(packageADV3[4]);
        m_dashboard->setFuelc(packageADV3[5]);
        m_dashboard->setpim(advboost);
        m_dashboard->setKnock(packageADV3[12]);
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
    if (LOG_LEVEL >= LOGGING_DEBUG) {
        cout << "Aux Packet:" << rawmessagedata.toHex().toStdString() << endl;
    }
    fc_aux_info_t *info = reinterpret_cast<fc_aux_info_t *>(rawmessagedata.data());

    packageAux[0] = mul[29] * info->AN1 + add[29];
    packageAux[1] = mul[29] * info->AN2 + add[29];
    packageAux[2] = mul[29] * info->AN3 + add[29];
    packageAux[3] = mul[29] * info->AN4 + add[29];

    if (LOG_LEVEL >= LOGGING_DEBUG) {
        cout << fixed << setprecision(3)
             << " An1:" <<  packageAux[0]
             << " An3:" <<  packageAux[2]
             << endl;
    }

    const double auxCalc1 = ((((an1_2volt5 - an1_2volt0) * 0.2) * (packageAux[0] - packageAux[1])) + an1_2volt0);
    const double auxCalc2 = ((((an3_4volt5 - an3_4volt0) * 0.2) * (packageAux[2] - packageAux[3])) + an3_4volt0);

    // TODO should be configurable, for now wideband is tied with An3-4
    loggedAFR = auxCalc2;

    m_dashboard->setauxcalc1(auxCalc1);
    m_dashboard->setauxcalc2(auxCalc2);
}

void Apexi::decodeAuxBlack(QByteArray rawmessagedata) {
    if (LOG_LEVEL >= LOGGING_DEBUG) {
        cout << "Aux(Black) Packet:" << rawmessagedata.toHex().toStdString() << endl;
    }

    fc_aux2_info_t *info = reinterpret_cast<fc_aux2_info_t *>(rawmessagedata.data());

    const double an1 = info -> AN1 * AUX_BLACK_VOLT_TRANSFORM;
    const double an2 = info -> AN2 * AUX_BLACK_VOLT_TRANSFORM;
    const double an3 = info -> AN3 * AUX_BLACK_VOLT_TRANSFORM;
    const double an4 = info -> AN4 * AUX_BLACK_VOLT_TRANSFORM;
    const double an5 = info -> AN5 * AUX_BLACK_VOLT_TRANSFORM;
    const double an6 = info -> AN6 * AUX_BLACK_VOLT_TRANSFORM;
    const double an7 = info -> AN7 * AUX_BLACK_VOLT_TRANSFORM;
    const double an8 = info -> AN8 * AUX_BLACK_VOLT_TRANSFORM;

    const double auxCalc1 = ((an1_2volt5 - an1_2volt0) / MAX_AUX_VOLT) * (an1 - an2) + an1_2volt0;
    const double auxCalc2 = ((an3_4volt5 - an3_4volt0) / MAX_AUX_VOLT) * (an3 - an4) + an3_4volt0;
    const double auxCalc3 = ((an5_6volt5 - an5_6volt0) / MAX_AUX_VOLT) * (an5 - an6) + an5_6volt0;
    const double auxCalc4 = ((an7_8volt5 - an7_8volt0) / MAX_AUX_VOLT) * (an7 - an8) + an7_8volt0;

    // TODO should be configurable, for now wideband is tied with An3-4
    loggedAFR = auxCalc2;

    m_dashboard->setauxcalc1(auxCalc1);
    m_dashboard->setauxcalc2(auxCalc2);

    if (aux3CalcIdx == 9) {
        emitAux3Value = true;
    }
    aux3Values[aux3CalcIdx] = auxCalc3;
    aux3CalcIdx = (aux3CalcIdx + 1) % 10;
    if (emitAux3Value) {
        double sum = 0;
        for (int i=0; i<10; i++) {
            sum += aux3Values[i];
        }
        m_dashboard->setauxcalc3(sum / 10.0);
    }

    m_dashboard->setauxcalc4(auxCalc4);

    if (LOG_LEVEL >= LOGGING_DEBUG) {
        cout << fixed << setprecision(3)
             << " an1_2volt0: " << an1_2volt0
             << " an1_2volt5: " << an1_2volt5
             << " an3_4volt0: " << an3_4volt0
             << " an3_4volt5: " << an3_4volt5
             << " an5_6volt0: " << an5_6volt0
             << " an5_6volt5: " << an5_6volt5
             << " an7_8volt0: " << an7_8volt0
             << " an7_8volt5: " << an7_8volt5 << endl;

        cout << fixed << setprecision(3)
             << " AN1: " << an1
             << " AN2: " << an2
             << " AN3: " << an3
             << " AN4: " << an4
             << " AN5: " << an5
             << " AN6: " << an6
             << " AN7: " << an7
             << " AN8: " << an8 << endl;

        cout << fixed << setprecision(3)
             << " auxCalc1: " << auxCalc1
             << " auxCalc2: " << auxCalc2
             << " auxCalc3: " << auxCalc3
             << " auxCalc4: " << auxCalc4 << endl;
    }
}

// Decodes map indices (MapN, MapP)
void Apexi::decodeMapIndices(QByteArray rawmessagedata) {
    fc_map_info_t *info = reinterpret_cast<fc_map_info_t *>(rawmessagedata.data());

    packageMap[0] = mul[0] * info->Map_N + add[0]; // rpm (column)
    packageMap[1] = mul[0] * info->Map_P + add[0]; // load (row)

    m_dashboard->setMapN(packageMap[0]);
    m_dashboard->setMapP(packageMap[1]);
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

    m_dashboard->setInjDuty(packageBasic[0]);
    m_dashboard->setLeadingign(packageBasic[1]);
    m_dashboard->setTrailingign(packageBasic[2]);
    m_dashboard->setrpm(packageBasic[3]);
    m_dashboard->setSpeed(packageBasic[4]);
    m_dashboard->setBoostPres(Boost);
    m_dashboard->setKnock(packageBasic[6]);
    m_dashboard->setWatertemp(packageBasic[7]);
    m_dashboard->setIntaketemp(packageBasic[8]);
    m_dashboard->setBatteryV(packageBasic[9]);
}

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
    if (modelname == "TOYOTA-L" || modelname == "1ZZ-FE  " || modelname == "1ZZ-FET " || modelname == " 2ZZ-GE " ||
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

    if (Model == 0) {
        cout << "Could not recognize platform:" << modelname.toStdString() << endl;
    }

    m_dashboard->setPlatform(modelname);

    if (LOG_LEVEL >= LOGGING_INFO) {
        cout << "Decode Init. Platform:" << modelname.toStdString() << ", Model: "
             << Model << ", Ecu Idx: " << m_dashboard->ecu() << endl;
    }
}

void Apexi::decodeDatalogitVersion(QByteArray rawmessagedata) {
    // Sample response from black datalogit
    // 01 07 56 32 2e 30 00 11
    //       V  2  .  0
    majorDatalogitVersion = rawmessagedata[3];
    if (LOG_LEVEL >= LOGGING_INFO) {
        cout << "Datalogit Version(raw):" << rawmessagedata.toStdString() << endl
             << "Decoded major version:" << majorDatalogitVersion << endl;
    }
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

void Apexi::enableClosedLoop(bool enable) {
    closedLoopEnabled = enable;
}

void Apexi::setAuxCalcData(float aux1min, float aux1max,
                           float aux2min, float aux2max,
                           float aux3min, float aux3max,
                           QString Auxunit1,
                           QString Auxunit2, QString Auxunit3) {
    an1_2volt0 = aux1min;
    an1_2volt5 = aux1max;
    an3_4volt0 = aux2min;
    an3_4volt5 = aux2max;
    an5_6volt0 = aux3min;
    an5_6volt5 = aux3max;
    Auxname1 = Auxunit1;
    Auxname2 = Auxunit2;
    Auxname3 = Auxunit3;

    if (LOG_LEVEL >= LOGGING_INFO) {
        cout << "AuxCalcData:" << endl
             << " an1_2volt0:" << an1_2volt0 << endl
             << " an1_2volt5:" << an1_2volt5 << endl
             << " an3_4volt0:" << an3_4volt0 << endl
             << " an3_4volt5:" << an3_4volt5 << endl
             << " an5_6volt0:" << an5_6volt0 << endl
             << " an5_6volt5:" << an5_6volt5 << endl;
    }
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
