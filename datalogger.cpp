#include "datalogger.h"
#include "dashboard.h"
#include <QFile>
#include <QTextStream>
#include <QThread>
#include <QDebug>

// Run this as a thread and update every 50 ms
// still need to find a way to make this configurable
QTime loggerStartT;
QFile logFile;
bool logging = false;
int minLoggingRPM = 100;

datalogger::datalogger(QObject *parent)
    : QObject(parent)
    , m_dashboard(Q_NULLPTR) {
}

datalogger::datalogger(DashBoard *dashboard, QObject *parent)
    : QObject(parent)
    , m_dashboard(dashboard) {
}

void datalogger::startLog() {
    connect(&m_updatetimer, &QTimer::timeout, this, &datalogger::updateLog);
    loggerStartT = QTime::currentTime();
    m_updatetimer.start(100);
}

void datalogger::stopLog() {
    logging = false;
    logFile.close();
    m_updatetimer.stop();
}

void datalogger::updateLog() {
    if (logging) {
        // Stop logging when engine is shut down
        if (m_dashboard->rpm() < minLoggingRPM) {
            logging = false;
            logFile.close();
        }
    } else {
        // Not logging, check if we need to start logging after engine is started
        if (m_dashboard->rpm() >= minLoggingRPM) {
            const QString logFileName = QDate::currentDate().toString("log_dd.MM.yyyy") +
                    QTime::currentTime().toString("_hh.mm.ss") +
                    ".csv";
            logFile = QFile::QFile(logFileName);
            if (logFile.open(QIODevice::ReadWrite)) {
                logging = true;
                datalogger::createHeader();
            }
        }
    }

    if (logging) {
        QTextStream textStream(&logFile);
        switch (m_dashboard->ecu()) {
            case 1: ////Apexi ECU
                textStream << (loggerStartT.msecsTo(QTime::currentTime()) / 1000.0) << ","
                    << m_dashboard->rpm() << ","
                    << m_dashboard->InjDuty() << ","
                    << m_dashboard->Leadingign() << ","
                    << (m_dashboard->PressureV() * 1000.0) << ","
                    << m_dashboard->speed() << ","
                    << m_dashboard->Knock() << ","
                    << m_dashboard->Watertemp() << ","
                    << m_dashboard->Intaketemp() << ","
                    << m_dashboard->BatteryV() << ","
                    << m_dashboard->auxcalc3() << "," // Oil Press
                    << m_dashboard->auxcalc1() << "," // Oil Temp
                    << m_dashboard->auxcalc2() << "," // Wideband
                    << (m_dashboard->mapP() + 1) << "," // Map N, Map P is zero based
                    << (m_dashboard->mapN() + 1) << ","  // but in FCEdit start from 1
                    << m_dashboard->PressureV() << ","
                    << m_dashboard->ThrottleV() << ","
                    << m_dashboard->injms() << "," // Not working
                    << m_dashboard->Dwell() << "," // Not working
                    << endl;
                break;
            case 0: ////Link ECU Generic CAN
                textStream << (loggerStartT.msecsTo(QTime::currentTime())) << ","
                    << m_dashboard->rpm() << ","
                    << m_dashboard->MAP() << ","
                    << "MGP" << ","
                    << m_dashboard->ambipress() << ","
                    << m_dashboard->TPS() << ","
                    << m_dashboard->InjDuty() << ","
                    << m_dashboard->InjDuty2() << ","
                    << m_dashboard->injms() << ","
                    << m_dashboard->Watertemp() << ","
                    << m_dashboard->Intaketemp() << ","
                    << m_dashboard->BatteryV() << ","
                    << m_dashboard->MAFactivity() << ","
                    << m_dashboard->Gear() << ","
                    << m_dashboard->InjAngle() << ","
                    << m_dashboard->Ign() << ","
                    << m_dashboard->incamangle1() << ","
                    << m_dashboard->incamangle2() << ","
                    << m_dashboard->excamangle1() << ","
                    << m_dashboard->excamangle2() << ","
                    << m_dashboard->LAMBDA() << ","
                    << m_dashboard->lambda2() << ","
                    << "Trig 1 Error Counter" << ","
                    << m_dashboard->Error() << ","
                    << m_dashboard->FuelPress() << ","
                    << m_dashboard->oiltemp() << ","
                    << m_dashboard->oilpres() << ","
                    << m_dashboard->wheelspdftleft() << ","
                    << m_dashboard->wheelspdrearleft() << ","
                    << m_dashboard->wheelspdftright() << ","
                    << m_dashboard->wheelspdrearright() << ","
                    << m_dashboard->Knock() << ","
                    << "Knock Level 2" << ","
                    << "Knock Level 3" << ","
                    << "Knock Level 4" << ","
                    << "Knock Level 5" << ","
                    << "Knock Level 6" << ","
                    << "Knock Level 7" << ","
                    << "Knock Level 8" << ","
                    << m_dashboard->currentLap() << ","
                    << m_dashboard->laptime() << ","
                    << endl;
                break;
            case 2: ////Toyota86 BRZ FRS
                textStream << (loggerStartT.msecsTo(QTime::currentTime())) << ","
                    << m_dashboard->rpm() << ","
                    << m_dashboard->Watertemp() << ","
                    << m_dashboard->oiltemp() << ","
                    << m_dashboard->wheelspdftleft() << ","
                    << m_dashboard->wheelspdrearleft() << ","
                    << m_dashboard->wheelspdftright() << ","
                    << m_dashboard->wheelspdrearright() << ","
                    << m_dashboard->SteeringWheelAngle() << ","
                    << m_dashboard->brakepress() << ","
                    << m_dashboard->gpsTime() << ","
                    << m_dashboard->gpsAltitude() << ","
                    << m_dashboard->gpsLatitude() << ","
                    << m_dashboard->gpsLongitude() << ","
                    << m_dashboard->gpsSpeed() << ","
                    << m_dashboard->currentLap() << ","
                    << m_dashboard->laptime() << ","
                    << endl;
                break;
            case 5: ////ECU MASTERS EMU CAN
                textStream << (loggerStartT.msecsTo(QTime::currentTime())) << ","
                    << m_dashboard->rpm() << ","
                    << m_dashboard->TPS() << ","
                    << m_dashboard->injms() << ","
                    << m_dashboard->speed() << ","
                    << m_dashboard->ambipress() << ","
                    << m_dashboard->oiltemp() << ","
                    << m_dashboard->oilpres() << ","
                    << m_dashboard->FuelPress() << ","
                    << m_dashboard->Watertemp() << ","
                    << m_dashboard->Ign() << ","
                    << m_dashboard->Dwell() << ","
                    << m_dashboard->LAMBDA() << ","
                    << m_dashboard->LAMBDA() << ","
                    << m_dashboard->egt1() << ","
                    << m_dashboard->egt2() << ","
                    << m_dashboard->Gear() << ","
                    << m_dashboard->BatteryV() << ","
                    << m_dashboard->fuelcomposition() << ","
                    << m_dashboard->Analog1() << ","
                    << m_dashboard->Analog2() << ","
                    << m_dashboard->Analog3() << ","
                    << m_dashboard->Analog4() << ","
                    << m_dashboard->Analog5() << ","
                    << m_dashboard->Analog6() << ","
                    << m_dashboard->gpsTime() << ","
                    << m_dashboard->gpsAltitude() << ","
                    << m_dashboard->gpsLatitude() << ","
                    << m_dashboard->gpsLongitude() << ","
                    << m_dashboard->gpsSpeed() << ","
                    << m_dashboard->currentLap() << ","
                    << m_dashboard->laptime() << ","
                    << endl;
                break;
        }
    }
    // Trigger next logging cycle
    m_updatetimer.start(50);
}


void datalogger::createHeader() {
    QTextStream textStream(&logFile);
    switch(m_dashboard->ecu()) {
        case 1: //Apexi
            textStream << "Time(S)" << ","
                << "RPM" << ","
                << "InjDuty" << ","
                << "IgnTmng" << ","
                << "AirFlow" << ","
                << "Speed" << ","
                << "Knock" << ","
                << "WtrTemp" << ","
                << "AirTemp" << ","
                << "BatVolt" << ","
                << "OilPress" << "," // TODO use be aux name
                << "OilTemp" << ","  // TODO use be aux name
                << "WideBand" << "," // TODO use be aux name
                << "MAPP" << ","
                << "MAPN" << ","
                << "AFL V" << ","
                << "VTA V" << ","
                << "Inj ms" << ","
                << "Dwell" << ","
                << endl;
            break;
        case 0: ////Link ECU Generic CAN
            textStream << "Time ms" << ","
                << "RPM" << ","
                << "MAP"  << ","
                << "MGP"  << ","
                << "Barometric Pressure"  << ","
                << "TPS"  << ","
                << "Injector DC (pri)"  << ","
                << "Injector DC (sec)"  << ","
                << "Injector Pulse Width (Actual)"  << ","
                << "ECT"  << ","
                << "IAT"  << ","
                << "ECU Volts"  << ","
                << "MAF"  << ","
                << "Gear Position"  << ","
                << "Injector Timing"  << ","
                << "Ignition Timing"  << ","
                << "Cam Inlet Position L"  << ","
                << "Cam Inlet Position R"  << ","
                << "Cam Exhaust Position L"  << ","
                << "Cam Exhaust Position R"  << ","
                << "Lambda 1"  << ","
                << "Lambda 2"  << ","
                << "Trig 1 Error Counter"  << ","
                << "Fault Codes"  << ","
                << "Fuel Pressure" << ","
                << "Oil Temp " << ","
                << "Oil Pressure" << ","
                << "LF Wheel Speed"  << ","
                << "LR Wheel Speed"  << ","
                << "RF Wheel Speed"  << ","
                << "RR Wheel Speed"  << ","
                << "Knock Level 1"  << ","
                << "Knock Level 2"  << ","
                << "Knock Level 3"  << ","
                << "Knock Level 4"  << ","
                << "Knock Level 5"  << ","
                << "Knock Level 6"  << ","
                << "Knock Level 7"  << ","
                << "Knock Level 8"  << ","
                << "Current LAP"    << ","
                << "LAP TIME"       << ","
                << endl;
            break;
        case 2: ////Toyota86 BRZ FRS
            textStream << "Time ms" << ","
                << "RPM" << ","
                << "Coolant Temp" << ","
                << "Oil Temp" << ","
                << "LF Wheel Speed"  << ","
                << "LR Wheel Speed"  << ","
                << "RF Wheel Speed"  << ","
                << "RR Wheel Speed"  << ","
                << "Steering Wheel Angle "<< ","
                << "Brake Pressure" ","
                << "GPS Time"   << ","
                << "GPS Altitude"  << ","
                << "GPS Latitude" << ","
                << "GPS Longitude"   << ","
                << "GPS Speed"  << ","
                << "Current LAP"    << ","
                << "LAP TIME"  << ","
                << endl;
            break;
        case 5: ////EMU CAN
            textStream << "Time ms" << ","
                << "RPM" << ","
                << "TPS" << ","
                << "IAT" << ","
                << "MAP"  << ","
                << "Inj PW (ms)"  << ","
                << "Speed"  << ","
                << "Barometric Pressure"  << ","
                << "Oil Temp"<< ","
                << "Oil Pressure" ","
                << "Fuel Pressure" ","
                << "Coolant Temp" ","
                << "Ignition Angle" ","
                << "Dwell (ms)" ","
                << "LAMDA λ" ","
                << "LAMDA Corr. %" ","
                << "EGT 1" ","
                << "EGT 2" ","
                << "Gear" ","
                << "Battery V" ","
                << "Ethanol %" ","
                << "Analog 1 V" ","
                << "Analog 2 V" ","
                << "Analog 3 V" ","
                << "Analog 4 V" ","
                << "Analog 5 V" ","
                << "Analog 6 V" ","
                << "GPS Time"   << ","
                << "GPS Altitude"  << ","
                << "GPS Latitude" << ","
                << "GPS Longitude"   << ","
                << "GPS Speed"  << ","
                << "Current LAP"    << ","
                << "LAP TIME"  << ","
                << endl;
            break;
    }
}
