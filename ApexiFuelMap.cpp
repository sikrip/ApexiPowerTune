#include "ApexuFuelMap.h"
#include <stdexcept>
#include <iostream>
#include <iomanip>

using namespace std;

/**
 * This is the map that is currently in the PFC.
 */
double currentFuelMap[FUEL_TABLE_SIZE][FUEL_TABLE_SIZE];

/**
 * The new adjusted map to be sent to PFC.
 * These values are set by calculating the exact row/column.
 */
double newFuelMap[FUEL_TABLE_SIZE][FUEL_TABLE_SIZE];

/**
 * The total number of logged AFR samples.
 */
long afrSamplesCount = 0;

/**
 * A table that holds the sum of the AFR values.
 */
double loggedSumAfrMap[FUEL_TABLE_SIZE][FUEL_TABLE_SIZE];

/**
 * A table that holds the total AFR samples per row/column.
 */
double loggedNumAfrMap[FUEL_TABLE_SIZE][FUEL_TABLE_SIZE];

/**
 * The current request number when writing the map to PFC.
 */
int fuelMapWriteRequest;

/**
 * Attempt to write the map after each this number of samples.
 */
int fuelMapWriteAttemptInterval = 100;

/**
 * This number of samples are required for each cell in order for this cell to
 * be eligible to be sent to PFC.
 * TODO should be table
 */
int minCellSamples = 10;

/**
 * This target AFR for each cell.
 * TODO should be table
 */
double targetAFR = 14.7;

/**
 * This amount of cell changes are required in order for the map to be sent to PFC.
 */
int minCellsChangesForWriteAttempt = 5;

/**
 * Counts how many times the fuel map was sent to PFC.
 */
int mapWriteCount = 0;

/**
 * Calculates the row of fuel map based on the provided fuel request number (1-FUEL_MAP_TOTAL_REQUESTS).
 *
 * @param fuelRequestNumber the fuel request number (1..FUEL_MAP_TOTAL_REQUESTS)
 * @return the fuel map row corresponding to the provided request
 */
int getFuelMapRow(int fuelRequestNumber) {
    // 1, 3, 5, 7 start with row 0; 2, 4, 6, 8 start at row 10
    return (fuelRequestNumber % 2 == 1) ? 0 : 10;
}

/**
  * Calculates the column of fuel map based on the provided fuel request number (1-FUEL_MAP_TOTAL_REQUESTS).
  *
  * @param fuelRequestNumber the fuel request number (1..FUEL_MAP_TOTAL_REQUESTS)
  * @return the fuel map column corresponding to the provided request
  */
int getFuelMapColumn(int fuelRequestNumber) {
    int cells = fuelRequestNumber * FUEL_CELLS_PER_REQUEST;
    return (cells / FUEL_TABLE_SIZE) - ((fuelRequestNumber % 2) ? 2 : 3);
}

/**
 * Reads the portion of the fuel map that corresponds to the provided request number.
 * The map is stored in both current and new fuel maps.
 *
 * @param fuelRequestNumber defines the part of the map to be read
 * @param rawData the PFC raw map data
 */
void readFuelMap(int fuelRequestNumber, const char rawData[]) {
    int row = getFuelMapRow(fuelRequestNumber);
    int col = getFuelMapColumn(fuelRequestNumber);

    // 0 = id, 1 = number of bytes, 2...101 = fuel table payload
    unsigned char packetId = rawData[0];
    if (packetId != 0xB0 + (fuelRequestNumber - 1)) {
        throw std::invalid_argument("Invalid packet id for fuel map read request");
    }
    unsigned char packetLength = rawData[1];
    if (packetLength != 102) {// the packet length does not contain the id
        throw std::invalid_argument("Invalid packet length for fuel map read request");
    }

    for (unsigned int i = 2; i < 102; i += 2) {
        unsigned char byte1 = rawData[i];
        unsigned char byte2 = rawData[i + 1];

        int fuelCellValue = (byte2 << 8) + byte1; // two byte big endian
        double humanFuelValue = (fuelCellValue * 4.0) / 1000.0;
        currentFuelMap[row][col] = humanFuelValue;

        // Initially the new fuel map is equal to the current.
        newFuelMap[row][col] = humanFuelValue;

        // Move to next row and column as needed
        row++;
        if (row == 20) {
            row = 0;
            col++;
        }
    }
}

/**
 * Creates a PFC write packet containing the fuel map portion that corresponds to the given request number.
 * An ack packet (0xF2 0x02 0x0B) is expected after this is sent to PFC.
 *
 * @param fuelRequestNumber the request number for the fuel map (1..FUEL_MAP_TOTAL_REQUESTS)
 * @param map the fuel map to send
 * @return the write packet for sending the fuel map to PFC
 */
char* createFuelMapWritePacket(int fuelRequestNumber, double (&map)[FUEL_TABLE_SIZE][FUEL_TABLE_SIZE]) {

    int row = getFuelMapRow(fuelRequestNumber);
    int col = getFuelMapColumn(fuelRequestNumber);
    union {
        int celValue;
        char celValueBytes[2];
    } fuelCell{};

    char* pfcDataPacket = new char[103];

    const char requestId = (char)(0xB0 + (fuelRequestNumber - 1));
    const char packetSize = 102;
    char checksum = (char)(255 - requestId - packetSize);

    pfcDataPacket[0] = requestId;
    pfcDataPacket[1] = packetSize;

    int cellsWritenCount = 0;
    int dataIdx = 2;
    while(cellsWritenCount < FUEL_CELLS_PER_REQUEST) {
        // from human readable format to PFC format
        fuelCell.celValue = (int)((map[row][col] * 1000.0) / 4.0);
        pfcDataPacket[dataIdx++] = fuelCell.celValueBytes[0];
        pfcDataPacket[dataIdx++] = fuelCell.celValueBytes[1];

        checksum -= fuelCell.celValueBytes[0];
        checksum -= fuelCell.celValueBytes[1];

        // Move to next row/col
        row++;
        if (row == FUEL_TABLE_SIZE) {
            row = 0;
            col++;
        }
        // move to next table cell
        cellsWritenCount++;
    }
    pfcDataPacket[dataIdx] = checksum;

    return pfcDataPacket;
}

/**
 * Creates a PFC write packet containing the fuel map portion that corresponds to the given request number.
 * The new fuel map is used to create the requests.
 * An ack packet (0xF2 0x02 0x0B) is expected after this is sent to PFC.
 */
char* getNextFuelMapWritePacket() {
    return createFuelMapWritePacket(fuelMapWriteRequest, newFuelMap);
}

/**
 * Updates the AFR in the provided position.
 *
 * @param rpmIdx the row to update
 * @param loadIdx the column to update
 * @param afr the new AFR value
 */
void updateAFRData(int rpmIdx, int loadIdx, double afr) {
    if (rpmIdx >= FUEL_TABLE_SIZE || loadIdx >= FUEL_TABLE_SIZE) {
        throw std::out_of_range("RPM or Load index out of bounds!");
    }
    // TODO if (rmp > 500 && temp > 75)

    afrSamplesCount++;

    // sum afr values
    loggedSumAfrMap[loadIdx][rpmIdx] += afr;

    // advance number of samples on cell
    loggedNumAfrMap[loadIdx][rpmIdx]++;
}

/**
 * Calculates the new fuel map based on the logged AFR.
 *
 * @return the number of cells that have changed in the new map
 */
int calculateNewFuelMap() {
    int cellsChanged = 0;
    for (int row = 0; row < FUEL_TABLE_SIZE; row++) {
        for (int col = 0; col < FUEL_TABLE_SIZE; col++) {
            if (loggedNumAfrMap[row][col] >= minCellSamples) {
                // enough samples logged; re-calc fuel
                const double loggedAvgAfr = loggedSumAfrMap[row][col] / loggedNumAfrMap[row][col];
                if (abs(loggedAvgAfr - targetAFR) >= MIN_AFR_DELTA) {
                    const double currentFuel = currentFuelMap[row][col];
                    const double newFuel = (loggedAvgAfr / targetAFR) * currentFuel;
                    // Make sure that no huge changes are made in the fuel map at once
                    if (abs(newFuel - currentFuel) / currentFuel <= MAX_FUEL_PERCENTAGE_CHANGE) {
                        newFuelMap[row][col] = newFuel;
                    } else {
                        const double maxFuelDelta = (newFuel < currentFuel) ? -MAX_FUEL_PERCENTAGE_CHANGE * currentFuel : MAX_FUEL_PERCENTAGE_CHANGE * currentFuel;
                        newFuelMap[row][col] = maxFuelDelta + currentFuel;
                    }
                    cellsChanged++;
                }
            }
        }
    }
    return cellsChanged;
}

/**
 * Copies the newFuelMap to the currentFuelMap and resets the AFR samples where needed.
 * To be used after sending the newFuelMap to PFC.
 */
void syncFuelTablesAndAfrData() {
    for (int row = 0; row < 20; row++) {
        for (int col = 0; col < 20; col++) {
            if (newFuelMap[row][col] != currentFuelMap[row][col]) {
                currentFuelMap[row][col] = newFuelMap[row][col];
                // for each cell that is written to PFC reset the AFR samples.
                loggedSumAfrMap[row][col] = 0;
                loggedNumAfrMap[row][col] = 0;
            }
        }
    }
}

/**
 * Decides whether the new fuel map should be sent to PFC.
 * Also, updates the fuelMapWriteRequest because the map is sent in chunks to the PFC.
 *
 * @param log when true the afr and new fuel table data will be logged
 */
bool handleNextFuelMapWriteRequest(bool log) {
    if (fuelMapWriteRequest == 0) {
        // not writing (fuelMapWriteRequest == 0) and its time to attempt
        if (afrSamplesCount % fuelMapWriteAttemptInterval == 0 &&
            calculateNewFuelMap() >= minCellsChangesForWriteAttempt) {
            // this is the first write request of this cycle
            fuelMapWriteRequest = 1;

            // update the stats
            mapWriteCount++;

            if (log) {
                logFuelData();
            }
            return true;
        } else {
            return false;
        }
    } else if (fuelMapWriteRequest >= FUEL_MAP_TOTAL_REQUESTS) {
        // this was the last write request
        syncFuelTablesAndAfrData();
        fuelMapWriteRequest = 0;
        return false;
    } else {
        // fuelMapWriteRequest = 1..FUEL_MAP_TOTAL_REQUESTS, continue with the next fuel write request
        fuelMapWriteRequest++;
        return true;
    }
}

/**
 * Gets the value of the current fuel map in the provided row/column.
 * @param row the row of the map
 * @param col the column of the map
 * @return the value of the current fuel map in the provided row/column.
 */
double getCurrentFuel(int row, int col) {
    return currentFuelMap[row][col];
}

/**
 * Gets the value of the new fuel map in the provided row/column.
 * @param row the row of the map
 * @param col the column of the map
 * @return the value of the current fuel map in the provided row/column.
 */
double getNewFuel(int row, int col){
    return newFuelMap[row][col];
}

void printLoggedAfrAvg() {
    cout << "\n== Logged AFR avg ==" << endl;
    for(int r=0; r<FUEL_TABLE_SIZE; r++) {
        for(int c=0; c<FUEL_TABLE_SIZE; c++) {
            const double avgAfr = loggedNumAfrMap[r][c] > 0 ? loggedSumAfrMap[r][c] / loggedNumAfrMap[r][c] : 0;
            cout << setw(4) << fixed << setprecision(1) << avgAfr
                 << "(" << setw(4) << loggedNumAfrMap[r][c] << ")";
            if (c < 19) {
                cout << ",";
            }
        }
        cout << endl;
    }
}

void printNewFuelTable() {
    cout << "\n== New Fuel table ==" << endl;
    for(int r=0; r < FUEL_TABLE_SIZE; r++) {
        for(int c=0; c < FUEL_TABLE_SIZE; c++) {
            cout << setw(4) << fixed << setprecision(1) << newFuelMap[r][c]
                 << "(" << newFuelMap[r][c] - currentFuelMap[r][c] << ")";
            if (c < 19) {
                cout << ",";
            }
        }
        cout << endl;
    }
}

void logFuelData() {
    cout << "Total fuel table writes: " << mapWriteCount << endl;
    printLoggedAfrAvg();
    printNewFuelTable();
}
