#include "ApexuFuelMap.h"
#include <stdexcept>
#include <iostream>
#include <iomanip>

using namespace std;

/**
 * The initial fuel map.
 * Will be sent to PFC during startup.
 */
double initialFuelMap[FUEL_TABLE_SIZE][FUEL_TABLE_SIZE] = {
 {0.848,0.824,1.176,1.132,1.416,1.500,1.504,1.200,1.284,1.012,1.452,1.512,0.904,0.888,0.908,0.904,0.948,1.024,1.084,1.096},
 {1.300,1.300,1.400,1.500,1.612,1.132,1.480,1.040,1.788,1.600,1.756,1.756,1.400,1.032,1.056,1.052,1.104,1.192,1.260,1.276},
 {1.300,1.300,1.400,1.500,1.700,1.568,1.584,1.544,1.620,2.080,2.220,2.220,1.464,1.400,1.472,1.444,1.504,1.620,1.704,1.744},
 {1.300,1.500,1.600,2.000,2.464,2.276,2.248,2.096,2.000,2.296,2.472,2.472,1.640,1.552,1.648,1.608,1.668,1.796,1.880,1.932},
 {1.348,2.000,2.100,2.600,3.472,2.904,2.696,2.592,2.516,2.656,2.724,2.724,1.820,1.708,1.824,1.772,1.832,1.972,2.060,2.124},
 {1.600,2.624,3.000,3.300,3.284,3.224,3.224,3.188,3.232,3.056,3.044,3.044,2.280,2.212,2.160,2.192,2.224,2.304,2.376,2.408},
 {2.000,3.000,3.500,4.000,4.000,4.364,4.032,3.920,3.856,3.668,3.364,3.364,2.740,2.716,2.500,2.616,2.612,2.632,2.692,2.696},
 {2.596,3.124,5.000,5.000,5.000,4.684,4.608,4.672,4.708,4.672,4.120,4.120,3.652,3.508,3.140,3.232,3.208,3.292,3.396,3.388},
 {2.252,3.608,5.200,5.072,5.828,7.252,6.244,6.296,6.096,5.388,4.976,4.584,4.612,4.212,3.908,3.864,3.856,4.096,4.448,4.428},
 {2.572,4.100,5.000,5.032,6.832,6.764,7.328,7.188,6.304,6.472,5.372,5.220,5.068,4.824,4.952,5.016,4.992,5.016,5.084,5.072},
 {4.548,4.368,4.576,4.992,6.472,8.284,8.440,7.844,7.476,7.220,7.628,7.688,6.908,6.784,6.184,6.540,6.828,7.084,7.108,7.080},
 {5.548,5.980,6.020,6.500,6.600,6.988,8.548,9.884,9.880,8.828,9.208,8.908,8.224,8.976,8.624,8.276,8.328,8.440,8.632,8.656},
 {7.508,7.500,7.500,8.296,8.196,8.100,8.864,8.928,9.532,10.084,9.784,9.848,10.372,10.256,11.100,11.500,11.000,11.000,11.000,11.000},
 {8.480,9.812,9.856,9.940,9.892,9.848,9.800,9.700,9.744,9.536,10.364,10.040,9.856,10.696,11.176,11.500,11.100,11.000,11.000,10.716},
 {8.508,9.184,9.220,9.352,9.284,9.420,9.420,9.692,9.776,10.748,11.352,10.632,12.200,12.700,11.476,12.200,11.668,11.404,11.880,11.460},
 {8.504,8.932,8.964,9.108,9.216,9.324,9.436,9.544,10.228,10.816,11.352,11.480,11.608,12.200,12.380,12.956,12.328,11.768,11.300,11.308},
 {8.480,8.916,8.948,9.152,9.276,9.396,9.520,9.644,10.228,10.816,11.352,11.480,11.608,11.740,12.600,13.500,12.592,12.004,11.448,11.260},
 {8.200,8.636,8.668,9.408,9.560,9.716,9.868,10.024,10.176,10.332,11.352,11.480,11.608,11.740,12.600,13.000,12.500,12.004,11.448,11.260},
 {8.320,8.756,8.788,9.684,9.940,10.164,10.388,10.612,10.756,10.904,11.352,11.480,11.608,11.736,12.600,13.000,12.500,12.004,11.500,11.500},
 {8.520,8.956,8.988,9.884,10.112,10.336,10.560,10.788,11.096,11.224,11.352,11.480,11.608,11.736,12.600,13.000,12.724,12.004,12.000,12.000}
};

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
 * This controls the the writing og the new (after auto tune) map.
 */
int fuelMapWriteRequest;

/**
 * The current request number when writing the map to PFC.
 * This controls the writing og the initial map.
 * 0 -> nothing sent
 * 1..FUEL_MAP_TOTAL_REQUESTS -> corresponds to a chunk of the fuel map
 */
int initialFuelMapWriteRequestNumber = 0;

/**
 * Attempt to re-calc and write the map after this number of samples.
 */
int fuelMapWriteAttemptInterval = 50;

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
int minCellsChangesForWriteAttempt = 1;

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
                    const double newFuel = (loggedAvgAfr / targetAFR) * currentFuelMap[row][col];
                    newFuelMap[row][col] = newFuel;
                    cellsChanged++;
                    cout << "Changing fuel at row:" << row
                         << " col:" << col
                         << " from:" << currentFuelMap[row][col]
                         << " to:" << newFuelMap[row][col] << endl;
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
                // for each cell that is written to PFC as exact match reset the AFR samples.
                if (loggedNumAfrMap[row][col] >= minCellSamples) {
                    // this will exclude cells changed as neighbor cells
                    loggedSumAfrMap[row][col] = 0;
                    loggedNumAfrMap[row][col] = 0;
                }
            }
        }
    }
}

/**
 * Decides whether the new fuel map should be sent to PFC.
 * Also, updates the fuelMapWriteRequest because the map is sent in chunks to the PFC.
 *
 * @param maxWriteRequests value lower than FUEL_MAP_TOTAL_REQUESTS will not write the entire fuel map
 */
bool handleNextFuelMapWriteRequest(int maxWriteRequests) {
    if (maxWriteRequests > FUEL_MAP_TOTAL_REQUESTS) {
        maxWriteRequests = FUEL_MAP_TOTAL_REQUESTS;
    }
    if (fuelMapWriteRequest == 0) {
        // not writing (fuelMapWriteRequest == 0) and its time to attempt
        if (afrSamplesCount % fuelMapWriteAttemptInterval == 0 &&
            calculateNewFuelMap() >= minCellsChangesForWriteAttempt) {
            // this is the first write request of this cycle
            fuelMapWriteRequest = 1;

            // update the stats
            mapWriteCount++;
            return true;
        } else {
            return false;
        }
    } else if (fuelMapWriteRequest >= maxWriteRequests) {
        // this was the last write request
        syncFuelTablesAndAfrData();
        fuelMapWriteRequest = 0;
        return false;
    } else {
        // fuelMapWriteRequest = 1..maxWriteRequests, continue with the next fuel write request
        fuelMapWriteRequest++;
        return true;
    }
}

/**
 * Gets the next write packet for writing the initial fuel map to the PFC.
 * An empty string means no further writing.
 */
char* getInitialFuelMapNextWritePacket() {
    if (initialFuelMapWriteRequestNumber <= INITIAL_FUEL_MAP_MAX_REQUESTS) {
        cout << "Sending request " << initialFuelMapWriteRequestNumber << " of initial fuel map." << endl;
        return createFuelMapWritePacket(++initialFuelMapWriteRequestNumber, initialFuelMap);
    }
    return nullptr;
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

int getCurrentFuelMapWriteRequest() {
    return fuelMapWriteRequest;
}

void printCurrentFuelTable() {
    cout << "\n== Current Fuel table ==" << endl;
    for(int r=0; r < FUEL_TABLE_SIZE; r++) {
        for(int c=0; c < FUEL_TABLE_SIZE; c++) {
            // 2 digits + 1 dot + 3 decimal places
            cout << setw(6) << fixed << setprecision(3) << currentFuelMap[r][c];
            if (c < FUEL_TABLE_SIZE-1) {
                cout << " | ";
            }
        }
        cout << endl;
    }
}

void printLoggedAfrAvg(int printMapSize) {
    cout << "\n== Logged AFR avg ==" << endl;
    for(int r=0; r < printMapSize; r++) {
        for(int c=0; c < printMapSize; c++) {
            const double avgAfr = loggedNumAfrMap[r][c] > 0 ? loggedSumAfrMap[r][c] / loggedNumAfrMap[r][c] : 0;
            cout << setw(4) << fixed << setprecision(1) << avgAfr
                 << "(" << setw(4) << loggedNumAfrMap[r][c] << ")";
            if (c < printMapSize-1) {
                cout << " | ";
            }
        }
        cout << endl;
    }
}

void printFuelAndDelta(double fuel, double delta) {
    cout << setw(6) << fixed << setprecision(3) << fuel << "(";
    if (delta !=0) {
        cout << setw(6) << fixed << setprecision(3) << delta << ")";
    } else {
        cout << setw(6) << "" << ")";
    }
}

void printNewFuelTable(int printMapSize) {
    cout << "\n== New Fuel table ==" << endl;
    for(int r=0; r < printMapSize; r++) {
        for(int c=0; c < printMapSize; c++) {
            printFuelAndDelta(newFuelMap[r][c], newFuelMap[r][c] - currentFuelMap[r][c]);
            if (c < printMapSize-1) {
                cout << " | ";
            }
        }
        cout << endl;
    }
}

void logFuelData(int printMapSize) {
    cout << "== Total fuel table writes: " << mapWriteCount << " ==" << endl;
    // Do not print entire map as only the lower rmp/load is auto - tuned.
    printLoggedAfrAvg(printMapSize);
    printNewFuelTable(printMapSize);
}

/*int main(int argc, char *argv[]) {
    printFuelAndDelta(12.345, 0.456);
    cout << endl;
    printFuelAndDelta(2.345, -0.456);
    cout << endl;
    printFuelAndDelta(2.345, 0);
    return 0;
}*/
