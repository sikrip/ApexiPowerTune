#ifndef HELLOCPP_APEXUFUELMAP_H
#define HELLOCPP_APEXUFUELMAP_H

/**
 * The length in bytes of a map write packet.
 */
static const int MAP_WRITE_PACKET_LENGTH = 103;

/**
 * In order to read or write the entire fuel map; 8 requests are required.
 */
static const int FUEL_MAP_TOTAL_REQUESTS = 8;

/**
 * The PFC fuel map has 400 values (20x20). In order to white/read the entire map 8 requests
 * are needed. So each request has 50 values.
 */
static const int FUEL_CELLS_PER_REQUEST = 50;

/**
 * The PFC fuel map is 20x20 rows.
 */
static const int FUEL_TABLE_SIZE = 20;

/**
 * AFR deltas lower than this are considered "close enough".
 */
static const double MIN_AFR_DELTA = 0.15;

/**
 * This is the max percentage change that a single fuel cell can change in one cycle.
 */
static const double MAX_FUEL_PERCENTAGE_CHANGE = 0.3;

#include <sstream>

using namespace std;

void readFuelMap(int fuelRequestNumber, const char* rawData);
char* createFuelMapWritePacket(int fuelRequestNumber, double (&map)[FUEL_TABLE_SIZE][FUEL_TABLE_SIZE]);
char* getNextFuelMapWritePacket();
void updateAFRData(int rpmIdx, int loadIdx, double afr);
bool handleNextFuelMapWriteRequest(int maxWriteRequests);

double getCurrentFuel(int row, int col);
double getNewFuel(int row, int col);
int getCurrentFuelMapWriteRequest();

void logFuelData(int printMapSize);

#endif //HELLOCPP_APEXUFUELMAP_H
