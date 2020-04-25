#ifndef HELLOCPP_APEXUFUELMAP_H
#define HELLOCPP_APEXUFUELMAP_H

/**
 * In order to read or write the entire fuel map; 8 requests are required.
 */
static const int FUEL_MAP_TOTAL_REQUESTS = 8;

/**
 * The PFC fuel map is 20x20 rows.
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
 * This is the max percentage change that a single fuel cell can change in on cycle.
 */
static const double MAX_FUEL_PERCENTAGE_CHANGE = 0.1;

#include <sstream>

using namespace std;

char* createFuelMapWritePacket(int fuelRequestNumber, double (&map)[20][20]);

void readFuelMap(int fuelRequestNumber, const char* rawData);
char* getNextFuelMapWritePacket();
void updateAFRData(int rpmIdx, int loadIdx, double afr);
bool handleNextFuelMapWriteRequest();

double getCurrentFuel(int row, int col);
double getNewFuel(int row, int col);

#endif //HELLOCPP_APEXUFUELMAP_H
