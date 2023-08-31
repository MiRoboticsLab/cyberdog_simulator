/*!
 * @file utilities.cpp
 * @brief Common utility functions
 */


#include <ctime>
#include <iomanip>
#include <iostream>

#include "utilities/utilities.h"
#include "Configuration.h"


/**
 * @brief Write std::string to file with given name
 *
 * @param fileName
 * @param fileData
 */
void WriteStringToFile( const std::string& fileName, const std::string& fileData ) {
    FILE* fp = fopen( fileName.c_str(), "w" );
    if ( !fp ) {
        printf( "Failed to fopen %s\n", fileName.c_str() );
        throw std::runtime_error( "Failed to open file" );
    }
    fprintf( fp, "%s", fileData.c_str() );
    fclose( fp );
}

/**
 * @brief Get the current time and return date object as a string
 *
 * @return std::string
 */
std::string GetCurrentTimeAndDate() {
    auto               t  = std::time( nullptr );
    auto               tm = *std::localtime( &t );
    std::ostringstream ss;
    ss << std::put_time( &tm, "%c" );
    return ss.str();
}

/**
 * @brief Return the path of config directory
 *
 * @return std::string
 */
std::string GetConfigDirectoryPath() { 
  return std::string( THIS_COM ) + "config/"; 
  }

/**
 * @brief Return the path of config directory of control program
 *
 * @return std::string
 */
std::string GetLocoConfigDirectoryPath() { return std::string( THIS_COM ) + "../../cyberdog_locomotion/common/config/"; }
